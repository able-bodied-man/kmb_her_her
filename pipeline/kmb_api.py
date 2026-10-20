#!/usr/bin/env python3
"""KMB/LWB open-data API helpers (data.etabus.gov.hk).

The API covers both KMB and LWB under the /kmb/ namespace. It gives us the two
things the TD GTFS feed lacks: Traditional Chinese names, and real-time arrivals
(used to calibrate per-segment run times in CI).

Stop ids here are 16-hex strings and are a *different namespace* from the TD GTFS
stop ids (plain integers), so the two sets are joined by coordinate proximity.

Stdlib only.
"""
from __future__ import annotations

import json
import math
import os
import time
import urllib.error
import urllib.request

BASE = "https://data.etabus.gov.hk/v1/transport/kmb"
CACHE = os.environ.get("KHH_CACHE", ".gtfs-cache")
EARTH_R = 6371000.0


def _cache_path(name: str) -> str:
    os.makedirs(CACHE, exist_ok=True)
    return os.path.join(CACHE, f"api_{name}.json")


def get(path: str, *, cache_name: str | None = None, ttl: float | None = None,
        retries: int = 3) -> dict:
    """GET a JSON endpoint, optionally through a disk cache."""
    if cache_name:
        p = _cache_path(cache_name)
        if os.path.exists(p):
            fresh = ttl is None or (time.time() - os.path.getmtime(p)) < ttl
            if fresh and os.path.getsize(p) > 0:
                with open(p, encoding="utf-8") as f:
                    return json.load(f)
    url = BASE + path
    last = None
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(url, timeout=45) as r:
                data = json.loads(r.read().decode())
            if cache_name:
                tmp = _cache_path(cache_name) + ".part"
                with open(tmp, "w", encoding="utf-8") as f:
                    json.dump(data, f)
                os.replace(tmp, _cache_path(cache_name))
            return data
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
            last = e
            time.sleep(1.5 * (attempt + 1))
    raise RuntimeError(f"GET {url} failed: {last}")


def stops() -> list[dict]:
    return get("/stop", cache_name="stops", ttl=6 * 3600)["data"]


def routes() -> list[dict]:
    return get("/route", cache_name="routes", ttl=6 * 3600)["data"]


def route_stops(route: str, bound: str, service_type: str) -> list[dict]:
    return get(f"/route-stop/{route}/{bound}/{service_type}")["data"]


def eta(stop_id: str, route: str, service_type: str = "1") -> list[dict]:
    return get(f"/eta/{stop_id}/{route}/{service_type}")["data"]


def haversine(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, a ** 0.5))


def match_stops(td_stops: list[list]) -> dict[str, dict]:
    """td stop_id -> {'stop','name_en','name_tc','name_sc','lat','long','dist'}

    Nearest-neighbour match by coordinate within MATCH_RADIUS_M. Both feeds are
    kerbside-level, so the match is essentially 1:1; stops that are simply not in
    the operator feed (other operators' kerbs) stay unmatched.
    """
    MATCH_RADIUS_M = 40.0
    api = stops()
    cell = 0.001                                    # ~110 m
    grid: dict[tuple[int, int], list[dict]] = {}
    for s in api:
        grid.setdefault((int(float(s["lat"]) / cell), int(float(s["long"]) / cell)), []).append(s)

    out: dict[str, dict] = {}
    for sid, lat, lon, _name in td_stops:
        gy, gx = int(lat / cell), int(lon / cell)
        found: list[tuple[float, dict]] = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                for s in grid.get((gy + dy, gx + dx), ()):
                    d = haversine(lat, lon, float(s["lat"]), float(s["long"]))
                    if d <= MATCH_RADIUS_M:
                        found.append((d, s))
        if not found:
            continue
        found.sort(key=lambda x: x[0])
        best_d, best = found[0]
        out[str(sid)] = {
            "stop": best["stop"],
            "name_en": best["name_en"],
            "name_tc": best["name_tc"],
            "name_sc": best["name_sc"],
            "lat": float(best["lat"]),
            "long": float(best["long"]),
            "dist": round(best_d, 1),
            # every operator stop within the radius: big interchanges have several
            # kerbs, and the one belonging to the route being matched is not
            # necessarily the nearest one
            "near": [s["stop"] for _d, s in found],
        }
    return out
