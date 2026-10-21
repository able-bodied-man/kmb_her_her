#!/usr/bin/env python3
"""
kmb_her_her data pipeline.

Source: Transport Department "Headway information of public transport services"
(https://static.data.gov.hk/td/pt-headway-en/, GTFS, refreshed biweekly).

That feed covers every HK mode (MTR, Citybus, minibus, tram, ferry, KMB, LWB ...).
This pipeline reduces it to what a KMB/LWB pass holder can actually ride —
KMB, LWB and the jointly operated KMB+CTB / LWB+CTB routes — and reshapes it
into compact arrays a browser can load and route over.

IMPORTANT PROPERTY OF THE SOURCE FEED
TD publishes only the *first and last* stop time of each trip (plus a handful of
timepoints); every intermediate stop is blank. Journey duration per trip is
therefore known exactly, but intermediate times have to be reconstructed:
this pipeline spreads the measured duration over the stops proportionally to
inter-stop distance, with a fixed dwell allowance per stop. Intermediate times
are therefore ESTIMATES (typically ±1-2 min); departure times at the first stop
and total run time are exact.

Output (written to <repo>/data/):
  meta.json        counts, feed timestamp, bounds, caveats
  stops.json       [[stop_id, lat, lon, name], ...]
  routes.json      [[route_id, short_name, long_name, agency_id], ...]
  calendar.json    {services:[...], calendar:{...}, exceptions:[...]}
  trips.bin        columnar: u16 route, u8 service, u32 start_sec,
                             u32 stop_time_offset, u16 n_stops
  stoptimes.bin    columnar: u16 (stop_delta<<3 | flags), u16 seconds_since_start
  walk.bin         walking-transfer index: u32 offsets + (u16 stop, u16 metres)

Stdlib only, so it runs on a bare CI runner.
"""
from __future__ import annotations

import csv
import json
import math
import os
import re
import struct
import sys
import urllib.request
from array import array
from datetime import datetime, timedelta, timezone

FEED_BASE = "https://static.data.gov.hk/td/pt-headway-en"
FEED_FILES = [
    "agency.txt", "routes.txt", "stops.txt", "trips.txt", "stop_times.txt",
    "calendar.txt", "calendar_dates.txt", "frequencies.txt",
]

# Agencies a KMB/LWB pass holder can ride. The joint KMB+CTB / LWB+CTB routes
# are partly operated by KMB/LWB, so they are offered (agency is visible in
# routes.json so the UI can flag them).
ALLOWED_AGENCIES = {"KMB", "LWB", "KMB+CTB", "LWB+CTB"}

WALK_RADIUS_M = 400.0          # stop <-> stop walking transfer
DWELL_SECS = 20                # assumed dwell at each intermediate stop
DEFAULT_KMH = 19.0             # fallback speed when a trip has a single timestamp
MAX_BUS_KMH = 45.0             # anything faster than this is a bad published duration
EARTH_R = 6371000.0

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CACHE = os.environ.get("KHH_CACHE", os.path.join(ROOT, ".gtfs-cache"))
OUT = os.path.join(ROOT, "data")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kmb_api  # noqa: E402  (local module, needs the path insert above)


def log(msg: str) -> None:
    print(msg, flush=True)


def data_digest(out_dir: str) -> str:
    """Hash of the payload files, so the client can tell a real data change from
    a rebuild that produced identical output (the CI redeploys every 30 min)."""
    import hashlib
    h = hashlib.sha256()
    for name in sorted(os.listdir(out_dir)):
        if name == "meta.json":
            continue
        p = os.path.join(out_dir, name)
        if not os.path.isfile(p):
            continue
        h.update(name.encode())
        with open(p, "rb") as f:
            for chunk in iter(lambda: f.read(1 << 20), b""):
                h.update(chunk)
    return h.hexdigest()[:16]


def fetch(name: str) -> str:
    os.makedirs(CACHE, exist_ok=True)
    path = os.path.join(CACHE, name)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        url = f"{FEED_BASE}/{name}"
        log(f"  downloading {name} ...")
        tmp = path + ".part"
        with urllib.request.urlopen(url, timeout=300) as r, open(tmp, "wb") as f:
            while chunk := r.read(1 << 20):
                f.write(chunk)
        os.replace(tmp, path)
    return path


def rows(name: str):
    with open(fetch(name), encoding="utf-8-sig", newline="") as f:
        yield from csv.DictReader(f)


def gtfs_secs(t: str) -> int:
    h, m, s = (int(x) for x in t.split(":"))
    return h * 3600 + m * 60 + s


def clean_stop_name(raw: str) -> str:
    """TD packs one name per agency into stop_name, pipe separated."""
    parts = [p.strip() for p in raw.split("|")]
    for pref in ("[KMB]", "[LWB]", "[KMB+CTB]", "[LWB+CTB]"):
        for p in parts:
            if p.startswith(pref):
                return re.sub(r"\s+", " ", p[len(pref):].replace("<BR>", " ")).strip(" /")
    first = parts[0] if parts else raw
    return re.sub(r"\s+", " ", re.sub(r"^\[[^\]]+\]\s*", "", first).replace("<BR>", " ")).strip()


def haversine(lat1, lon1, lat2, lon2) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, a ** 0.5))


def segment_kmh(dist_m: float) -> float:
    """Plausible average speed for a bus over one inter-stop hop.

    Short hops are dense urban stops (slow), long hops are expressway runs
    (fast). This is what keeps a 9 km Kowloon slice from being scheduled in
    12 minutes just because the published trip duration is optimistic.
    """
    if dist_m < 250:
        return 18.0
    if dist_m < 600:
        return 24.0
    if dist_m < 1200:
        return 32.0
    if dist_m < 2500:
        return 42.0
    return 52.0


def interpolate(secs: list[int | None], lats: list[float], lons: list[float]) -> tuple[int, bool]:
    """Fill the blank times of one trip in place.

    Returns (how many times were filled, whether the trip had to be stretched
    because the published duration is faster than any bus can drive).

    Known times (first stop, last stop, occasional timepoints) are kept verbatim
    when the published gap is long enough. The gap is shared out in proportion to
    a speed-profile model of each hop (short hop = slow, long hop = fast, plus a
    dwell allowance), so intermediate times reflect how the trip actually runs
    instead of a uniform average.
    """
    n = len(secs)
    if n == 0:
        return 0, False
    anchors = [i for i, s in enumerate(secs) if s is not None]

    cum = [0.0]
    for i in range(1, n):
        cum.append(cum[-1] + haversine(lats[i - 1], lons[i - 1], lats[i], lons[i]))

    # model seconds for each hop, and cumulative model time from the trip start
    model = [0.0]
    for i in range(1, n):
        d = cum[i] - cum[i - 1]
        model.append(model[-1] + d / (segment_kmh(d) / 3.6) + DWELL_SECS)

    filled = 0
    stretched = False
    if not anchors:
        return 0, False
    if len(anchors) == 1:
        a = anchors[0]
        for i in range(n):
            if i == a:
                continue
            secs[i] = int(round(secs[a] + (model[i] - model[a])))
            filled += 1
    else:
        first = anchors[0]
        for i in range(first):                        # before the first timestamp
            secs[i] = int(round(secs[first] - (model[first] - model[i])))
            filled += 1
        for lo, hi in zip(anchors, anchors[1:]):
            if hi - lo == 1:
                continue
            gap = secs[hi] - secs[lo]
            model_gap = model[hi] - model[lo]
            inner = list(range(lo + 1, hi))
            if model_gap <= 0 or len(inner) == 0:
                continue
            fit = gap / model_gap
            for i in inner:
                secs[i] = int(round(secs[lo] + (model[i] - model[lo]) * fit))
                filled += 1
            if fit < 0.92:                            # published gap faster than any bus: keep the model
                stretched = True
                for i in inner:
                    secs[i] = int(round(secs[lo] + (model[i] - model[lo])))
                secs[hi] = max(secs[hi], int(round(secs[lo] + model_gap)))
        last = anchors[-1]
        for i in range(last + 1, n):                  # after the last timestamp
            secs[i] = int(round(secs[last] + (model[i] - model[last])))
            filled += 1
    # enforce monotonic, non-negative times
    prev = -1
    for i in range(n):
        v = int(secs[i])
        if v < prev:
            v = prev
        if v < 0:
            v = 0
        secs[i] = v
        prev = v
    return filled, stretched


def main() -> int:
    os.makedirs(OUT, exist_ok=True)

    log("[1/7] agencies & routes")
    agencies = {r["agency_id"]: r["agency_name"] for r in rows("agency.txt")}
    routes = [r for r in rows("routes.txt") if r["agency_id"] in ALLOWED_AGENCIES]
    route_ids = {r["route_id"] for r in routes}
    log(f"      {len(routes)} routes from {sorted(ALLOWED_AGENCIES)}")

    log("[2/7] trips")
    trips = [t for t in rows("trips.txt") if t["route_id"] in route_ids]
    trip_ids = {t["trip_id"] for t in trips}
    service_ids = sorted({t["service_id"] for t in trips})
    log(f"      {len(trips)} trips, {len(service_ids)} service ids")

    log("[3/7] stops")
    coords: dict[str, tuple[float, float]] = {}
    names: dict[str, str] = {}
    for s in rows("stops.txt"):
        coords[s["stop_id"]] = (float(s["stop_lat"]), float(s["stop_lon"]))
        names[s["stop_id"]] = clean_stop_name(s["stop_name"])
    log(f"      {len(coords)} stops in feed")

    log("[4/7] stop_times (the 50 MB file; 91% of times are blank and get reconstructed)")
    raw: dict[str, list[tuple[int, str, str, str, int, int]]] = {}
    for st in rows("stop_times.txt"):
        tid = st["trip_id"]
        if tid not in trip_ids:
            continue
        raw.setdefault(tid, []).append((
            int(st["stop_sequence"]),
            st["arrival_time"] or st["departure_time"] or "",
            st["stop_id"],
            int(st["pickup_type"] or 0),
            int(st["drop_off_type"] or 0),
        ))

    stop_times: dict[str, list[tuple[int, int, str, int, int]]] = {}
    blanks = 0
    scaled = 0
    worst = []
    for tid, arr in raw.items():
        arr.sort(key=lambda x: x[0])
        if not arr:
            continue
        lats = [coords[r[2]][0] for r in arr]
        lons = [coords[r[2]][1] for r in arr]
        secs: list[int | None] = [gtfs_secs(r[1]) if r[1] else None for r in arr]
        filled, was_stretched = interpolate(secs, lats, lons)
        blanks += filled
        # Second plausibility pass: a route with urban stop spacing cannot average
        # expressway speed, however fast the operator's published trip duration
        # looks (HK1's sightseeing loop claims ~47 km/h down Nathan Road).
        hops = sorted(haversine(lats[i - 1], lons[i - 1], lats[i], lons[i])
                      for i in range(1, len(arr)))
        total_d = sum(hops)
        dur = (secs[-1] or 0) - (secs[0] or 0)
        if hops and total_d > 1500 and dur > 0:
            median_hop = hops[len(hops) // 2]
            # straight-line sums understate road distance; 1.5x is a conservative
            # detour factor for a bus route (measured road/straight ratio on our
            # own shapes is ~1.4-1.6)
            implied = (total_d * 1.5 / 1000) / (dur / 3600)
            if implied > 42.0 and median_hop < 1800:
                factor = implied / 38.0
                base = secs[0] or 0
                for i in range(len(secs)):
                    secs[i] = int(round(base + ((secs[i] or 0) - base) * factor))
                was_stretched = True
        if was_stretched:
            scaled += 1
            if len(worst) < 5:
                worst.append((tid, round(total_d / 1000, 1), (secs[-1] or 0) // 60))
        stop_times[tid] = [(r[0], secs[i], r[2], r[3], r[4]) for i, r in enumerate(arr)]  # type: ignore[list-item]

    used_stop_ids = {st[2] for arr in stop_times.values() for st in arr}
    n_rows = sum(len(v) for v in stop_times.values())
    log(f"      {n_rows} rows, reconstructed {blanks} intermediate times over {len(used_stop_ids)} stops")
    if scaled:
        log(f"      stretched {scaled} trips with implausible published durations, e.g. {worst}")

    log("[5/8] stop index")
    stop_list, stop_index = [], {}
    for sid in sorted(used_stop_ids, key=lambda x: int(x) if x.isdigit() else 0):
        lat, lon = coords[sid]
        stop_index[sid] = len(stop_list)
        stop_list.append([sid, round(lat, 5), round(lon, 5), names[sid]])

    log("[5/8] calendars")
    calendar = {}
    for c in rows("calendar.txt"):
        if c["service_id"] not in service_ids:
            continue
        bits = sum((int(c[d]) << i) for i, d in enumerate(
            ("monday", "tuesday", "wednesday", "thursday", "friday", "saturday", "sunday")))
        calendar[c["service_id"]] = [bits, int(c["start_date"]), int(c["end_date"])]
    cal_dates = [[c["service_id"], int(c["date"]), int(c["exception_type"])]
                 for c in rows("calendar_dates.txt") if c["service_id"] in service_ids]

    log("[7/8] Chinese names + headsigns from the KMB/LWB API")
    matched = kmb_api.match_stops(stop_list)
    for row in stop_list:
        hit = matched.get(str(row[0]))
        row.append(hit["name_tc"] if hit else None)
    log(f"      {len(matched)} / {len(stop_list)} stops matched to the operator feed "
        f"({100 * len(matched) / len(stop_list):.1f}%)")

    # route-bound-service variants, with their Chinese terminu
    variants: dict[str, list[dict]] = {}
    api_routes = {(r["route"], r["bound"], r["service_type"]): r for r in kmb_api.routes()}
    seqs_rows: dict[tuple[str, str, str], list[tuple[int, str]]] = {}
    for rs in kmb_api.get("/route-stop", cache_name="all_route_stops", ttl=24 * 3600)["data"]:
        seqs_rows.setdefault((rs["route"], rs["bound"], rs["service_type"]), []).append(
            (int(rs["seq"]), rs["stop"]))
    seqs: dict[tuple[str, str, str], list[str]] = {
        k: [s for _n, s in sorted(v)] for k, v in seqs_rows.items()}
    for key, stops_in_order in seqs.items():
        meta_r = api_routes.get(key)
        if not meta_r:
            continue
        variants.setdefault(meta_r["route"], []).append({
            "bound": meta_r["bound"],
            "service_type": meta_r["service_type"],
            "first": stops_in_order[0],
            "last": stops_in_order[-1],
            "orig_tc": meta_r["orig_tc"],
            "dest_tc": meta_r["dest_tc"],
        })

    headsigns: list[list] = []
    headsign_index: dict[tuple, int] = {}
    trip_headsign: dict[str, int] = {}
    ambiguous = 0
    route_short = {r["route_id"]: r["route_short_name"] for r in routes}

    def order_score(trip_stop_ids: list[str], variant_stops: list[str]) -> tuple[int, int]:
        """How many of the trip's stops the variant contains, in order, and the
        variant index the trip reached.

        Each TD stop may sit near several operator kerbs, so the trip stop is
        aligned to whichever nearby kerb the variant actually serves.
        """
        pos = {sid: i for i, sid in enumerate(variant_stops)}
        last, hit = -1, 0
        for sid in trip_stop_ids:
            hit_info = matched.get(sid)
            if not hit_info:
                continue
            best = -1
            for op_stop in hit_info["near"]:
                i = pos.get(op_stop)
                if i is not None and last < i and (best < 0 or i < best):
                    best = i
            if best >= 0:
                hit += 1
                last = best
        return hit, last

    end_ok = 0

    for trip in trips:
        arr = stop_times.get(trip["trip_id"])
        if not arr:
            continue
        arr.sort(key=lambda x: x[0])
        cands = variants.get(route_short[trip["route_id"]], [])
        if not cands:
            continue
        trip_stops = [sid for _seq, _sec, sid, _pu, _do in arr]
        best, best_score, best_last = None, 0, -1
        for v in cands:
            vstops = seqs[(route_short[trip["route_id"]], v["bound"], v["service_type"])]
            sc, last = order_score(trip_stops, vstops)
            if sc > best_score:
                best, best_score, best_last = v, sc, last
            elif sc == best_score and sc > 0:
                ambiguous += 1
        if best is None or best_score < max(2, len(trip_stops) // 2):
            continue
        vlen = len(seqs[(route_short[trip["route_id"]], best["bound"], best["service_type"])])
        if best_last >= vlen - 2:                 # trip really runs to that terminus
            end_ok += 1
        key = (route_short[trip["route_id"]], best["bound"], best["orig_tc"], best["dest_tc"])
        idx = headsign_index.get(key)
        if idx is None:
            idx = len(headsigns)
            headsign_index[key] = idx
            headsigns.append([key[0], key[1], key[2], key[3]])
        trip_headsign[trip["trip_id"]] = idx
    log(f"      {len(trip_headsign)} / {len(trips)} trips carry a Chinese headsign "
        f"({len(headsigns)} distinct, {end_ok} reach the terminus that names them, "
        f"{ambiguous} ambiguous matches)")
    headsign_end_ok = end_ok

    log("[8/8] packing")
    route_order = sorted(routes, key=lambda r: (r["agency_id"], r["route_short_name"], r["route_id"]))
    route_index = {r["route_id"]: i for i, r in enumerate(route_order)}
    service_index = {sid: i for i, sid in enumerate(service_ids)}

    t_route, t_service, t_start = array("H"), array("B"), array("I")
    t_offset, t_nstops, t_headsign = array("I"), array("H"), array("H")

    st_stop, st_arr = array("H"), array("H")
    trip_stop_lists: list[tuple[int, ...]] = []
    trip_index_by_id: dict[str, int] = {}

    n_stopts = 0
    for trip in trips:
        arr = stop_times.get(trip["trip_id"])
        if not arr:
            continue
        trip_index_by_id[trip["trip_id"]] = len(t_route)
        arr.sort(key=lambda x: x[0])
        start = arr[0][1]
        t_route.append(route_index[trip["route_id"]])
        t_service.append(service_index[trip["service_id"]])
        t_start.append(start)
        t_offset.append(n_stopts)
        t_nstops.append(len(arr))
        t_headsign.append(trip_headsign.get(trip["trip_id"], 0xFFFF))
        prev = 0
        idx_list: list[int] = []
        for seq, sec, sid, pickup, dropoff in arr:
            idx = stop_index[sid]
            idx_list.append(idx)
            delta = idx - prev + 3605                      # 0..7210 -> 13 bits
            flags = (1 if pickup == 0 else 0) | (2 if dropoff == 0 else 0)
            st_stop.append((delta & 0x1FFF) | (flags << 13))
            st_arr.append(max(0, min(65535, sec - start)))
            prev = idx
            n_stopts += 1
        trip_stop_lists.append(tuple(idx_list))

    with open(os.path.join(OUT, "trips.bin"), "wb") as f:
        f.write(struct.pack("<4sII", b"KHH2", len(t_route), n_stopts))
        t_route.tofile(f); t_service.tofile(f); t_start.tofile(f)
        t_offset.tofile(f); t_nstops.tofile(f); t_headsign.tofile(f)
    with open(os.path.join(OUT, "stoptimes.bin"), "wb") as f:
        f.write(struct.pack("<4sI", b"KHT2", n_stopts))
        st_stop.tofile(f); st_arr.tofile(f)

    # ------------------------------------------------------------- frequencies
    # The feed models frequent service as a template trip plus a window:
    # "this trip every 540 s between 07:12 and 07:39". Expanding those into real
    # trips would multiply the stop_times by ~5, so the windows are shipped as a
    # small table and expanded at query time by the router and the arrival chips.
    log("      packing frequency windows")
    fr_rows: list[tuple[int, int, int, int]] = []
    for row in rows("frequencies.txt"):
        ti = trip_index_by_id.get(row["trip_id"])
        if ti is None:
            continue
        fr_rows.append((ti, gtfs_secs(row["start_time"]), gtfs_secs(row["end_time"]),
                        max(30, int(row["headway_secs"]))))
    fr_rows.sort()
    fr_trip = array("I", [r[0] for r in fr_rows])
    fr_start = array("I", [r[1] for r in fr_rows])
    fr_end = array("I", [r[2] for r in fr_rows])
    fr_head = array("H", [min(65535, r[3]) for r in fr_rows])
    with open(os.path.join(OUT, "frequencies.bin"), "wb") as f:
        f.write(struct.pack("<4sI", b"KHF1", len(fr_rows)))
        fr_trip.tofile(f); fr_start.tofile(f); fr_end.tofile(f); fr_head.tofile(f)
    n_fr_trips = len({r[0] for r in fr_rows})
    log(f"      {len(fr_rows)} frequency windows over {n_fr_trips} trips "
        f"({100 * n_fr_trips / max(1, len(t_route)):.1f}% of trips)")

    # ---------------------------------------------------------------- patterns
    # Every distinct (route, ordered stop sequence) is a pattern; its trips are
    # stored sorted by departure so a routing round can binary-search the first
    # trip that can still be caught.
    log("      grouping trips into patterns")
    pattern_key: dict[tuple[int, tuple[int, ...]], int] = {}
    p_route = array("H")
    p_stops: list[tuple[int, ...]] = []
    p_trips: list[list[int]] = []
    for trip_idx, stops_tuple in enumerate(trip_stop_lists):
        key = (t_route[trip_idx], stops_tuple)
        pi = pattern_key.get(key)
        if pi is None:
            pi = len(p_stops)
            pattern_key[key] = pi
            p_route.append(t_route[trip_idx])
            p_stops.append(stops_tuple)
            p_trips.append([])
        p_trips[pi].append(trip_idx)
    for lst in p_trips:
        lst.sort(key=lambda ti: t_start[ti])

    p_stop_off = array("I", [0])
    p_trip_off = array("I", [0])
    pat_stops = array("H")
    pat_trips = array("I")
    sp_off = array("I", [0])
    sp_pair = array("I")
    inc_by_stop: list[list[tuple[int, int]]] = [[] for _ in range(len(stop_list))]
    for pi, stops_tuple in enumerate(p_stops):
        for pos, sidx in enumerate(stops_tuple):
            pat_stops.append(sidx)
            inc_by_stop[sidx].append((pi, pos))
        p_stop_off.append(len(pat_stops))
        for ti in p_trips[pi]:
            pat_trips.append(ti)
        p_trip_off.append(len(pat_trips))

    for sidx in range(len(stop_list)):
        for pi, pos in inc_by_stop[sidx]:
            sp_pair.append((pi << 8) | pos)
        sp_off.append(len(sp_pair))

    with open(os.path.join(OUT, "patterns.bin"), "wb") as f:
        f.write(struct.pack("<4sIII", b"KHP1", len(p_stops), len(pat_stops), len(pat_trips)))
        p_route.tofile(f); p_stop_off.tofile(f); p_trip_off.tofile(f)
        pat_stops.tofile(f); pat_trips.tofile(f)
    with open(os.path.join(OUT, "stoppatterns.bin"), "wb") as f:
        f.write(struct.pack("<4sII", b"KPS1", len(stop_list), len(sp_pair)))
        sp_off.tofile(f); sp_pair.tofile(f)
    log(f"      {len(p_stops)} patterns, {len(pat_stops)} pattern-stop entries, "
        f"{len(sp_pair)} stop->pattern incidences")

    # walking transfers: every pair of distinct stops within WALK_RADIUS_M
    lats = [s[1] for s in stop_list]
    lons = [s[2] for s in stop_list]
    cell = 0.005
    grid: dict[tuple[int, int], list[int]] = {}
    for i, (la, lo) in enumerate(zip(lats, lons)):
        grid.setdefault((int(la / cell), int(lo / cell)), []).append(i)
    w_off = array("I", [0])
    w_pairs, w_metres = array("H"), array("H")
    n_walk = 0
    for i, (la, lo) in enumerate(zip(lats, lons)):
        gy, gx = int(la / cell), int(lo / cell)
        found = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                for j in grid.get((gy + dy, gx + dx), ()):
                    if j == i:
                        continue
                    d = haversine(la, lo, lats[j], lons[j])
                    if d <= WALK_RADIUS_M:
                        found.append((j, int(round(d))))
        found.sort(key=lambda x: x[1])
        for j, d in found:
            w_pairs.append(j)
            w_metres.append(min(d, 65535))
            n_walk += 1
        w_off.append(n_walk)
    with open(os.path.join(OUT, "walk.bin"), "wb") as f:
        f.write(struct.pack("<4sII", b"KHW1", len(stop_list), n_walk))
        w_off.tofile(f); w_pairs.tofile(f); w_metres.tofile(f)

    log("[8/8] writing json")
    with open(os.path.join(OUT, "stops.json"), "w", encoding="utf-8") as f:
        json.dump(stop_list, f, ensure_ascii=False, separators=(",", ":"))
    with open(os.path.join(OUT, "routes.json"), "w", encoding="utf-8") as f:
        json.dump([[r["route_id"], r["route_short_name"], r["route_long_name"], r["agency_id"]]
                   for r in route_order], f, ensure_ascii=False, separators=(",", ":"))
    with open(os.path.join(OUT, "headsigns.json"), "w", encoding="utf-8") as f:
        json.dump(headsigns, f, ensure_ascii=False, separators=(",", ":"))
    with open(os.path.join(OUT, "calendar.json"), "w", encoding="utf-8") as f:
        json.dump({"services": service_ids, "calendar": calendar, "exceptions": cal_dates},
                  f, separators=(",", ":"))

    meta = {
        "generated": datetime.now(timezone(timedelta(hours=8))).isoformat(timespec="seconds"),
        "digest": data_digest(OUT),
        "feed": FEED_BASE,
        "agencies": [{"id": a, "name": agencies.get(a, a)} for a in sorted(ALLOWED_AGENCIES)],
        "counts": {"routes": len(route_order), "trips": len(t_route), "stop_times": n_stopts,
                   "stops": len(stop_list), "walk_links": n_walk, "times_reconstructed": blanks,
                   "headsigns": len(headsigns),
                   "stops_with_chinese_name": sum(1 for s in stop_list if len(s) > 4 and s[4]),
                   "trips_with_headsign": sum(1 for h in t_headsign if h != 0xFFFF),
                   "headsign_reaches_terminus": headsign_end_ok,
                   "trips_duration_stretched": scaled,
                   "frequency_windows": len(fr_rows),
                   "trips_with_frequencies": n_fr_trips},
        "names_source": "data.etabus.gov.hk (KMB/LWB open data API), matched by coordinate",
        "walk_radius_m": WALK_RADIUS_M,
        "bounds": [min(lats), min(lons), max(lats), max(lons)],
        "caveats": [
            "Source feed only timestamps the first and last stop of each trip.",
            "Intermediate stop times are reconstructed from measured trip duration "
            "and inter-stop distance (dwell %.0fs/stop); expect +/-1-2 min." % DWELL_SECS,
            "Trip departure times and total run times are exact.",
        ],
    }
    with open(os.path.join(OUT, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    total = sum(os.path.getsize(os.path.join(OUT, n)) for n in os.listdir(OUT))
    for n in sorted(os.listdir(OUT)):
        log(f"      {n:18s} {os.path.getsize(os.path.join(OUT, n)) / 1e6:6.2f} MB")
    log(f"      {'TOTAL':18s} {total / 1e6:6.2f} MB  |  "
        f"{meta['counts']['routes']} routes, {meta['counts']['trips']} trips, "
        f"{meta['counts']['stops']} stops, {n_walk} walking links")
    return 0


if __name__ == "__main__":
    sys.exit(main())
