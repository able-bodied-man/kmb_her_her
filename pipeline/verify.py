#!/usr/bin/env python3
"""Sanity checks over data/ produced by build_data.py (no deps).

Usage: python3 pipeline/verify.py [route_short_name]
"""
from __future__ import annotations

import json
import os
import struct
import sys
from array import array

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = os.path.join(ROOT, "data")


def load():
    stops = json.load(open(os.path.join(D, "stops.json"), encoding="utf-8"))
    routes = json.load(open(os.path.join(D, "routes.json"), encoding="utf-8"))
    headsigns = json.load(open(os.path.join(D, "headsigns.json"), encoding="utf-8"))
    meta = json.load(open(os.path.join(D, "meta.json"), encoding="utf-8"))
    with open(os.path.join(D, "trips.bin"), "rb") as f:
        magic, n_trips, n_st = struct.unpack("<4sII", f.read(12))
        assert magic == b"KHH2", magic
        t_route = array("H"); t_route.fromfile(f, n_trips)
        t_service = array("B"); t_service.fromfile(f, n_trips)
        t_start = array("I"); t_start.fromfile(f, n_trips)
        t_offset = array("I"); t_offset.fromfile(f, n_trips)
        t_nstops = array("H"); t_nstops.fromfile(f, n_trips)
        t_headsign = array("H"); t_headsign.fromfile(f, n_trips)
    with open(os.path.join(D, "stoptimes.bin"), "rb") as f:
        magic, n_rows = struct.unpack("<4sI", f.read(8))
        assert magic == b"KHT2", magic
        st_stop = array("H"); st_stop.fromfile(f, n_rows)
        st_arr = array("H"); st_arr.fromfile(f, n_rows)
    return (stops, routes, meta, headsigns,
            (t_route, t_service, t_start, t_offset, t_nstops, t_headsign), (st_stop, st_arr))


def hhmm(sec: int) -> str:
    return f"{sec // 3600:02d}:{sec % 3600 // 60:02d}:{sec % 60:02d}"


def main() -> int:
    want = sys.argv[1] if len(sys.argv) > 1 else "1"
    stops, routes, meta, headsigns, T, S = load()
    t_route, t_service, t_start, t_offset, t_nstops, t_headsign = T
    st_stop, st_arr = S
    print(f"meta: {meta['counts']}")
    print(f"bounds: {meta['bounds']}")
    assert len(t_route) == len(t_start) == len(t_offset) == len(t_nstops) == len(t_headsign)

    # 1. every stop_time row decodes to a valid stop and non-decreasing time
    bad = 0
    for ti in range(len(t_route)):
        off, n = t_offset[ti], t_nstops[ti]
        prev_idx, prev_arr = -9999, -1
        for k in range(n):
            packed = st_stop[off + k]
            delta = packed & 0x1FFF
            idx = prev_idx + delta - 3605 if k else delta - 3605
            arr = st_arr[off + k]
            if not (0 <= idx < len(stops)) or arr < prev_arr:
                bad += 1
            prev_idx, prev_arr = idx, arr
    print(f"decoding: {bad} bad rows (want 0)")

    # 2. show one trip end to end
    tripidx = [i for i, ri in enumerate(t_route) if routes[ri][1] == want]
    print(f"route {want}: {len(tripidx)} trips in feed, {len(routes)} routes total")
    if tripidx:
        for ti in (tripidx[0], tripidx[len(tripidx) // 2]):
            off, n = t_offset[ti], t_nstops[ti]
            start = t_start[ti]
            h = t_headsign[ti]
            head = f" → {headsigns[h][3]}" if h != 0xFFFF else ""
            print(f"\n  trip #{ti} service={t_service[ti]} stops={n} "
                  f"start={hhmm(start)} end={hhmm(start + st_arr[off + n - 1])} "
                  f"duration={st_arr[off + n - 1] // 60} min{head}")
            prev_idx = -9999
            for k in range(n):
                packed = st_stop[off + k]
                idx = (prev_idx + (packed & 0x1FFF) - 3605) if k else (packed & 0x1FFF) - 3605
                prev_idx = idx
                if k < 4 or k >= n - 3:
                    tc = stops[idx][4] if len(stops[idx]) > 4 and stops[idx][4] else ""
                    print(f"    {k + 1:3d} {hhmm(start + st_arr[off + k])}  "
                          f"{stops[idx][3][:40]:42s} {tc}")
                elif k == 4:
                    print("        ...")

    # 3. duration distribution vs distance, as a smell test
    import statistics
    durs = [st_arr[t_offset[i] + t_nstops[i] - 1] / 60 for i in range(len(t_route))]
    print(f"\ntrip durations: median {statistics.median(durs):.0f} min, "
          f"p5 {sorted(durs)[len(durs) // 20]:.0f}, p95 {sorted(durs)[len(durs) * 19 // 20]:.0f}, "
          f"max {max(durs):.0f}")
    print(f"walk links: {meta['counts']['walk_links']} over {meta['counts']['stops']} stops "
          f"({meta['counts']['walk_links'] / meta['counts']['stops']:.1f} per stop)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
