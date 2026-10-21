#!/usr/bin/env python3
"""Reconstruct the walking legs: real footpaths instead of straight lines.

The dataset models a transfer as "these two kerbs are 180 m apart, walk there".
Drawn as a straight line that chord cuts through blocks, buildings and the
harbour. This script routes every walkable kerb pair along the OpenStreetMap
pedestrian network (footways, paths, crossings plus the walkable street
network) and stores the resulting polyline, so the itinerary can draw the same
walk a pedestrian would actually take.

Output: shapes/walk.bin (+ shapes/walk_meta.json), consumed by js/app.js.
Committed to the repo on purpose: footpaths change when the city changes, not
when the timetable refreshes.

Requires `osmium` (pip) and the Geofabrik Hong Kong extract:
  curl -LO https://download.geofabrik.de/asia/china/hong-kong-latest.osm.pbf
"""
from __future__ import annotations

import heapq
import json
import math
import os
import pickle
import struct
import time
from array import array

import osmium

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "shapes")
CACHE = os.environ.get("KHH_OSM_CACHE", os.path.join(ROOT, ".gtfs-cache"))
PBF = os.environ.get("KHH_PBF", os.path.join(CACHE, "hong-kong-latest.osm.pbf"))

# What a pedestrian may use. Motorways and tunnels without a footway are out.
WALKABLE = {
    "footway", "path", "pedestrian", "steps", "living_street", "track", "corridor",
    "residential", "service", "unclassified", "tertiary", "secondary", "primary",
    "tertiary_link", "secondary_link", "primary_link",
}
SPEED = {                       # metres per second
    "footway": 1.35, "path": 1.15, "pedestrian": 1.35, "steps": 0.75,
    "corridor": 1.25, "track": 1.05, "living_street": 1.25, "service": 1.25,
    "residential": 1.3, "unclassified": 1.3, "tertiary": 1.3, "secondary": 1.3,
    "primary": 1.3, "tertiary_link": 1.2, "secondary_link": 1.2, "primary_link": 1.2,
}
MAX_V = 1.4                     # heuristic speed, m/s
SNAP_M = 60.0                   # kerb -> footpath node
SIMPLIFY_M = 3.5                # Douglas-Peucker tolerance (tight: paths are short)
R = 6371000.0


def log(m):
    print(m, flush=True)


def hav(a, b, c, d):
    p1, p2 = a * math.pi / 180, c * math.pi / 180
    dp, dl = p2 - p1, (d - b) * math.pi / 180
    x = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(x)))


# --------------------------------------------------------------------- graph
def load_graph():
    pkl = os.path.join(CACHE, "hk_graph_walking.pkl")
    if os.path.exists(pkl):
        log("[graph] loading cached walking graph")
        with open(pkl, "rb") as f:
            return pickle.load(f)

    log(f"[graph] parsing {PBF} for walkable ways")
    ways = []
    for obj in osmium.FileProcessor(PBF):
        if not obj.is_way():
            continue
        tags = obj.tags
        hw = tags.get("highway")
        if hw not in WALKABLE:
            continue
        access = tags.get("access")
        if access in ("no", "private") or tags.get("foot") in ("no", "private"):
            continue
        nodes = [n.ref for n in obj.nodes]
        if len(nodes) >= 2:
            ways.append((nodes, hw))
    log(f"[graph] {len(ways)} walkable ways")

    need = {n for nodes, _ in ways for n in nodes}
    coords = {}
    for obj in osmium.FileProcessor(PBF).with_locations():
        if obj.is_node() and obj.id in need:
            coords[obj.id] = (obj.location.lat, obj.location.lon)

    idx_of, lats, lons = {}, array("d"), array("d")
    for nid, (la, lo) in coords.items():
        idx_of[nid] = len(lats)
        lats.append(la); lons.append(lo)
    N = len(lats)

    # undirected: a pedestrian walks both ways on every path
    e_from, e_to, e_cost = array("i"), array("i"), array("d")
    for nodes, hw in ways:
        v = SPEED.get(hw, 1.2)
        for a, b in zip(nodes, nodes[1:]):
            ia, ib = idx_of.get(a), idx_of.get(b)
            if ia is None or ib is None:
                continue
            c = hav(lats[ia], lons[ia], lats[ib], lons[ib]) / v
            e_from.append(ia); e_to.append(ib); e_cost.append(c)
            e_from.append(ib); e_to.append(ia); e_cost.append(c)

    E = len(e_from)
    order = sorted(range(E), key=lambda i: e_from[i])
    head = [0] * (N + 1)
    for i in order:
        head[e_from[i] + 1] += 1
    for i in range(N):
        head[i + 1] += head[i]
    to = array("i", [0]) * E
    cost = array("d", [0.0]) * E
    pos = head[:]
    for i in order:
        f = e_from[i]
        to[pos[f]] = e_to[i]
        cost[pos[f]] = e_cost[i]
        pos[f] += 1

    graph = {"N": N, "lats": lats, "lons": lons, "head": head, "to": to, "cost": cost}
    with open(pkl, "wb") as f:
        pickle.dump(graph, f, protocol=4)
    log(f"[graph] {N} nodes, {E} directed pedestrian edges")
    return graph


def grid_index(G, cell_m=80.0):
    """Bucket nodes by a coarse lat/lon grid so snapping is not O(N) per kerb."""
    lats, lons = G["lats"], G["lons"]
    dlat = cell_m / 111320.0
    dlon = cell_m / (111320.0 * math.cos(math.radians(22.35)))
    grid = {}
    for i in range(G["N"]):
        key = (int(lats[i] / dlat), int(lons[i] / dlon))
        grid.setdefault(key, []).append(i)
    return grid, dlat, dlon


def snap(G, grid, dlat, dlon, lat, lon, radius_m=SNAP_M):
    lats, lons = G["lats"], G["lons"]
    cx, cy = int(lat / dlat), int(lon / dlon)
    best, bestd = None, 1e9
    reach = max(1, int(radius_m / 80.0) + 1)
    for gx in range(cx - reach, cx + reach + 1):
        for gy in range(cy - reach, cy + reach + 1):
            for i in grid.get((gx, gy), ()):
                d = hav(lat, lon, lats[i], lons[i])
                if d < bestd:
                    best, bestd = i, d
    return (best, bestd) if best is not None and bestd <= radius_m else (None, None)


def astar(G, src, dst):
    lats, lons = G["lats"], G["lons"]
    head, to, cost = G["head"], G["to"], G["cost"]
    INF = float("inf")
    hk = 1.0 / MAX_V
    dl, do = lats[dst], lons[dst]
    g = {src: 0.0}
    closed = set()
    came = {}
    pq = [(hav(lats[src], lons[src], dl, do) * hk, src)]
    while pq:
        _f, u = heapq.heappop(pq)
        if u == dst:
            path = [u]
            while u in came:
                u = came[u]
                path.append(u)
            return path[::-1]
        if u in closed:
            continue
        closed.add(u)
        gu = g[u]
        for k in range(head[u], head[u + 1]):
            v = to[k]
            if v in closed:
                continue
            ng = gu + cost[k]
            if ng < g.get(v, INF):
                g[v] = ng
                came[v] = u
                heapq.heappush(pq, (ng + hav(lats[v], lons[v], dl, do) * hk, v))
    return None


def simplify(points, tol_m):
    """Douglas-Peucker on [(lat, lon)], tolerance in metres."""
    if len(points) < 3:
        return points

    def perp(p, a, b):
        # planar approximation is fine at street scale
        ax, ay = a[1] * math.cos(math.radians(a[0])), a[0]
        bx, by = b[1] * math.cos(math.radians(b[0])), b[0]
        px, py = p[1] * math.cos(math.radians(p[0])), p[0]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 == 0:
            t = 0.0
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
        return hav(p[0], p[1], a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))

    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        dmax, idx = 0.0, i
        for k in range(i + 1, j):
            d = perp(points[k], points[i], points[j])
            if d > dmax:
                dmax, idx = d, k
        if dmax > tol_m:
            keep[idx] = True
            stack.append((i, idx))
            stack.append((idx, j))
    return [p for p, k in zip(points, keep) if k]


# --------------------------------------------------------------- walk links
def load_walk_links():
    """Read data/walk.bin: u32 offsets per stop, u16 neighbour + u16 metres."""
    b = open(os.path.join(DATA, "walk.bin"), "rb").read()
    n_stops, n_links = struct.unpack_from("<II", b, 4)
    o = 12
    off = array("I"); off.frombytes(b[o:o + (n_stops + 1) * 4]); o += (n_stops + 1) * 4
    pairs = array("H"); pairs.frombytes(b[o:o + n_links * 2]); o += n_links * 2
    metres = array("H"); metres.frombytes(b[o:o + n_links * 2])
    links = {}
    for s in range(n_stops):
        for k in range(off[s], off[s + 1]):
            n = pairs[k]
            a, c = (s, n) if s < n else (n, s)
            links[(a, c)] = metres[k]
    return links, n_stops


def main():
    t0 = time.time()
    os.makedirs(OUT, exist_ok=True)
    stops = json.load(open(os.path.join(DATA, "stops.json")))
    links, n_stops = load_walk_links()
    log(f"[links] {len(links)} unique walkable kerb pairs over {n_stops} stops")

    G = load_graph()
    grid, dlat, dlon = grid_index(G)
    snapped = {}
    for sid, ((a, b), _m) in enumerate(links.items()):
        for s in (a, b):
            if s not in snapped:
                snapped[s] = snap(G, grid, dlat, dlon, stops[s][1], stops[s][2])
        if sid % 5000 == 0 and sid:
            log(f"[snap] {sid}/{len(links)}  {time.time() - t0:.0f}s")

    pairs = sorted(links)
    point_off = array("I", [0])
    pts = array("i")            # lat/lon interleaved, 1e7 fixed point
    routed = unroutable = straight = 0
    for n, (a, b) in enumerate(pairs):
        na, da = snapped[a]
        nb, db = snapped[b]
        path = None
        declared = links[(a, b)]
        if declared >= 60 and na is not None and nb is not None and na != nb:
            node_path = astar(G, na, nb)
            if node_path and len(node_path) > 1:
                coords = [(G["lats"][i], G["lons"][i]) for i in node_path]
                # pin the polyline to the real kerbs, then simplify
                coords = [(stops[a][1], stops[a][2])] + coords + [(stops[b][1], stops[b][2])]
                coords = simplify(coords, SIMPLIFY_M)
                path = coords
        if path is None:
            path = [(stops[a][1], stops[a][2]), (stops[b][1], stops[b][2])]
            unroutable += 1
        else:
            # refuse a "path" that wanders absurdly far for a <400 m kerb pair —
            # a 30 m hop is drawn as a short line, not a lap of the block
            length = sum(hav(path[i][0], path[i][1], path[i + 1][0], path[i + 1][1])
                         for i in range(len(path) - 1))
            if length > max(1.9 * declared, declared + 140):
                path = [(stops[a][1], stops[a][2]), (stops[b][1], stops[b][2])]
                straight += 1
            else:
                routed += 1
        for la, lo in path:
            pts.append(int(round(la * 1e7)))
            pts.append(int(round(lo * 1e7)))
        point_off.append(len(pts) // 2)
        if n % 5000 == 0 and n:
            log(f"[route] {n}/{len(pairs)}  routed={routed} fallback={unroutable + straight}  "
                f"{time.time() - t0:.0f}s")

    with open(os.path.join(OUT, "walk.bin"), "wb") as f:
        f.write(struct.pack("<4sII", b"KHW1", len(pairs), len(pts) // 2))
        array("I", [a for a, _ in pairs]).tofile(f)
        array("I", [b for _, b in pairs]).tofile(f)
        point_off.tofile(f)
        pts.tofile(f)

    vertices = len(pts) // 2
    log(f"[done] {len(pairs)} kerb pairs, {vertices} vertices "
        f"({vertices / max(1, len(pairs)):.1f} per walk), routed {routed}, "
        f"straight fallback {unroutable + straight} (unroutable {unroutable}), "
        f"{os.path.getsize(os.path.join(OUT, 'walk.bin')) / 1e6:.2f} MB, {time.time() - t0:.0f}s")
    json.dump({"pairs": len(pairs), "vertices": vertices, "routed": routed,
               "unroutable": unroutable, "too_long": straight,
               "simplify_m": SIMPLIFY_M, "built": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())},
              open(os.path.join(OUT, "walk_meta.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
