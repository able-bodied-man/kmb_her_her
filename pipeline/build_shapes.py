#!/usr/bin/env python3
"""Build real route geometry (bus alignments) for every pattern.

The TD GTFS feed has no shapes.txt (the file is 403), so the alignment of each
route is reconstructed by routing along OpenStreetMap roads: every consecutive
stop pair is connected by a shortest-path search over the drivable street graph,
and the node sequence becomes the polyline.

Output: shapes/shapes.bin (+ shapes/meta.json), consumed by js/app.js.
Committed to the repo on purpose: geometry changes when roads change, not when
the timetable is refreshed, so it does not need rebuilding in CI every push.

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
import sys
import time
from array import array

import osmium

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(ROOT, "data")
OUT = os.path.join(ROOT, "shapes")
CACHE = os.environ.get("KHH_OSM_CACHE", os.path.join(ROOT, ".gtfs-cache"))
PBF = os.environ.get("KHH_PBF", os.path.join(CACHE, "hong-kong-latest.osm.pbf"))

DRIVABLE = {
    "motorway", "trunk", "primary", "secondary", "tertiary", "unclassified",
    "residential", "service", "living_street", "busway", "bus_guideway",
    "motorway_link", "trunk_link", "primary_link", "secondary_link", "tertiary_link",
}
SPEED = {
    "motorway": 70, "trunk": 60, "primary": 50, "secondary": 45, "tertiary": 40,
    "unclassified": 35, "residential": 30, "living_street": 15, "service": 20,
    "busway": 40, "bus_guideway": 40,
}
for _k in list(SPEED):
    SPEED[_k + "_link"] = SPEED[_k] * 0.7
MAX_KMH = 70.0
SNAP_M = 90.0            # stop -> road node
SIMPLIFY_M = 7.0         # Douglas-Peucker tolerance

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
    pkl = os.path.join(CACHE, "hk_graph_driving.pkl")
    if os.path.exists(pkl):
        log("[graph] loading cached graph")
        with open(pkl, "rb") as f:
            return pickle.load(f)

    log(f"[graph] parsing {PBF}")
    ways = []
    for obj in osmium.FileProcessor(PBF):
        if not obj.is_way():
            continue
        tags = obj.tags
        hw = tags.get("highway")
        if hw not in DRIVABLE or tags.get("access") in ("no", "private"):
            continue
        nodes = [n.ref for n in obj.nodes]
        if len(nodes) < 2:
            continue
        oneway = tags.get("oneway", "")
        fwd = oneway != "-1"
        bwd = oneway in ("-1", "no", "false", "0") or oneway == ""
        ways.append((nodes, hw, fwd, bwd))

    need = {n for nodes, *_ in ways for n in nodes}
    coords = {}
    for obj in osmium.FileProcessor(PBF).with_locations():
        if obj.is_node() and obj.id in need:
            coords[obj.id] = (obj.location.lat, obj.location.lon)

    idx_of, lats, lons = {}, array("d"), array("d")
    for nid, (la, lo) in coords.items():
        idx_of[nid] = len(lats)
        lats.append(la); lons.append(lo)
    N = len(lats)

    from_nodes, to_nodes, costs = array("i"), array("i"), array("d")
    for nodes, hw, fwd, bwd in ways:
        kmh = SPEED.get(hw, 30)
        for a, b in zip(nodes, nodes[1:]):
            ia, ib = idx_of.get(a), idx_of.get(b)
            if ia is None or ib is None:
                continue
            c = hav(lats[ia], lons[ia], lats[ib], lons[ib]) / (kmh / 3.6)
            if fwd:
                from_nodes.append(ia); to_nodes.append(ib); costs.append(c)
            if bwd:
                from_nodes.append(ib); to_nodes.append(ia); costs.append(c)

    E = len(from_nodes)
    order = sorted(range(E), key=lambda i: from_nodes[i])
    head = [0] * (N + 1)
    for i in order:
        head[from_nodes[i] + 1] += 1
    for i in range(N):
        head[i + 1] += head[i]
    to = array("i", [0]) * E
    cost = array("d", [0.0]) * E
    pos = head[:]
    for i in order:
        f = from_nodes[i]
        to[pos[f]] = to_nodes[i]
        cost[pos[f]] = costs[i]
        pos[f] += 1

    # undirected copy: some stop pairs are only reachable against the one-way
    # restrictions (a bus can loop, but we still want a line rather than a 20 km hop)
    u_from = array("i", list(from_nodes) + list(to_nodes))
    u_to = array("i", list(to_nodes) + list(from_nodes))
    u_cost = array("d", list(costs) + list(costs))
    uo = sorted(range(len(u_from)), key=lambda i: u_from[i])
    uhead = [0] * (N + 1)
    for i in uo:
        uhead[u_from[i] + 1] += 1
    for i in range(N):
        uhead[i + 1] += uhead[i]
    uto = array("i", [0]) * len(uo)
    ucost = array("d", [0.0]) * len(uo)
    upos = uhead[:]
    for i in uo:
        f = u_from[i]
        uto[upos[f]] = u_to[i]
        ucost[upos[f]] = u_cost[i]
        upos[f] += 1

    graph = {"N": N, "lats": lats, "lons": lons, "head": head, "to": to, "cost": cost,
             "uhead": uhead, "uto": uto, "ucost": ucost}
    with open(pkl, "wb") as f:
        pickle.dump(graph, f, protocol=4)
    log(f"[graph] {N} nodes, {E} directed edges")
    return graph


def astar(G, src, dst, undirected=False):
    lats, lons = G["lats"], G["lons"]
    if undirected:
        head, to, cost = G["uhead"], G["uto"], G["ucost"]
    else:
        head, to, cost = G["head"], G["to"], G["cost"]
    INF = float("inf")
    hk = 1.0 / (MAX_KMH / 3.6)
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
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        if b <= a + 1:
            continue
        ax, ay = points[a][1], points[a][0]
        bx, by = points[b][1], points[b][0]
        dx, dy = bx - ax, by - ay
        norm = math.hypot(dx, dy)
        best, best_i = -1.0, None
        for i in range(a + 1, b):
            px, py = points[i][1], points[i][0]
            if norm == 0:
                d = hav(points[i][0], points[i][1], points[a][0], points[a][1])
            else:
                t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (norm * norm)))
                qx, qy = ax + t * dx, ay + t * dy
                d = hav(points[i][0], points[i][1], qy, qx)
            if d > best:
                best, best_i = d, i
        if best > tol_m:
            keep[best_i] = True
            stack.append((a, best_i))
            stack.append((best_i, b))
    return [p for p, k in zip(points, keep) if k]


def main():
    os.makedirs(OUT, exist_ok=True)
    stops = json.load(open(os.path.join(DATA, "stops.json"), encoding="utf-8"))
    n_stops = len(stops)

    # pattern table straight out of patterns.bin
    with open(os.path.join(DATA, "patterns.bin"), "rb") as f:
        magic, n_pat, n_pat_stops, n_refs = struct.unpack("<4sIII", f.read(16))
        assert magic == b"KHP1", magic
        p_route = array("H"); p_route.fromfile(f, n_pat)
        p_stop_off = array("I"); p_stop_off.fromfile(f, n_pat + 1)
        p_trip_off = array("I"); p_trip_off.fromfile(f, n_pat + 1)   # skip trips
        pat_stops = array("H"); pat_stops.fromfile(f, n_pat_stops)
    log(f"[data] {n_pat} patterns, {n_pat_stops} pattern-stop entries")

    G = load_graph()
    lats, lons = G["lats"], G["lons"]

    # keep only the largest connected component: a stop that snaps to an isolated
    # service node produces an unroutable pair and a 20 km straight line on the map
    log("[graph] finding the largest connected component")
    parent = list(range(G["N"]))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for u in range(G["N"]):
        for k in range(G["head"][u], G["head"][u + 1]):
            v = G["to"][k]
            ru, rv = find(u), find(v)
            if ru != rv:
                parent[ru] = rv
    from collections import Counter
    sizes = Counter(find(i) for i in range(G["N"]))
    main_root, main_size = sizes.most_common(1)[0]
    in_main = [find(i) == main_root for i in range(G["N"])]
    log(f"[graph] largest component {main_size} of {G['N']} nodes")

    # snap stops to graph nodes via a coarse grid
    log("[snap] matching stops to road nodes")
    cell = 0.004
    grid = {}
    for i in range(G["N"]):
        grid.setdefault((int(lats[i] / cell), int(lons[i] / cell)), []).append(i)
    snap = [None] * n_stops
    far = 0
    for si, s in enumerate(stops):
        la, lo = float(s[1]), float(s[2])
        gy, gx = int(la / cell), int(lo / cell)
        best = None
        for limit in (SNAP_M, SNAP_M * 2.5):
            bestd = limit
            for dy in (-1, 0, 1, 2, -2):
                for dx in (-1, 0, 1, 2, -2):
                    for ni in grid.get((gy + dy, gx + dx), ()):
                        if not in_main[ni]:
                            continue
                        d = hav(la, lo, lats[ni], lons[ni])
                        if d < bestd:
                            best, bestd = ni, d
            if best is not None:
                break
        if best is None:
            far += 1
        snap[si] = best
    log(f"[snap] {n_stops - far} stops snapped, {far} too far from any road node")

    cache: dict[tuple[int, int], list[int] | None] = {}

    def route_pair(sa, sb):
        if sa is None or sb is None:
            return None
        path = cache.get((sa, sb))
        if path is None:
            path = astar(G, sa, sb)
            if not path:                                  # try against the one-way system
                path = astar(G, sa, sb, undirected=True)
            cache[(sa, sb)] = path
        return path

    # route each distinct consecutive stop pair once
    log("[route] connecting consecutive stops along the street network")
    t0 = time.time()
    missing_pairs = 0
    for pi in range(n_pat):
        base = p_stop_off[pi]
        prev = pat_stops[base]
        for k in range(base + 1, p_stop_off[pi + 1]):
            cur = pat_stops[k]
            if route_pair(snap[prev], snap[cur]) is None:
                missing_pairs += 1
            prev = cur
        if pi % 250 == 0:
            log(f"       {pi}/{n_pat} patterns, {len(cache)} pairs, {time.time() - t0:.0f}s")

    # assemble per-pattern polylines
    log("[build] assembling pattern polylines")
    p_point_off = array("I", [0])
    p_stop_off2 = array("I", [0])
    pts = array("i")
    stop_vertex = array("H")
    pic = 0
    for pi in range(n_pat):
        base = p_stop_off[pi]
        seq = [pat_stops[k] for k in range(base, p_stop_off[pi + 1])]
        acc = []            # [(lat, lon)]
        vids = []           # vertex index of each stop
        for k, sidx in enumerate(seq):
            if k == 0:
                n0 = snap[sidx]
                if n0 is None:
                    acc.append((float(stops[sidx][1]), float(stops[sidx][2])))
                else:
                    acc.append((lats[n0], lons[n0]))
                vids.append(0)
                continue
            path = route_pair(snap[seq[k - 1]], snap[sidx])
            if path:
                for n in path[1:]:
                    acc.append((lats[n], lons[n]))
            else:
                acc.append((float(stops[sidx][1]), float(stops[sidx][2])))
            vids.append(len(acc) - 1)
        simp_vids = []
        if len(acc) > 2:
            keep_idx = set()
            simple = simplify(acc, SIMPLIFY_M)
            # map simplified points back to vertex ids (nearest original index)
            j = 0
            for p in simple:
                while j < len(acc) - 1 and acc[j] != p:
                    j += 1
                keep_idx.add(j)
            # always keep stop vertices
            keep_idx |= set(vids)
            order = sorted(keep_idx)
            remap = {old: new for new, old in enumerate(order)}
            acc = [acc[o] for o in order]
            simp_vids = [remap[v] for v in vids]
        else:
            simp_vids = vids
        for la, lo in acc:
            pts.append(int(round(la * 1e5)))
            pts.append(int(round(lo * 1e5)))
        p_point_off.append(len(pts) // 2)
        for v in simp_vids:
            stop_vertex.append(v)
        p_stop_off2.append(len(stop_vertex))
        pic += 1

    with open(os.path.join(OUT, "shapes.bin"), "wb") as f:
        f.write(struct.pack("<4sIII", b"KHS1", n_pat, len(pts) // 2, len(stop_vertex)))
        p_point_off.tofile(f)
        p_stop_off2.tofile(f)
        pts.tofile(f)
        stop_vertex.tofile(f)

    size = os.path.getsize(os.path.join(OUT, "shapes.bin"))
    meta = {
        "source": "OpenStreetMap (Geofabrik HK extract) road network, A* per stop pair",
        "simplify_m": SIMPLIFY_M, "snap_m": SNAP_M,
        "patterns": n_pat, "points": len(pts) // 2, "bytes": size,
        "pairs_routed": len(cache), "pairs_unroutable": missing_pairs,
        "stops_snapped": n_stops - far,
    }
    with open(os.path.join(OUT, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)
    log(f"[done] {size / 1e6:.2f} MB, {len(pts)//2} points, "
        f"{len(cache)} pairs routed, {missing_pairs} unroutable")
    return 0


if __name__ == "__main__":
    sys.exit(main())
