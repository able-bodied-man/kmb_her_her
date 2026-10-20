# kmb_her_her

A point-A → point-B journey planner for Hong Kong that only uses what a
**KMB/LWB pass holder** can actually ride: KMB, LWB, jointly operated
KMB+CTB / LWB+CTB routes, plus walking. No MTR, no Citybus-only routes, no
minibus, no tram, no ferry.

## Status

Data pipeline is built and verified. The router and the map UI are next.

## Data (measured, not assumed)

Source: Transport Department — *Headway information of public transport services*
(GTFS, refreshed biweekly) — <https://static.data.gov.hk/td/pt-headway-en/>.
It covers every HK mode; the pipeline keeps only KMB/LWB.

| item | value |
|---|---|
| routes kept | 1,012 (794 KMB, 89 LWB, 126 KMB+CTB, 3 LWB+CTB) |
| trips | 22,931 |
| stop_times rows | 525,044 |
| stops | 3,605 |
| stop↔stop walking links ≤400 m | 39,314 |
| browser payload | **2.84 MB** raw / ~1 MB gzipped |
| pipeline runtime | ~5 s locally |

**The catch:** TD timestamps only the *first and last* stop of each trip — 91 %
of intermediate times are blank. Trip departure times and total run times are
exact; intermediate times are reconstructed as
`duration spread over inter-stop distance + 20 s dwell per stop`, so they carry
roughly ±1–2 min of error. Verified against route 1 (Chuk Yuen → Star Ferry,
46 min, 25 stops) and 968 (Yuen Long → Causeway Bay, 78 min).

That reconstruction is the main accuracy lever. Upgrade path: poll the KMB/LWB
real-time ETA API for the same routes and calibrate per-segment run times from
observed data, then re-emit the dataset.

Also noted for later: GTFS stop names are English-only and use TD stop IDs,
while the KMB ETA API uses its own 16-hex stop IDs. Crossing the two needs a
coordinate match (~15 m), which is how Chinese stop names and live arrivals get
added.

## Architecture

Everything static, no server:

1. **`pipeline/build_data.py`** — downloads the feed, filters to KMB/LWB,
   reconstructs intermediate times, and packs compact columnar binaries
   (`trips.bin`, `stoptimes.bin`, `walk.bin`) plus JSON (stops, routes,
   calendars). Stdlib only.
2. **GitHub Actions** — runs the pipeline on every push and weekly, then deploys
   the whole tree to Pages. Data is never committed; it is rebuilt from source.
3. **Client** — loads ~1 MB gzipped, runs a RAPTOR search over the timetable with
   walking transfers (`walk.bin`), and draws the itinerary on a map
   (Leaflet + OSM tiles, with attributed tile source).

Why client-side: GitHub Pages cannot run a server, and the whole dataset fits in
1 MB compressed. Search stays instant and hosting stays free. If live ETAs and
per-segment calibration arrive, the same pipeline can precompute them in CI
instead of adding a backend.

## Phases

- [x] **0. Hosting** — Pages + Actions deploy, live at
      <https://able-bodied-man.github.io/kmb_her_her/>
- [x] **1. Data** — GTFS → compact routing dataset, verified
- [ ] **2. Routing engine** — RAPTOR over trips + walking transfers, unit-tested
      against hand-checked journeys (e.g. Chuk Yuen → Causeway Bay)
- [ ] **3. Map UI** — origin/destination picking (search stops, click map,
      geolocate), itinerary with leg-by-leg times, route numbers, walking legs
- [ ] **4. Polish** — Chinese (zh-HK) names via KMB ETA coordinate match, live
      arrivals, "leave at / arrive by", service-day handling incl. public holidays
- [ ] **5. Calibration** — per-segment run times from observed ETA data

## Known limits

- Walking legs are straight-line distance, capped at 400 m. Cross-water pairs at
  that range (Lei Yue Mun) can produce an unusable transfer; a land-mask guard is
  planned.
- `calendar_dates.txt` holidays are in the data but the service-day logic must
  use them, or holiday timetables will be wrong.
- KMB+CTB joint routes are offered because KMB operates part of them; the UI
  should flag them.
