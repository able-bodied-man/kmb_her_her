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
| browser payload | **3.02 MB** raw → ~390 KB over the wire (Pages serves gzip) |
| pipeline runtime | ~5 s locally |

### Chinese names and headsigns

The TD feed is English-only, so names come from the KMB/LWB open-data API
(`data.etabus.gov.hk`, one namespace for both operators), joined by coordinate:

- **3,352 / 3,605 stops (93.0 %)** get a Traditional Chinese name (`name_tc`),
  median match distance 10 m
- **20,088 / 22,931 trips (87.6 %)** get a Chinese headsign (`orig_tc → dest_tc`),
  via per-trip alignment against the operator's route-stop sequences
- 1,218 distinct headsigns; 4 routes (K12/K14/K17/K18) are absent from the API

Stop ids are two different namespaces (TD integers vs operator 16-hex), and a
single TD stop can sit next to several operator kerbs at an interchange — so the
matcher keeps *all* kerbs within 40 m and aligns trips against them, rather than
taking the nearest one.

### Route geometry (real alignments)

The TD feed has no `shapes.txt` (the URL is 403), so alignments are reconstructed
in `pipeline/build_shapes.py` from the OpenStreetMap Hong Kong road extract:

- 69,683 drivable OSM ways → 376,617 nodes / 609,205 directed edges
- every consecutive stop pair is connected by A* over that graph (one-way
  restrictions respected, undirected retry when a loop makes a pair unroutable)
- snapped to the largest connected component, 3,604 / 3,605 stops matched
- Douglas-Peucker at 7 m, then stored per pattern with the vertex index of each
  stop, so drawing a leg is a slice of the pattern polyline

| metric | value |
|---|---|
| patterns with geometry | 1,503 |
| vertices | 254,986 (median spacing 95 m) |
| road length ÷ straight-line length | median **1.86** (a straight line would be 1.00) |
| unroutable stop pairs | 8 of 7,493 |
| payload | 2.12 MB raw → ~750 KB gzipped |

`shapes/shapes.bin` is committed (geometry changes when roads change, not when the
timetable refreshes); CI serves it as-is and the client refuses it if its pattern
count does not match the freshly built timetable, falling back to straight lines.


**The catch:** TD timestamps only the *first and last* stop of each trip — 91 %
of intermediate times are blank. Trip departure times and total run times are
exact; intermediate times are reconstructed with a per-hop speed profile (short
hop = slow urban stop, long hop = expressway run, 20 s dwell each) fitted to the
published duration, so they carry roughly ±1–2 min of error. Trips whose published
duration implies a bus averaging faster than 42 km/h with urban stop spacing are
stretched (3,096 of 22,931) — an operator timetable that has a 9 km Kowloon run
in 12 minutes is wrong, and the map would otherwise show a bus outrunning traffic.
Verified against route 1 (Chuk Yuen → Star Ferry, 46 min, 25 stops) and 968
(Yuen Long → Causeway Bay, 78 min).

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
- [x] **2. Routing engine** — label-setting search over patterns + walking
      transfers (`js/app.js`). 20–80 ms per query in the browser; checked against
      hand-verifiable journeys (竹園邨→銅鑼灣, 元朗→尖沙咀, 天水圍→機場) and a
      negative case (赤柱 returns no plan rather than a fake one)
- [x] **3. Map UI** — Chinese-first UI, stop search (zh + en), map-pick, waiting
      times, itinerary legs, OSM raster tiles, optional second option
- [x] **4a. Chinese** — stop names and headsigns from the operator API; service
      days use `calendar_dates.txt` (public-holiday overrides honoured)
- [ ] **4b. Polish** — live arrivals, "arrive by" search, joint-route flagging
- [ ] **5. Calibration** — per-segment run times from observed ETA data

## Known limits

- Walking legs are straight-line distance, capped at 400 m. Cross-water pairs at
  that range (Lei Yue Mun) can produce an unusable transfer; a land-mask guard is
  planned.
- `calendar_dates.txt` holidays are in the data but the service-day logic must
  use them, or holiday timetables will be wrong.
- KMB+CTB joint routes are offered because KMB operates part of them; the UI
  should flag them.
