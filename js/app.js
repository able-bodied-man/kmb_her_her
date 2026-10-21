/* kmb_her_her — KMB/LWB-pass journey planner, fully client side.
 *
 * Data files are built by pipeline/build_data.py and served from data/.
 * Routing: label-setting (Dijkstra-style) search over stops, where the edge
 * from a stop is "board the earliest catchable trip of any pattern serving it",
 * plus precomputed walking transfers. Trips are grouped into patterns
 * (route + stop sequence) so a boarding decision is one binary search per
 * pattern instead of a scan over every trip.
 */
'use strict';

const DATA = 'data/';
const WALK_SPEED = 1.2;          // m/s ~ 4.3 km/h
const ORIGIN_WALK_M = 900;       // walk we will accept at the start / end
const TRANSFER_PENALTY = 45;     // s to change buses at the same stop
const INF = 1e9;
const MAX_OFFSET_SLACK = 7200;   // guard for binary search over trip starts

const state = {
  stops: [], routes: [], headsigns: [], cal: null, calIndex: {}, cards: [],
  trips: null, st: null, walk: null, pat: null, sp: null,
  from: null, to: null,
  map: null, mapLayers: [],
  picking: null,
};

/* ------------------------------------------------------------------ loading */
async function getBuffer(name) {
  const r = await fetch(DATA + name, { cache: 'no-cache' });
  if (!r.ok) throw new Error(`${name}: HTTP ${r.status}`);
  return r.arrayBuffer();
}
async function getJSON(name) {
  const r = await fetch(DATA + name, { cache: 'no-cache' });
  if (!r.ok) throw new Error(`${name}: HTTP ${r.status}`);
  return r.json();
}
const u8 = (b, o, n) => new Uint8Array(b.slice(o, o + n));
const u16 = (b, o, n) => new Uint16Array(b.slice(o, o + n * 2));
const u32 = (b, o, n) => new Uint32Array(b.slice(o, o + n * 4));

async function boot() {
  const [stops, routes, headsigns, cal, meta] = await Promise.all([
    getJSON('stops.json'), getJSON('routes.json'), getJSON('headsigns.json'), getJSON('calendar.json'),
    getJSON('meta.json'),
  ]);
  state.stops = stops;
  state.routes = routes;
  state.headsigns = headsigns;
  state.cal = cal;
  state.meta = meta;
  state.generated = meta.generated || null;
  state.digest = meta.digest || null;
  cal.services.forEach((sid, i) => { state.calIndex[sid] = i; });

  const [tripsBuf, stBuf, walkBuf, patBuf, spBuf, frBuf] = await Promise.all([
    getBuffer('trips.bin'), getBuffer('stoptimes.bin'), getBuffer('walk.bin'),
    getBuffer('patterns.bin'), getBuffer('stoppatterns.bin'), getBuffer('frequencies.bin'),
  ]);
  // route alignments are optional: without them the map falls back to straight lines
  state.shapes = await loadShapes();
  state.walkShapes = await loadWalkShapes();

  // trips.bin: <4s II> then u16 route, u8 service, u32 start, u32 offset, u16 nstops, u16 headsign
  const nTrips = new DataView(tripsBuf).getUint32(4, true);
  const nSt = new DataView(tripsBuf).getUint32(8, true);
  let o = 12;
  const tRoute = u16(tripsBuf, o, nTrips); o += nTrips * 2;
  const tService = u8(tripsBuf, o, nTrips); o += nTrips;
  const tStart = u32(tripsBuf, o, nTrips); o += nTrips * 4;
  const tOffset = u32(tripsBuf, o, nTrips); o += nTrips * 4;
  const tNstops = u16(tripsBuf, o, nTrips); o += nTrips * 2;
  const tHeadsign = u16(tripsBuf, o, nTrips);
  state.trips = { n: nTrips, route: tRoute, service: tService, start: tStart,
                  offset: tOffset, nstops: tNstops, headsign: tHeadsign, nSt };

  // stoptimes.bin: <4s I> then u16 packed stop, u16 seconds-since-start
  const nRows = new DataView(stBuf).getUint32(4, true);
  state.st = { packed: u16(stBuf, 8, nRows), arr: u16(stBuf, 8 + nRows * 2, nRows) };

  // walk.bin: <4s II> then u32 offsets[nStops+1], u16 stops, u16 metres
  const dvw = new DataView(walkBuf);
  const wStops = dvw.getUint32(4, true), wLinks = dvw.getUint32(8, true);
  let wo = 12;
  const wOff = u32(walkBuf, wo, wStops + 1); wo += (wStops + 1) * 4;
  const wPairs = u16(walkBuf, wo, wLinks); wo += wLinks * 2;
  const wMetres = u16(walkBuf, wo, wLinks);
  state.walk = { off: wOff, pairs: wPairs, metres: wMetres };

  // patterns.bin: <4s III> then u16 route, u32 stopOff, u32 tripOff, u16 stops, u32 trips
  const dvp = new DataView(patBuf);
  const nPat = dvp.getUint32(4, true), nPatStops = dvp.getUint32(8, true), nTripRefs = dvp.getUint32(12, true);
  let po = 16;
  const pRoute = u16(patBuf, po, nPat); po += nPat * 2;
  const pStopOff = u32(patBuf, po, nPat + 1); po += (nPat + 1) * 4;
  const pTripOff = u32(patBuf, po, nPat + 1); po += (nPat + 1) * 4;
  const patStops = u16(patBuf, po, nPatStops); po += nPatStops * 2;
  const patTrips = u32(patBuf, po, nTripRefs);
  state.pat = { n: nPat, route: pRoute, stopOff: pStopOff, tripOff: pTripOff,
                stops: patStops, trips: patTrips };
  if (state.shapes && state.shapes.nPat !== state.pat.n) {
    console.warn(`route shapes cover ${state.shapes.nPat} patterns, data has ${state.pat.n}; ignoring them`);
    state.shapes = null;                     // geometry built against a different dataset
  }

  // stoppatterns.bin: <4s II> then u32 offsets[nStops+1], u32 (pattern<<8 | pos)
  const dvsp = new DataView(spBuf);
  const spStops = dvsp.getUint32(4, true), spInc = dvsp.getUint32(8, true);
  state.sp = { off: u32(spBuf, 12, spStops + 1), pair: u32(spBuf, 12 + (spStops + 1) * 4, spInc) };

  // frequencies.bin: <4s I> then u32 trip, u32 start, u32 end, u16 headway
  const dvf = new DataView(frBuf);
  const nFr = dvf.getUint32(4, true);
  let fo = 8;
  const frTrip = u32(frBuf, fo, nFr); fo += nFr * 4;
  const frStart = u32(frBuf, fo, nFr); fo += nFr * 4;
  const frEnd = u32(frBuf, fo, nFr); fo += nFr * 4;
  const frHead = u16(frBuf, fo, nFr);
  const freq = new Map();
  for (let i = 0; i < nFr; i++) {
    const t = frTrip[i];
    let list = freq.get(t);
    if (!list) { list = []; freq.set(t, list); }
    list.push([frStart[i], frEnd[i], frHead[i]]);
  }
  state.freq = freq;
  if (nFr) console.info(`frequency windows: ${nFr} over ${freq.size} trips`);

  buildSearchIndex();
  if (!state.map) initMap();
  setStatus(`已載入 ${state.stops.length} 個站、${state.pat.n} 條路線組合；請輸入起點及終點。`);
  document.getElementById('go').disabled = !(state.from && state.to);
}

/* ------------------------------------------------------- keeping data current
 * The dataset is rebuilt in CI every 30 minutes. An open page polls meta.json
 * and, when a newer build appears, reloads the data and re-runs the current
 * query in place — no page refresh, no lost selection.
 */
const DATA_POLL_MS = 3 * 60 * 1000;
const AUTO_REFRESH_MS = 30 * 1000;          // re-plan against the current clock
let refreshing = false;

function rememberQuery(depart) {
  state.lastQuery = { from: state.from, to: state.to, depart };
}

async function checkForUpdate(manual = false) {
  if (refreshing || !state.stops.length) return;
  try {
    const r = await fetch(DATA + 'meta.json', { cache: 'no-store' });
    if (!r.ok) return;
    const meta = await r.json();
    const same = meta.digest ? meta.digest === state.digest : meta.generated === state.generated;
    if (!meta.generated || same) {
      if (manual) setStatus(`資料已是最新（${state.generated || '未知'} UTC 版本）。`);
      return;
    }
    refreshing = true;
    const hadQuery = state.lastQuery;
    await boot();
    if (hadQuery && state.from && state.to) run(true);
    setStatus(`資料已更新至 ${meta.generated} UTC 版本（班次 ${meta.counts.trips} 班）。`);
  } catch (e) {
    console.warn('update check failed', e);
  } finally {
    refreshing = false;
  }
}

/* -------------------------------------------------------------- search index */
const norm = (s) => (s || '').toLowerCase().replace(/[\s\-_()（）,，.、/]/g, '');
function buildSearchIndex() {
  state.searchIndex = state.stops.map((s, i) => ({
    i, name: (s[4] || s[3] || ''), en: s[3] || '', tc: s[4] || '',
    key: norm((s[4] || '') + ' ' + (s[3] || '')),
  }));
}

function searchStops(q, limit = 8) {
  const nq = norm(q);
  if (!nq) return [];
  const starts = [], contains = [];
  for (const s of state.searchIndex) {
    const k = s.key;
    if (k.startsWith(nq) || s.tc.startsWith(q) || s.en.toLowerCase().startsWith(nq)) starts.push(s);
    else if (k.includes(nq)) contains.push(s);
    if (starts.length >= limit) break;
  }
  return starts.concat(contains).slice(0, limit);
}

/* --------------------------------------------------------------- place search
 * Stops are searched locally and instantly. Anything that is not a stop (a mall,
 * a building, an address) goes to an OSM geocoder, restricted to the HK bounding
 * box: Photon first because its Hong Kong coverage of Chinese POI names is good
 * and it filters by bbox cleanly; Nominatim as a fallback when Photon is empty,
 * only on an explicit submit (its usage policy forbids autocomplete).
 */
const HK_BBOX = '113.83,22.15,114.45,22.57';
const placeCache = new Map();

function inHK(lat, lon) {
  return lat > 22.15 && lat < 22.57 && lon > 113.83 && lon < 114.45;
}

function photonResult(f, fallbackName) {
  const p = f.properties || {};
  const [lon, lat] = f.geometry.coordinates;
  if (!inHK(lat, lon)) return null;
  const detail = [p.street, p.district || p.city || p.county, p.postcode].filter(Boolean).join(' · ');
  return { kind: 'place', name: p.name || fallbackName, detail, lat, lon, source: 'Photon' };
}

async function searchPlaces(q) {
  const key = q.trim();
  if (key.length < 2) return [];
  if (placeCache.has(key)) return placeCache.get(key);
  let out = [];
  try {
    const url = `https://photon.komoot.io/api/?q=${encodeURIComponent(key)}&limit=6&lang=default&bbox=${HK_BBOX}`;
    const r = await fetch(url);
    if (r.ok) {
      const d = await r.json();
      out = (d.features || []).map(f => photonResult(f, key)).filter(Boolean).slice(0, 6);
    }
  } catch (e) {
    console.warn('Photon unavailable', e);
  }
  placeCache.set(key, out);
  return out;
}

async function searchPlacesNominatim(q) {
  const key = q.trim();
  if (key.length < 2) return [];
  const cacheKey = 'nom:' + key;
  if (placeCache.has(cacheKey)) return placeCache.get(cacheKey);
  let out = [];
  try {
    const url = 'https://nominatim.openstreetmap.org/search?format=jsonv2&limit=5&accept-language=zh-HK'
      + `&viewbox=${HK_BBOX.split(',').slice(0, 1)},${HK_BBOX.split(',')[3]},${HK_BBOX.split(',')[2]},${HK_BBOX.split(',')[1]}&bounded=1`
      + `&q=${encodeURIComponent(key)}`;
    const r = await fetch(url, { headers: { 'Accept-Language': 'zh-HK' } });
    if (r.ok) {
      const d = await r.json();
      out = d.map(x => ({
        kind: 'place', name: x.name || key,
        detail: (x.display_name || '').split(',').slice(1, 3).join(' · '),
        lat: parseFloat(x.lat), lon: parseFloat(x.lon), source: 'Nominatim',
      })).filter(x => inHK(x.lat, x.lon));
    }
  } catch (e) {
    console.warn('Nominatim unavailable', e);
  }
  placeCache.set(cacheKey, out);
  return out;
}

/* ------------------------------------------------------------------- service */
/** Hong Kong wall-clock time, regardless of the browser's timezone. */
function hkNow() {
  const parts = new Intl.DateTimeFormat('en-GB', {
    timeZone: 'Asia/Hong_Kong', hour12: false,
    year: 'numeric', month: '2-digit', day: '2-digit',
    hour: '2-digit', minute: '2-digit', second: '2-digit',
  }).formatToParts(new Date());
  const g = (t) => Number(parts.find((p) => p.type === t).value);
  return new Date(Date.UTC(g('year'), g('month') - 1, g('day'), g('hour') % 24, g('minute'), g('second')));
}

function activeServices(date = hkNow()) {
  const bits = 1 << ((date.getUTCDay() + 6) % 7);
  const ymd = date.getUTCFullYear() * 10000 + (date.getUTCMonth() + 1) * 100 + date.getUTCDate();
  const set = new Set();
  for (const sid of Object.keys(state.cal.calendar)) {
    const [mask, start, end] = state.cal.calendar[sid];
    if (ymd >= start && ymd <= end && (mask & bits)) set.add(state.calIndex[sid]);
  }
  for (const [sid, d, type] of state.cal.exceptions) {
    if (d !== ymd) continue;
    const i = state.calIndex[sid];
    if (i === undefined) continue;
    if (type === 1) set.add(i); else set.delete(i);
  }
  return set;
}

/* -------------------------------------------------------------- route shapes
 * shapes/shapes.bin holds, per pattern, the polyline along OSM roads plus the
 * vertex index of every stop, so an alighting-to-boarding slice is a slice.
 */
async function loadShapes() {
  try {
    const b = await (await fetch('shapes/shapes.bin', { cache: 'no-cache' })).arrayBuffer();
    const dv = new DataView(b);
    const magic = String.fromCharCode(...new Uint8Array(b, 0, 4));
    if (magic !== 'KHS1') throw new Error(`bad magic ${magic}`);
    const nPat = dv.getUint32(4, true), nPts = dv.getUint32(8, true), nStopIdx = dv.getUint32(12, true);
    let o = 16;
    const pointOff = u32(b, o, nPat + 1); o += (nPat + 1) * 4;
    const stopOff = u32(b, o, nPat + 1); o += (nPat + 1) * 4;
    const pts = new Int32Array(b.slice(o, o + nPts * 8)); o += nPts * 8;
    const stopVertex = u16(b, o, nStopIdx);
    return { nPat, pointOff, stopOff, pts, stopVertex };
  } catch (e) {
    console.warn('route shapes unavailable, drawing straight lines', e);
    return null;
  }
}

/**
 * Walking geometry: the footpath polyline for every kerb pair within walking
 * distance, built from the OSM pedestrian network (shapes/walk.bin).
 * `KHW1` + u32 nPairs, u32 nPts, then pairA[n], pairB[n], pointOff[n+1],
 * and interleaved int32 lat/lon at 1e7.
 */
async function loadWalkShapes() {
  try {
    const b = await (await fetch('shapes/walk.bin', { cache: 'no-cache' })).arrayBuffer();
    const dv = new DataView(b);
    const magic = String.fromCharCode(...new Uint8Array(b, 0, 4));
    if (magic !== 'KHW1') throw new Error(`bad magic ${magic}`);
    const nPairs = dv.getUint32(4, true), nPts = dv.getUint32(8, true);
    let o = 12;
    const pairA = u32(b, o, nPairs); o += nPairs * 4;
    const pairB = u32(b, o, nPairs); o += nPairs * 4;
    const off = u32(b, o, nPairs + 1); o += (nPairs + 1) * 4;
    const pts = new Int32Array(b.slice(o, o + nPts * 8));
    const index = new Map();
    for (let i = 0; i < nPairs; i++) {
      const a = Math.min(pairA[i], pairB[i]), c = Math.max(pairA[i], pairB[i]);
      if (a >= state.stops.length || c >= state.stops.length) continue;
      index.set(a * state.stops.length + c, i);
    }
    return { nPairs, nPts, off, pts, index };
  } catch (e) {
    console.warn('walk shapes unavailable, drawing straight lines', e);
    return null;
  }
}

function walkGeometry(a, c) {
  const w = state.walkShapes;
  if (!w) return null;
  const lo = Math.min(a, c), hi = Math.max(a, c);
  const i = w.index.get(lo * state.stops.length + hi);
  if (i === undefined) return null;
  const out = [];
  for (let k = w.off[i]; k < w.off[i + 1]; k++) out.push([w.pts[k * 2] / 1e7, w.pts[k * 2 + 1] / 1e7]);
  if (a > c) out.reverse();                     // keep the walking direction
  return out.length > 1 ? out : null;
}

function legGeometry(pi, board, alight) {
  const s = state.shapes;
  if (!s || pi === undefined || pi === null || pi < 0 || pi >= s.nPat) return null;
  const a = s.stopVertex[s.stopOff[pi] + board];
  const b = s.stopVertex[s.stopOff[pi] + alight];
  if (a === undefined || b === undefined || b <= a) return null;
  const base = s.pointOff[pi];
  const out = [];
  for (let i = a; i <= b; i++) {
    out.push([s.pts[(base + i) * 2] / 1e5, s.pts[(base + i) * 2 + 1] / 1e5]);
  }
  return out.length > 1 ? out : null;
}

/* ------------------------------------------------------------------ geometry */
const R = 6371000;

/**
 * Every departure time of trip `ti` from stop-position `pos` inside [after, horizon].
 *
 * A trip normally departs once. A trip listed in frequencies.bin runs repeatedly
 * (a template trip plus a window: "every 540 s between 07:12 and 07:39"), so the
 * window is expanded here instead of materialising ~5x more trips in the dataset.
 */
function depTimes(ti, pos, after, horizon) {
  const row = state.trips.offset[ti] + pos;
  const base = state.trips.start[ti] + state.st.arr[row];
  const windows = state.freq && state.freq.get(ti);
  if (!windows) return base >= after && base <= horizon ? [base] : [];
  const shift = base - state.trips.start[ti];          // travel time to this stop
  const out = [];
  for (const [ws, we, h] of windows) {
    let k = Math.max(0, Math.ceil((after - ws - shift) / h));
    for (;;) {
      const dep = ws + k * h + shift;
      if (ws + k * h > we || dep > horizon) break;
      if (dep >= after) out.push(dep);
      k += 1;
    }
    if (out.length > 40) break;                        // plenty for any query
  }
  if (base >= after && base <= horizon && !out.includes(base)) out.push(base);
  return out.sort((a, b) => a - b);
}

function haversine(aLat, aLon, bLat, bLon) {
  const p1 = aLat * Math.PI / 180, p2 = bLat * Math.PI / 180;
  const dp = p2 - p1, dl = (bLon - aLon) * Math.PI / 180;
  const x = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dl / 2) ** 2;
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(x)));
}

function stopsNear(lat, lon, radius) {
  const out = [];
  for (let i = 0; i < state.stops.length; i++) {
    const s = state.stops[i];
    const d = haversine(lat, lon, s[1], s[2]);
    if (d <= radius) out.push({ stop: i, metres: d, seconds: Math.round(d / WALK_SPEED) });
  }
  out.sort((a, b) => a.metres - b.metres);
  return out.slice(0, 12);
}

/* ------------------------------------------------------------------- routing */
class MinHeap {
  constructor() { this.a = []; this.t = []; }
  get size() { return this.a.length; }
  push(stop, time) {
    this.a.push(stop); this.t.push(time);
    let i = this.a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (this.t[p] <= this.t[i]) break;
      this._swap(i, p); i = p;
    }
  }
  pop() {
    const top = [this.a[0], this.t[0]];
    const lastS = this.a.pop(), lastT = this.t.pop();
    if (this.a.length) {
      this.a[0] = lastS; this.t[0] = lastT;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < this.a.length && this.t[l] < this.t[m]) m = l;
        if (r < this.a.length && this.t[r] < this.t[m]) m = r;
        if (m === i) break;
        this._swap(i, m); i = m;
      }
    }
    return top;
  }
  _swap(i, j) {
    [this.a[i], this.a[j]] = [this.a[j], this.a[i]];
    [this.t[i], this.t[j]] = [this.t[j], this.t[i]];
  }
}

/**
 * Earliest time trip `ti` departs the stop in row `row`, at or after `after`.
 *
 * A trip normally runs once; a trip carrying a frequency window runs repeatedly
 * ("every 540 s between 07:12 and 07:39"), so the k-th run is resolved here
 * arithmetically instead of materialising the repeats in the dataset.
 */
function firstDep(ti, row, after) {
  const startT = state.trips.start[ti];
  const base = startT + state.st.arr[row];
  let bestDep = base >= after ? base : null;
  const windows = state.freq && state.freq.get(ti);
  if (windows) {
    const shift = base - startT;
    for (const [ws, we, h] of windows) {
      let k = Math.ceil((after - ws - shift) / h);
      if (k < 0) k = 0;
      const runStart = ws + k * h;
      if (runStart > we) continue;                    // window already over
      const t = runStart + shift;
      if (t >= after && (bestDep === null || t < bestDep)) bestDep = t;
    }
  }
  return bestDep;
}

/** Earliest catchable trip of `pattern` at `pos`, departing at/after `after`. */
function findTrip(pi, pos, after, active) {
  const { start, offset, service } = state.trips;
  const list = state.pat.trips;
  const from = state.pat.tripOff[pi], to = state.pat.tripOff[pi + 1];
  let lo = from, hi = to - 1, ans = -1;
  while (lo <= hi) {
    const mid = (lo + hi) >> 1;
    if (start[list[mid]] >= after - MAX_OFFSET_SLACK) { ans = mid; hi = mid - 1; } else lo = mid + 1;
  }
  if (ans < 0) return null;
  let best = null, scanned = 0;
  // Scan *backwards* as well as forwards: a trip carrying a frequency window runs
  // long after its template start (a 09:35 trip may still be departing at 15:35),
  // so the template the binary search lands on is often not the earliest *run*.
  const fromK = Math.max(from, ans - 80);
  for (let k = fromK; k < to && scanned < 220; k++, scanned++) {
    const ti = list[k];
    if (start[ti] > after + 6 * 3600) break;
    if (!active.has(service[ti])) continue;
    const row = offset[ti] + pos;
    if (!(state.st.packed[row] & (1 << 13))) continue;          // no pick-up here
    const dep = firstDep(ti, row, after);
    if (dep === null) continue;
    if (!best || dep < best.dep) {
      // a frequency run departs later than its template trip, so remember the shift
      best = { trip: ti, dep, offset: Math.round(dep - (start[ti] + state.st.arr[row])) };
    }
  }
  return best;
}

function plan(origin, dest, departSec, options = {}) {
  const nStops = state.stops.length;
  const active = activeServices(options.date);
  const banned = options.bannedPat || new Set();
  const tau = new Float64Array(nStops).fill(INF);
  const parent = new Array(nStops).fill(null);
  const seeded = new Uint8Array(nStops);
  const pq = new MinHeap();

  const origins = origin.stop !== undefined
    ? [{ stop: origin.stop, metres: 0, seconds: 0 }]
    : stopsNear(origin.lat, origin.lon, ORIGIN_WALK_M);

  for (const s of origins) {
    const t = departSec + s.seconds;
    if (t < tau[s.stop]) {
      tau[s.stop] = t;
      parent[s.stop] = { type: 'walk0', metres: s.metres };
      seeded[s.stop] = 1;
      pq.push(s.stop, t);
    }
  }

  const bestBoard = new Float64Array(state.pat.n).fill(INF);
  const bestPos = new Int32Array(state.pat.n).fill(-1);

  while (pq.size) {
    const [s, t] = pq.pop();
    if (t > tau[s] + 1e-6) continue;

    // walking transfers out of this stop
    for (let k = state.walk.off[s]; k < state.walk.off[s + 1]; k++) {
      const n = state.walk.pairs[k];
      const cand = t + Math.max(30, Math.round(state.walk.metres[k] / WALK_SPEED));
      if (cand < tau[n] - 1e-6) {
        tau[n] = cand;
        parent[n] = { type: 'walk', from: s, metres: state.walk.metres[k] };
        pq.push(n, cand);
      }
    }

    // boardings
    for (let k = state.sp.off[s]; k < state.sp.off[s + 1]; k++) {
      const v = state.sp.pair[k];
      const pi = v >>> 8, pos = v & 0xff;
      if (banned.has(pi)) continue;
      if (bestBoard[pi] <= t && bestPos[pi] <= pos) continue;     // pattern already boarded better
      const ready = t + (seeded[s] ? 0 : TRANSFER_PENALTY);
      const found = findTrip(pi, pos, ready, active);
      if (!found) continue;
      const { trip, dep } = found;
      const shift = found.offset || 0;                            // frequency-run displacement
      bestBoard[pi] = dep; bestPos[pi] = pos;

      const pStopBase = state.pat.stopOff[pi];
      const nPatStops = state.pat.stopOff[pi + 1] - pStopBase;
      const rowBase = state.trips.offset[trip];
      for (let p = pos + 1; p < nPatStops; p++) {
        const sidx = state.pat.stops[pStopBase + p];
        const row = rowBase + p;
        if (!(state.st.packed[row] & (1 << 14))) continue;       // no set-down here
        const arr = state.trips.start[trip] + state.st.arr[row] + shift;
        if (arr < tau[sidx] - 1e-6) {
          tau[sidx] = arr;
          parent[sidx] = { type: 'bus', from: s, trip, board: pos, alight: p, boardTime: dep, shift };
          pq.push(sidx, arr);
        }
      }
    }
  }

  // pick the best arrival at the destination
  let best = null;
  if (dest.stop !== undefined) {
    if (tau[dest.stop] < INF) best = { stop: dest.stop, time: tau[dest.stop], walk: 0 };
  } else {
    for (const s of stopsNear(dest.lat, dest.lon, ORIGIN_WALK_M)) {
      if (tau[s.stop] >= INF) continue;
      const t = tau[s.stop] + s.seconds;
      if (!best || t < best.time) best = { stop: s.stop, time: t, walk: s.metres };
    }
  }
  if (!best) return null;

  // rebuild the itinerary
  const legs = [];
  let cur = best.stop;
  const guard = 200;
  let steps = 0;
  while (cur !== undefined && cur !== null && steps++ < guard) {
    const p = parent[cur];
    if (!p) break;
    if (p.type === 'walk0') {
      legs.push({ type: 'walk', metres: p.metres, to: cur });
      break;
    } else if (p.type === 'walk') {
      legs.push({ type: 'walk', metres: p.metres, from: p.from, to: cur });
      cur = p.from;
    } else {
      legs.push({ type: 'bus', trip: p.trip, boardStop: p.from, alightStop: cur,
                  board: p.board, alight: p.alight, shift: p.shift || 0 });
      cur = p.from;
    }
  }
  legs.reverse();

  // walk from the last stop to the actual destination point (added before the
  // merge so it collapses with an alighting walk into one leg)
  if (best.walk > 0) {
    legs.push({ type: 'walk', metres: best.walk, toDest: true, from: best.stop });
  }

  // collapse chains of walking legs (a multi-hop walk is one walk to the rider)
  const merged = [];
  for (const leg of legs) {
    const prev = merged[merged.length - 1];
    if (leg.type === 'walk' && prev && prev.type === 'walk') {
      prev.metres += leg.metres;
      prev.to = leg.to;
      if (leg.toDest) prev.toDest = true;
      continue;
    }
    merged.push({ ...leg });
  }

  let t = departSec;
  for (const leg of merged) {
    if (leg.type === 'walk') {
      leg.start = t;
      leg.seconds = Math.round(leg.metres / WALK_SPEED);
      leg.end = t + leg.seconds;
      t = leg.end;
    } else {
      const ti = leg.trip;
      const sh = leg.shift || 0;                                  // frequency-run displacement
      leg.start = state.trips.start[ti] + state.st.arr[state.trips.offset[ti] + leg.board] + sh;
      leg.end = state.trips.start[ti] + state.st.arr[state.trips.offset[ti] + leg.alight] + sh;
      leg.routeIdx = state.trips.route[ti];
      leg.seconds = leg.end - leg.start;
      leg.stopCount = leg.alight - leg.board;
      leg.headsign = state.trips.headsign[ti];
      leg.pattern = patternOfTrip(ti);
      t = leg.end;
    }
  }
  return { depart: departSec, arrive: t, legs: merged };
}

const tripPattern = new Map();
function patternOfTrip(ti) {
  if (tripPattern.size === 0) {
    for (let pi = 0; pi < state.pat.n; pi++) {
      for (let k = state.pat.tripOff[pi]; k < state.pat.tripOff[pi + 1]; k++) {
        tripPattern.set(state.pat.trips[k], pi);
      }
    }
  }
  return tripPattern.get(ti);
}

/* --------------------------------------------------------------------- helpers */
function hhmm(sec) {
  const s = ((sec % 86400) + 86400) % 86400;
  return `${String(Math.floor(s / 3600)).padStart(2, '0')}:${String(Math.floor(s % 3600 / 60)).padStart(2, '0')}`;
}
function mins(sec) { return Math.max(1, Math.round(sec / 60)); }
function stopName(i) {
  const s = state.stops[i];
  return (s && (s[4] || s[3])) || '未知車站';
}
function stopLabel(i) {
  const s = state.stops[i];
  if (!s) return '未知車站';
  return s[4] ? `${s[4]}` : s[3];
}
function routeShort(ri) { return state.routes[ri] ? state.routes[ri][1] : '?'; }
function headsignText(h) {
  return (h !== 0xFFFF && state.headsigns[h]) ? `往 ${state.headsigns[h][3]}` : '';
}
function setStatus(text) { document.getElementById('status').textContent = text; }

/* ------------------------------------------------------------------------ map */
function initMap() {
  const map = L.map('map', { zoomControl: true }).setView([22.335, 114.17], 12);
  L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright" target="_blank">OpenStreetMap</a> contributors',
  }).addTo(map);
  state.map = map;
  map.on('click', (e) => {
    if (!state.picking) return;
    const which = state.picking;
    state.picking = null;
    document.querySelectorAll('button.ghost').forEach(b => b.classList.remove('picking'));
    pickPoint(which, e.latlng.lat, e.latlng.lng);
  });
  const pointIcon = (label, colour) => L.divIcon({
    className: '', iconSize: [30, 30], iconAnchor: [15, 15],
    html: `<div style="background:${colour};color:#fff;border-radius:50%;width:28px;height:28px;display:flex;align-items:center;justify-content:center;font-weight:700;font-size:13px;border:2px solid #fff;box-shadow:0 1px 4px rgba(0,0,0,.4)">${label}</div>`,
  });
  state.pointIcon = pointIcon;
}

function clearMap() {
  for (const l of state.mapLayers) state.map.removeLayer(l);
  state.mapLayers = [];
}
function addLayer(l) { l.addTo(state.map); state.mapLayers.push(l); return l; }

function drawPlan(plan) {
  clearMap();
  const pts = [];
  if (state.from) {
    const p = coordOf(state.from);
    pts.push([p.lat, p.lon]);
    addLayer(L.marker([p.lat, p.lon], { icon: state.pointIcon('A', '#1a7f37') }));
  }
  for (const leg of plan.legs) {
    if (leg.type === 'bus') {
      const { pattern: pi, board, alight } = leg;
      let line = legGeometry(pi, board, alight);
      if (!line) {                          // no alignment for this pattern: straight hops
        const base = state.pat.stopOff[pi];
        line = [];
        for (let p = board; p <= alight; p++) {
          const s = state.stops[state.pat.stops[base + p]];
          line.push([s[1], s[2]]);
        }
      }
      addLayer(L.polyline(line, { color: '#b4232b', weight: 5, opacity: .85, smoothFactor: 1 }));
      const b = line[0], a = line[line.length - 1];
      addLayer(L.circleMarker(b, { radius: 4, color: '#b4232b', fillColor: '#fff', fillOpacity: 1, weight: 2 }));
      addLayer(L.circleMarker(a, { radius: 4, color: '#b4232b', fillColor: '#fff', fillOpacity: 1, weight: 2 }));
      pts.push(...line);
    } else if (leg.type === 'walk' && leg.metres > 25) {
      const line = walkLine(leg);
      if (line && line.length > 1) {
        addLayer(L.polyline(line, { color: '#6b6b70', weight: 3, dashArray: '5,6', opacity: .9 }));
      }
    }
  }
  if (state.to) {
    const p = coordOf(state.to);
    pts.push([p.lat, p.lon]);
    addLayer(L.marker([p.lat, p.lon], { icon: state.pointIcon('B', '#b4232b') }));
  }
  if (pts.length) state.map.fitBounds(pts, { padding: [40, 40] });
}

/**
 * The polyline to draw for a walking leg.
 *
 * Stop→stop transfers use the precomputed footpath directly. Walks that start
 * or end at an arbitrary picked place have no precomputed geometry, so they are
 * drawn as: the point, a short hop to the nearest kerb, then that kerb's real
 * footpath to the destination stop. If the detour is silly, fall back to a line.
 */
function walkLine(leg) {
  const st = state.stops;
  const fromStop = leg.from !== undefined ? leg.from : null;
  const toStop = leg.to !== undefined ? leg.to : null;

  if (fromStop !== null && toStop !== null) {
    const geo = walkGeometry(fromStop, toStop);
    if (geo) return geo;
    return [[st[fromStop][1], st[fromStop][2]], [st[toStop][1], st[toStop][2]]];
  }

  const point = coordOf(leg.from === undefined ? state.from : state.to);
  const stopIdx = fromStop !== null ? fromStop : toStop;
  if (stopIdx === null || stopIdx === undefined) return null;
  const target = [st[stopIdx][1], st[stopIdx][2]];
  const toStopWalk = endWalkPath(point, stopIdx);
  const line = toStopWalk || [[point.lat, point.lon], target];
  // keep the drawn line in the same ballpark as the quoted metres
  let length = 0;
  for (let i = 1; i < line.length; i++) length += haversine(line[i - 1][0], line[i - 1][1], line[i][0], line[i][1]);
  const ok = length <= Math.max(1.7 * Math.max(leg.metres, 60), leg.metres + 90);
  const draw = ok ? line : [[point.lat, point.lon], target];
  return leg.from === undefined ? draw : draw.slice().reverse();   // stop → place walks backwards
}

/**
 * Footpath from an arbitrary point to a stop: snap the point to the nearest
 * vertex of the footpaths that feed that stop, then follow that path in.
 */
function endWalkPath(point, stopIdx) {
  const st = state.stops;
  let best = null;
  for (let k = state.walk.off[stopIdx]; k < state.walk.off[stopIdx + 1]; k++) {
    const n = state.walk.pairs[k];
    let geo = walkGeometry(n, stopIdx);
    if (!geo) { geo = walkGeometry(stopIdx, n); if (geo) geo = geo.slice().reverse(); }
    if (!geo) continue;
    const end = geo[geo.length - 1];
    if (haversine(end[0], end[1], st[stopIdx][1], st[stopIdx][2]) > 40) continue;  // not pointing at the kerb
    for (let i = 0; i < geo.length; i++) {
      const d = haversine(point.lat, point.lon, geo[i][0], geo[i][1]);
      if (!best || d < best.dist) best = { geo, idx: i, dist: d };
    }
  }
  if (!best || best.dist > 350) return null;
  return [[point.lat, point.lon], ...best.geo.slice(best.idx)];
}

function coordOf(sel) {
  if (sel.stop !== undefined) {
    const s = state.stops[sel.stop];
    return { lat: s[1], lon: s[2] };
  }
  return { lat: sel.lat, lon: sel.lon };
}

/* ------------------------------------------------------------------------- UI */
function setupField(which) {
  const input = document.getElementById(`${which}-input`);
  const list = document.getElementById(`${which}-suggest`);
  const picked = document.getElementById(`${which}-picked`);
  if (!input || !list || !picked) {
    console.warn(`field ${which} missing from the page; skipping wiring`);
    return { setPoint() {} };
  }
  let items = [], cursor = -1, placeSeq = 0, timer = null;

  const render = () => {
    list.innerHTML = '';
    let lastKind = null;
    items.forEach((s, idx) => {
      const groupKind = s.kind === 'note' ? 'place' : s.kind;
      if (groupKind !== lastKind) {
        lastKind = groupKind;
        const h = document.createElement('li');
        h.className = 'group';
        h.textContent = groupKind === 'stop' ? '巴士站' : '地點（網上搜尋）';
        list.appendChild(h);
      }
      const li = document.createElement('li');
      if (idx === cursor) li.className = 'active';
      if (s.kind === 'note') {
        li.className = 'note';
        li.textContent = s.text;
        list.appendChild(li);
        return;
      }
      if (s.kind === 'stop') {
        li.innerHTML = `<span>${s.tc || s.en}</span><span class="en">${s.tc ? s.en : ''}</span>`;
      } else {
        li.innerHTML = `<span>📍 ${s.name}</span><span class="en">${s.detail || ''}</span>`;
      }
      li.addEventListener('mousedown', (e) => { e.preventDefault(); choose(s); });
      list.appendChild(li);
    });
    list.hidden = items.length === 0;
  };

  const choose = (s) => {
    if (!s || s.kind === 'note') return;
    if (s.kind === 'place') {
      state[which] = { lat: s.lat, lon: s.lon, label: s.name };
      input.value = s.name;
      picked.hidden = false;
      picked.textContent = `已選：${s.name}` + (s.detail ? `（${s.detail}）` : '');
    } else {
      state[which] = { stop: s.i };
      input.value = s.tc || s.en;
      picked.hidden = false;
      picked.textContent = `已選：${stopName(s.i)}`;
    }
    list.hidden = true;
    items = [];
    cursor = -1;
    maybeRedraw();
  };

  // stops are instant; places arrive from the geocoder after a pause
  const refreshPlaces = async (query) => {
    const seq = ++placeSeq;
    if (query.trim().length < 2) return;
    const stopsNow = searchStops(query).map(s => ({ ...s, kind: 'stop' }));
    items = stopsNow.concat([{ kind: 'note', text: '搜尋地點中…' }]);
    cursor = stopsNow.length ? 0 : -1;
    render();
    const places = await searchPlaces(query);
    if (seq !== placeSeq) return;                      // a newer keystroke won
    if (!picked.hidden) return;                        // a choice was already made
    items = stopsNow.concat(places);
    cursor = items.length ? 0 : -1;
    render();
  };

  input.addEventListener('input', () => {
    const q = input.value;
    items = searchStops(q).map(s => ({ ...s, kind: 'stop' }));
    cursor = items.length ? 0 : -1;
    render();
    picked.hidden = true;
    state[which] = null;
    maybeRedraw();
    clearTimeout(timer);
    if (q.trim().length >= 2) timer = setTimeout(() => refreshPlaces(q), 450);
  });

  input.addEventListener('keydown', async (e) => {
    if (e.key === 'ArrowDown' && items.length) { cursor = Math.min(cursor + 1, items.length - 1); render(); e.preventDefault(); return; }
    if (e.key === 'ArrowUp' && items.length) { cursor = Math.max(cursor - 1, 0); render(); e.preventDefault(); return; }
    if (e.key === 'Escape') { list.hidden = true; return; }
    if (e.key !== 'Enter') return;
    e.preventDefault();
    if (items[cursor] && items[cursor].kind !== 'note') { choose(items[cursor]); return; }
    if (items.some(i => i.kind === 'place')) { choose(items.find(i => i.kind === 'place')); return; }
    // nothing local: ask Nominatim once, on explicit submit
    const q = input.value.trim();
    if (q.length < 2) return;
    setStatus(`搜尋「${q}」…`);
    const places = await searchPlacesNominatim(q);
    if (places.length) {
      items = places;
      cursor = 0;
      render();
      choose(places[0]);
      setStatus(`已找到「${places[0].name}」，再按「規劃路線」。`);
    } else {
      setStatus(`找不到「${q}」，請試其他名稱，或在地圖上點選位置。`);
    }
  });
  input.addEventListener('blur', () => setTimeout(() => { list.hidden = true; }, 150));

  document.getElementById(`${which}-map`).addEventListener('click', (e) => {
    state.picking = which;
    document.querySelectorAll('button.ghost').forEach(b => b.classList.remove('picking'));
    e.currentTarget.classList.add('picking');
    setStatus('請在地圖上點選位置…');
  });

  return { setPoint(lat, lon, label) {
    state[which] = { lat, lon, label: label || null };
    input.value = label || `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
    picked.hidden = false;
    picked.textContent = label ? `已選：${label}` : '已選：地圖上的座標';
    maybeRedraw();
  } };
}

let fieldFrom, fieldTo;

function pickPoint(which, lat, lon) {
  (which === 'from' ? fieldFrom : fieldTo).setPoint(lat, lon);
  setStatus('已在地圖上選取位置。');
  maybeRedraw();
}

function locate() {
  if (!navigator.geolocation) { setStatus('此瀏覽器不支援定位。'); return; }
  setStatus('正在取得位置…');
  navigator.geolocation.getCurrentPosition(
    (pos) => {
      fieldFrom.setPoint(pos.coords.latitude, pos.coords.longitude, '我的位置');
      setStatus('已使用你的位置作為起點。');
    },
    (err) => setStatus(`無法取得位置：${err.message}`),
    { enableHighAccuracy: true, timeout: 10000, maximumAge: 60000 },
  );
}

function maybeRedraw() {
  const go = document.getElementById('go');
  go.disabled = !(state.from && state.to);
}

/**
 * The next `count` departures of a *route* at a boarding stop, from the timetable.
 *
 * Trips are grouped into patterns by exact stop sequence, so one route can be
 * split across several patterns (a variant that skips a stop, a short working).
 * Asking only the pattern the plan used reports a 100-minute wait on a route that
 * runs every 15 minutes, so this walks every pattern of the same route and the
 * same headsign and merges their departures at the kerb.
 */
function nextDepartures(routeIdx, boardStop, headsignIdx, afterSec, count = 3, horizonSec = 5400) {
  const active = activeServices();
  const cands = [];
  const horizon = afterSec + horizonSec;      // "next bus" means within the next while,
  for (let pi = 0; pi < state.pat.n; pi++) {  // not the 17:09 departure while it is 10:39
    if (state.pat.route[pi] !== routeIdx) continue;
    const firstTrip = state.pat.trips[state.pat.tripOff[pi]];
    if (headsignIdx !== undefined && headsignIdx !== 0xFFFF
        && state.trips.headsign[firstTrip] !== headsignIdx) continue;
    const base = state.pat.stopOff[pi];
    for (let p = base; p < state.pat.stopOff[pi + 1]; p++) {
      if (state.pat.stops[p] !== boardStop) continue;
      const pos = p - base;
      for (let k = state.pat.tripOff[pi]; k < state.pat.tripOff[pi + 1]; k++) {
        const ti = state.pat.trips[k];
        if (!active.has(state.trips.service[ti])) continue;
        const row = state.trips.offset[ti] + pos;
        if (!(state.st.packed[row] & (1 << 13))) continue;      // cannot board here
        for (const dep of depTimes(ti, pos, afterSec, horizon)) cands.push({ trip: ti, dep });
      }
    }
  }
  cands.sort((a, b) => a.dep - b.dep);
  const out = [];
  for (const c of cands) {
    if (out.length && c.dep - out[out.length - 1].dep < 60) continue;   // same bus, other variant
    out.push(c);
    if (out.length >= count) break;
  }
  return out;
}

/**
 * Ovals sitting right next to the direction: the next buses of this route at this
 * kerb, counted in minutes **from the journey's departure time** so every oval in
 * the itinerary shares one reference point. 1st green, 2nd amber, 3rd grey.
 */
function renderNextBuses(list, planDepart) {
  if (!list || !list.length) return '';
  return list.slice(0, 3).map((d, i) => {
    const wait = Math.max(0, Math.round((d.dep - planDepart) / 60));
    const label = ['下一班', '第二班', '第三班'][i];
    return `<i class="eta r${i + 1}" title="${label} · ${hhmm(d.dep)} 開出（出發後 ${wait} 分鐘）">${wait} 分</i>`;
  }).join('');
}

function summaryHTML(plan) {
  const transfers = plan.legs.filter(l => l.type === 'bus').length - 1;
  const walkMetres = plan.legs.filter(l => l.type === 'walk').reduce((a, l) => a + l.metres, 0);
  return `
    <span class="total">${mins(plan.arrive - plan.depart)} 分鐘</span>
    <span class="times">${hhmm(plan.depart)} 出發 → ${hhmm(plan.arrive)} 抵達</span>
    <span class="times">${transfers > 0 ? `轉乘 ${transfers} 次` : '直達'}${walkMetres >= 50 ? ` · 步行 ${walkMetres >= 1000 ? (walkMetres / 1000).toFixed(1) + ' 公里' : Math.round(walkMetres) + ' 米'}` : ''}</span>`;
}

function legsHTML(plan) {
  let html = '';
  let prevEnd = plan.depart;
  for (const leg of plan.legs) {
    if (leg.type === 'walk' && leg.metres < 40) { prevEnd = leg.end; continue; }   // same-kerb shuffle
    const wait = leg.start - prevEnd;
    if (wait >= 180) {
      html += `<div class="leg wait"><span class="when">${hhmm(prevEnd)}</span>
        <span class="what">等候 ${mins(wait)} 分鐘</span></div>`;
    }
    if (leg.type === 'walk') {
      html += `<div class="leg walk">
        <span class="when">${hhmm(leg.start)} – ${hhmm(leg.end)}</span>
        <span class="what">步行 ${mins(leg.seconds)} 分鐘（約 ${Math.round(leg.metres)} 米）
          ${leg.toDest ? '至目的地' : `至 ${stopName(leg.to)}`}</span></div>`;
    } else {
      const badge = `<span class="route-badge">${routeShort(leg.routeIdx)}</span>`;
      const next = nextDepartures(leg.routeIdx, leg.boardStop, leg.headsign, prevEnd - 30, 3);
      html += `<div class="leg bus">
        <span class="when">${hhmm(leg.start)} – ${hhmm(leg.end)}</span>
        <span class="what">${badge}${headsignText(leg.headsign)}${renderNextBuses(next, plan.depart)}
          <span class="stops"><br>${stopName(leg.boardStop)} 上車 · ${stopName(leg.alightStop)} 落車 ·
          ${leg.stopCount} 個站 · ${mins(leg.seconds)} 分鐘</span></span></div>`;
    }
    prevEnd = leg.end;
  }
  return html;
}

function renderResult(plan, container, index) {
  const card = document.createElement('article');
  card.className = 'card' + (index === 0 ? ' active' : '');

  const head = document.createElement('div');
  head.className = 'summary';
  head.innerHTML = summaryHTML(plan);
  card.appendChild(head);

  const legs = document.createElement('div');
  legs.className = 'legs';
  legs.innerHTML = legsHTML(plan);
  card.appendChild(legs);

  card.addEventListener('click', () => {
    document.querySelectorAll('.card').forEach(c => c.classList.remove('active'));
    card.classList.add('active');
    drawPlan(card.__plan || plan);
  });
  card.__plan = plan;
  container.appendChild(card);
  return card;
}

/**
 * Redraw one card in place from a fresh plan. Used by the 30-second ticker when
 * the itinerary itself is unchanged but the countdowns have moved — replacing the
 * legs block keeps the map, the pan/zoom and the reader's scroll position intact.
 */
function refreshCard(card, plan) {
  if (!card || !plan) return;
  card.__plan = plan;
  const head = card.querySelector('.summary');
  if (head) head.innerHTML = summaryHTML(plan);
  const legs = card.querySelector('.legs');
  if (legs) legs.innerHTML = legsHTML(plan);
}

/** Departure time is simply "when the user pressed 規劃路線", in Hong Kong time. */
function nowSeconds() {
  const d = hkNow();
  return d.getUTCHours() * 3600 + d.getUTCMinutes() * 60 + d.getUTCSeconds();
}

/**
 * Identity of an itinerary: the legs you would actually ride. It deliberately
 * ignores clock times and countdowns, so the ticker can tell "same journey, the
 * seconds moved" apart from "a different journey".
 */
function structureSig(p) {
  if (!p) return 'null';
  return p.legs.map(l => l.type === 'walk'
    ? `w${Math.round(l.metres)}`
    : `b${l.trip}:${l.board}:${l.alight}`).join(',');
}

/**
 * Re-plan the current origin/destination from *now*. When `auto` is set this is
 * the 30-second ticker: it only touches the DOM if the itinerary actually changed,
 * so a rider reading the results is not disturbed by a redraw every half minute.
 */
function run(auto = false) {
  if (!state.from || !state.to) return;
  const container = document.getElementById('results');
  const depart = nowSeconds();
  rememberQuery(depart);

  // a coordinate origin/destination only works if a stop is walkable from it
  for (const [sel, name] of [[state.from, '起點'], [state.to, '終點']]) {
    if (sel && sel.stop === undefined && stopsNear(sel.lat, sel.lon, ORIGIN_WALK_M).length === 0) {
      if (auto) return;
      container.innerHTML = `<div class="empty">${name} ${ORIGIN_WALK_M} 米範圍內沒有九巴／龍運車站，
        這個位置無法只用九巴／龍運及步行到達。</div>`;
      setStatus('沒有結果。');
      return;
    }
  }

  const t0 = performance.now();
  const first = plan(state.from, state.to, depart);
  if (!first) {
    if (auto) return;                       // keep the last good result on screen
    container.innerHTML = `<div class="empty">找不到只使用九巴／龍運及步行的路線。
      可能這個行程需要其他交通工具，或站點距離太遠。</div>`;
    setStatus('沒有結果。');
    return;
  }
  const bus = first.legs.find(l => l.type === 'bus');
  const alt = bus ? plan(state.from, state.to, depart, { bannedPat: new Set([bus.pattern]) }) : null;
  const altOk = alt && alt.arrive > first.arrive ? alt : null;

  const sig = structureSig(first) + '#' + structureSig(altOk);
  if (auto && sig === state.lastSignature) {
    // Same journey, but the clock moved: refresh times and countdowns in place.
    refreshCard(state.cards[0], first);
    if (altOk && state.cards[1]) refreshCard(state.cards[1], altOk);
    state.lastDepart = depart;
    return;
  }

  container.innerHTML = '';
  state.cards = [renderResult(first, container, 0)];
  if (altOk) state.cards.push(renderResult(altOk, container, 1));
  drawPlan(first);
  state.lastSignature = sig;
  state.lastDepart = depart;

  const ms = Math.round(performance.now() - t0);
  setStatus(`以 ${hhmm(depart)} 出發計算，運算時間 ${ms} 毫秒` +
    (auto ? `；${hhmm(depart + 30)} 再更新。` : `；之後每 30 秒自動更新。`));
  if (!auto) card.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
}

/** Every 30 s, refresh the answer against the current clock. */
function autoRefresh() {
  if (document.hidden) return;
  if (!state.lastSignature) return;         // no query has been run yet
  run(true);
}

window.addEventListener('DOMContentLoaded', () => {
  // A stale cached HTML/JS pair must never leave the page without a map: wire the
  // controls defensively, then load the data no matter what happened above.
  try {
    fieldFrom = setupField('from');
    fieldTo = setupField('to');
    const geoBtn = document.getElementById('from-geo');
    if (geoBtn) geoBtn.addEventListener('click', locate);
    const go = document.getElementById('go');
    if (go) {
      go.addEventListener('click', () => run(false));
      go.disabled = true;
    }
    const nowBtn = document.getElementById('now-btn');
    if (nowBtn) nowBtn.addEventListener('click', () => run(false));
    setInterval(() => checkForUpdate(false), DATA_POLL_MS);
    setInterval(autoRefresh, AUTO_REFRESH_MS);
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) return;
      checkForUpdate(false);
      autoRefresh();
    });
  } catch (err) {
    console.error('UI wiring failed', err);
    setStatus(`介面初始化出錯：${err.message}（請按 Ctrl+Shift+R 重新載入）`);
  }
  // handle for debugging / tests (no effect on the UI)
  window.__khh = { state, plan, searchStops, searchPlaces, hhmm, stopName, routeShort,
                   headsignText, run, autoRefresh, stopsNear, checkForUpdate, boot };
  boot().catch(err => {
    console.error(err);
    setStatus(`載入失敗：${err.message}`);
  });
});
