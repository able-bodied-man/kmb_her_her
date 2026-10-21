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
  stops: [], routes: [], headsigns: [], cal: null, calIndex: {},
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

  const [tripsBuf, stBuf, walkBuf, patBuf, spBuf] = await Promise.all([
    getBuffer('trips.bin'), getBuffer('stoptimes.bin'), getBuffer('walk.bin'),
    getBuffer('patterns.bin'), getBuffer('stoppatterns.bin'),
  ]);
  // route alignments are optional: without them the map falls back to straight lines
  state.shapes = await loadShapes();

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
let refreshing = false;

function rememberQuery() {
  state.lastQuery = { from: state.from, to: state.to, depart: timeFromInput() };
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
    if (hadQuery && state.from && state.to) {
      document.getElementById('time-input').value = hhmm(hadQuery.depart);
      run();
    }
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
function activeServices(date) {
  const bits = 1 << ((date.getDay() + 6) % 7);
  const ymd = date.getFullYear() * 10000 + (date.getMonth() + 1) * 100 + date.getDate();
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
  for (let k = ans; k < to && scanned < 80; k++, scanned++) {
    const ti = list[k];
    if (start[ti] > after + 6 * 3600) break;
    if (!active.has(service[ti])) continue;
    const row = offset[ti] + pos;
    if (!(state.st.packed[row] & (1 << 13))) continue;          // no pick-up here
    const dep = start[ti] + state.st.arr[row];
    if (dep < after) continue;
    if (!best || dep < best.dep) best = { trip: ti, dep };
  }
  return best;
}

function plan(origin, dest, departSec, options = {}) {
  const nStops = state.stops.length;
  const active = activeServices(options.date || new Date());
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
      bestBoard[pi] = dep; bestPos[pi] = pos;

      const pStopBase = state.pat.stopOff[pi];
      const nPatStops = state.pat.stopOff[pi + 1] - pStopBase;
      const rowBase = state.trips.offset[trip];
      for (let p = pos + 1; p < nPatStops; p++) {
        const sidx = state.pat.stops[pStopBase + p];
        const row = rowBase + p;
        if (!(state.st.packed[row] & (1 << 14))) continue;       // no set-down here
        const arr = state.trips.start[trip] + state.st.arr[row];
        if (arr < tau[sidx] - 1e-6) {
          tau[sidx] = arr;
          parent[sidx] = { type: 'bus', from: s, trip, board: pos, alight: p, boardTime: dep };
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
                  board: p.board, alight: p.alight });
      cur = p.from;
    }
  }
  legs.reverse();

  // walk from the last stop to the actual destination point (added before the
  // merge so it collapses with an alighting walk into one leg)
  if (best.walk > 0) {
    legs.push({ type: 'walk', metres: best.walk, toDest: true });
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
      leg.start = state.trips.start[ti] + state.st.arr[state.trips.offset[ti] + leg.board];
      leg.end = state.trips.start[ti] + state.st.arr[state.trips.offset[ti] + leg.alight];
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
      const from = leg.from !== undefined ? state.stops[leg.from] : null;
      const to = leg.to !== undefined ? state.stops[leg.to] : null;
      if (from && to) {
        addLayer(L.polyline([[from[1], from[2]], [to[1], to[2]]],
          { color: '#6b6b70', weight: 3, dashArray: '5,6', opacity: .9 }));
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

function timeFromInput() {
  const v = document.getElementById('time-input').value || hhmmNow();
  const [h, m] = v.split(':').map(Number);
  return h * 3600 + m * 60;
}
function hhmmNow() {
  const d = new Date();
  return `${String(d.getHours()).padStart(2, '0')}:${String(d.getMinutes()).padStart(2, '0')}`;
}

function renderResult(plan, container, index) {
  const card = document.createElement('article');
  card.className = 'card' + (index === 0 ? ' active' : '');

  const transfers = plan.legs.filter(l => l.type === 'bus').length - 1;
  const walkMetres = plan.legs.filter(l => l.type === 'walk').reduce((a, l) => a + l.metres, 0);
  const head = document.createElement('div');
  head.className = 'summary';
  head.innerHTML = `
    <span class="total">${mins(plan.arrive - plan.depart)} 分鐘</span>
    <span class="times">${hhmm(plan.depart)} 出發 → ${hhmm(plan.arrive)} 抵達</span>
    <span class="times">${transfers > 0 ? `轉乘 ${transfers} 次` : '直達'}${walkMetres >= 50 ? ` · 步行 ${walkMetres >= 1000 ? (walkMetres / 1000).toFixed(1) + ' 公里' : Math.round(walkMetres) + ' 米'}` : ''}</span>`;
  card.appendChild(head);

  const legs = document.createElement('div');
  legs.className = 'legs';
  let prevEnd = plan.depart;
  for (const leg of plan.legs) {
    if (leg.type === 'walk' && leg.metres < 40) { prevEnd = leg.end; continue; }   // same-kerb shuffle
    const wait = leg.start - prevEnd;
    if (wait >= 180) {
      const w = document.createElement('div');
      w.className = 'leg wait';
      w.innerHTML = `<span class="when">${hhmm(prevEnd)}</span><span class="what">等候 ${mins(wait)} 分鐘</span>`;
      legs.appendChild(w);
    }
    const row = document.createElement('div');
    row.className = 'leg ' + leg.type;
    if (leg.type === 'walk') {
      row.innerHTML = `
        <span class="when">${hhmm(leg.start)} – ${hhmm(leg.end)}</span>
        <span class="what">步行 ${mins(leg.seconds)} 分鐘（約 ${Math.round(leg.metres)} 米）
          ${leg.toDest ? '至目的地' : (leg.from !== undefined ? `至 ${stopName(leg.to)}` : `至 ${stopName(leg.to)}`)}</span>`;
    } else {
      const badge = `<span class="route-badge">${routeShort(leg.routeIdx)}</span>`;
      row.innerHTML = `
        <span class="when">${hhmm(leg.start)} – ${hhmm(leg.end)}</span>
        <span class="what">${badge}${headsignText(leg.headsign)}
          <span class="stops"><br>${stopName(leg.boardStop)} 上車 · ${stopName(leg.alightStop)} 落車 ·
          ${leg.stopCount} 個站 · ${mins(leg.seconds)} 分鐘</span></span>`;
    }
    legs.appendChild(row);
    prevEnd = leg.end;
  }
  card.appendChild(legs);
  card.addEventListener('click', () => {
    document.querySelectorAll('.card').forEach(c => c.classList.remove('active'));
    card.classList.add('active');
    drawPlan(plan);
  });
  container.appendChild(card);
  return card;
}

function run() {
  const container = document.getElementById('results');
  container.innerHTML = '';
  const depart = timeFromInput();
  rememberQuery();

  // a coordinate origin/destination only works if a stop is walkable from it
  for (const [sel, name] of [[state.from, '起點'], [state.to, '終點']]) {
    if (sel && sel.stop === undefined && stopsNear(sel.lat, sel.lon, ORIGIN_WALK_M).length === 0) {
      container.innerHTML = `<div class="empty">${name} ${ORIGIN_WALK_M} 米範圍內沒有九巴／龍運車站，
        這個位置無法只用九巴／龍運及步行到達。</div>`;
      setStatus('沒有結果。');
      return;
    }
  }

  const t0 = performance.now();
  const first = plan(state.from, state.to, depart);
  if (!first) {
    container.innerHTML = `<div class="empty">找不到只使用九巴／龍運及步行的路線。
      可能這個行程需要其他交通工具，或站點距離太遠。</div>`;
    setStatus('沒有結果。');
    return;
  }
  const card = renderResult(first, container, 0);
  drawPlan(first);

  // one alternative: forbid the first bus pattern
  const bus = first.legs.find(l => l.type === 'bus');
  if (bus) {
    const alt = plan(state.from, state.to, depart, { bannedPat: new Set([bus.pattern]) });
    if (alt && alt.arrive > first.arrive) renderResult(alt, container, 1);
  }
  const ms = Math.round(performance.now() - t0);
  setStatus(`以 ${hhmm(depart)} 出發計算，運算時間 ${ms} 毫秒。`);
  card.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
}

window.addEventListener('DOMContentLoaded', () => {
  fieldFrom = setupField('from');
  fieldTo = setupField('to');
  document.getElementById('now-btn').addEventListener('click', () => {
    document.getElementById('time-input').value = hhmmNow();
  });
  const geoBtn = document.getElementById('from-geo');
  if (geoBtn) geoBtn.addEventListener('click', locate);
  document.getElementById('time-input').value = hhmmNow();
  document.getElementById('go').addEventListener('click', run);
  document.getElementById('go').disabled = true;
  const refreshBtn = document.getElementById('data-refresh');
  if (refreshBtn) refreshBtn.addEventListener('click', () => checkForUpdate(true));
  setInterval(() => checkForUpdate(false), DATA_POLL_MS);
  document.addEventListener('visibilitychange', () => { if (!document.hidden) checkForUpdate(false); });
  // handle for debugging / tests (no effect on the UI)
  window.__khh = { state, plan, searchStops, searchPlaces, hhmm, stopName, routeShort,
                   headsignText, run, stopsNear, checkForUpdate, boot };
  boot().catch(err => {
    console.error(err);
    setStatus(`載入失敗：${err.message}`);
  });
});
