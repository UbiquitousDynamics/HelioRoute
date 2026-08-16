"""Connectivity-aware external-download scheduler.

The scheduler estimates link quality and byte capacity from Wi-Fi, cellular,
RTT/loss, and measured goodput. It prioritizes data classes by importance and
freshness, selects a payload variant that fits the cycle budget, and protects
providers with HTTP 429 backoff and a circuit breaker. Injectable executors keep
the planning logic deterministic and testable offline.
"""

from __future__ import annotations

import math
import time

from service_models import FetchResult

# --------------------------- parametri globali ----------------------------- #
DEBUG = False          # True -> stampa quali metriche/parti dell'algoritmo sono usate
EWMA_GP = 0.4          # reattivita' della media del goodput
EWMA_RTT = 0.3
SAFETY = 0.6           # frazione del budget teorico che osiamo usare
MAX_PER_CYCLE = 4_000_000   # tetto byte per ciclo (non monopolizzare la rete)
MIN_GOODPUT = 800      # byte/s sotto cui consideriamo il link inutile
MIN_SCORE = 0.12       # punteggio link minimo per tentare
CB_FAILS = 3           # fallimenti consecutivi -> apri il circuit breaker
CB_BASE = 30.0         # cooldown base (s), cresce esponenzialmente
DEFAULT_GOODPUT = 300_000.0   # byte/s ipotizzati a punteggio pieno (bootstrap)


def _dbg(tag, msg):
    if DEBUG:
        print(f"[cs:{tag}] {msg}", flush=True)


# variabili orarie (nomi Open-Meteo) e loro sottoinsiemi per le varianti leggere
V_ALL = ["temperature_2m", "surface_pressure", "cloudcover", "precipitation",
         "shortwave_radiation", "windspeed_10m", "winddirection_10m", "windgusts_10m"]
V_CORE = ["windspeed_10m", "winddirection_10m", "windgusts_10m",
          "shortwave_radiation", "cloudcover", "precipitation"]
V_MIN = ["windspeed_10m", "winddirection_10m", "shortwave_radiation", "precipitation"]

# insiemi di modelli per le varianti (nomi Open-Meteo)
M3 = ["ecmwf_ifs025", "gfs_seamless", "icon_seamless"]
M2 = ["ecmwf_ifs025", "gfs_seamless"]
M1 = ["best_match"]

# quali campi del profilo aggiorna ciascun set (per non sovrascrivere con default)
_STORE_FIELDS = {
    "temperature_2m": "temp", "surface_pressure": "press", "cloudcover": "cloud",
    "precipitation": "precip", "shortwave_radiation": "ghi", "windgusts_10m": "gust",
}  # vento -> wind_u/wind_v gestito a parte


def store_fields_for(vars_):
    out = set()
    for v in vars_:
        if v in ("windspeed_10m", "winddirection_10m"):
            out.update(("wind_u", "wind_v"))
        elif v in _STORE_FIELDS:
            out.add(_STORE_FIELDS[v])
    return out


# --------------------------- registro classi dati -------------------------- #
# W = importanza (0..10) per WSC/auto solare; ttl = vita utile; min_int = intervallo
# minimo fra due download della stessa classe; critical = si tenta anche a banda bassa.
def _variants_weather(horizon, specs):
    out = []
    for label, rich, n_wp, models, vars_ in specs:
        out.append({"label": label, "kind": "weather", "richness": rich,
                    "horizon": horizon, "n_wp": n_wp, "models": models, "vars": vars_,
                    "store_fields": store_fields_for(vars_)})
    return out


CLASSES = [
    {"id": "weather_ahead", "name": "Weather ahead (PV + wind)", "W": 10.0,
     "ttl": 600, "min_int": 180, "critical": True, "prox": "ahead",
     "variants": _variants_weather(150, [
         ("ahead_rich", 1.0, 8, M3, V_ALL),
         ("ahead_std", 0.9, 6, M2, V_CORE),
         ("ahead_lite", 0.8, 4, M1, V_MIN)])},
    {"id": "weather_full", "name": "Whole-route weather (strategy)", "W": 7.0,
     "ttl": 1800, "min_int": 600, "critical": False, "prox": "whole",
     "variants": _variants_weather(0, [   # horizon 0 = intera rotta
         ("full_rich", 1.0, 30, M3, V_ALL),
         ("full_std", 0.9, 20, M2, V_CORE),
         ("full_lite", 0.8, 12, M1, V_CORE)])},
    {"id": "hazards", "name": "Roadworks/hazards ahead", "W": 6.0,
     "ttl": 2400, "min_int": 900, "critical": False, "prox": "ahead",
     "variants": [
         {"label": "haz_ahead", "kind": "osm", "richness": 1.0, "horizon": 80,
          "radius": 300, "default_bytes": 9000},
         {"label": "haz_min", "kind": "osm", "richness": 0.85, "horizon": 40,
          "radius": 300, "default_bytes": 4500}]},
    {"id": "dust", "name": "Dust/air quality ahead", "W": 5.0,
     "ttl": 1200, "min_int": 600, "critical": False, "prox": "ahead",
     "variants": [
         {"label": "dust_ahead", "kind": "dust", "richness": 1.0, "horizon": 150, "n_wp": 6},
         {"label": "dust_min", "kind": "dust", "richness": 0.85, "horizon": 80, "n_wp": 4}]},
    {"id": "traffic", "name": "Live incidents/traffic (TomTom)", "W": 6.0,
     "ttl": 300, "min_int": 120, "critical": False, "prox": "ahead", "needs_key": True,
     "variants": [
         {"label": "traffic_ahead", "kind": "tomtom", "richness": 1.0, "horizon": 80,
          "default_bytes": 2500}]},
]
# NB: "reroute" (OSRM) non e' periodico: si innesca a evento (fuori rotta / nuova
# meta) con priorita' alta; lo si gestisce fuori da questo ciclo.


def classes_for(ctx):
    return [c for c in CLASSES if not (c.get("needs_key") and not ctx.get("tomtom"))]


# --------------------------- normalizzazioni link -------------------------- #
def _clamp(x, a=0.0, b=1.0):
    return max(a, min(b, x))


def rssi_score(dbm):      # -90 dBm -> 0 ; -50 dBm -> 1
    return _clamp((dbm + 90) / 40.0)


def mbps_score(mbps):     # 0..~72 Mbps (2.4GHz decente)
    return _clamp(mbps / 72.0)


def rtt_score(ms):        # 100 ms -> 1 ; 800 ms -> 0
    return _clamp((800 - ms) / 700.0)


def loss_score(frac):     # 0% -> 1 ; 30% -> 0
    return _clamp(1 - frac / 0.30)


CELL_TYPE = {"none": 0.0, "2g": 0.15, "edge": 0.2, "3g": 0.4, "4g": 0.85, "lte": 0.85, "5g": 1.0}


def cell_type_score(t):
    return CELL_TYPE.get((t or "none").lower(), 0.0)


def rsrp_score(dbm):      # -120 -> 0 ; -80 -> 1
    return _clamp((dbm + 120) / 40.0)


def goodput_norm(bps):    # 500 kB/s -> 1
    return _clamp(bps / 500_000.0)


# --------------------------- monitor del link ------------------------------ #
class LinkMonitor:
    """Estimate device-to-hotspot-to-cellular link quality and capacity."""

    def __init__(self, now=None):
        self.wifi = None      # {"rssi", "mbps"}
        self.cell = None      # {"type", "dbm"}
        self.rtt = None       # ms (EWMA)
        self.loss = 0.0       # frazione (EWMA)
        self.goodput = 0.0    # byte/s (EWMA); 0 = sconosciuto
        self.consec_fail = 0
        self.cb_until = 0.0
        self.backoff_until = 0.0   # anti-429 (impostato dall'esterno o da note_429)
        self.last_seen = now or time.time()

    # --- ingestione campioni (dal dispositivo o dal companion sul telefono) ---
    def sample_wifi(self, rssi_dbm=None, link_mbps=None):
        self.wifi = {"rssi": rssi_dbm, "mbps": link_mbps}
        self.last_seen = time.time()
        _dbg("wifi", f"sample: rssi={rssi_dbm} dBm, bitrate={link_mbps} Mbps")

    def sample_cell(self, ctype=None, dbm=None):
        self.cell = {"type": ctype, "dbm": dbm}
        self.last_seen = time.time()
        _dbg("cell", f"sample: type={ctype}, rsrp={dbm} dBm")

    def sample_rtt(self, rtt_ms, loss=None):
        self.rtt = rtt_ms if self.rtt is None else (1 - EWMA_RTT) * self.rtt + EWMA_RTT * rtt_ms
        if loss is not None:
            self.loss = (1 - EWMA_RTT) * self.loss + EWMA_RTT * loss
        _dbg("rtt", f"sample: rtt(EWMA)={self.rtt:.0f} ms, loss(EWMA)={self.loss:.1%}")

    # --- esiti degli scaricamenti (misura il goodput vero) ---
    def note_fetch(self, n_bytes, seconds, ok, now=None):
        now = now or time.time()
        if ok and seconds > 0 and n_bytes > 0:
            bps = n_bytes / seconds
            self.goodput = bps if self.goodput <= 0 else (1 - EWMA_GP) * self.goodput + EWMA_GP * bps
            self.consec_fail = 0
            self.loss = (1 - EWMA_RTT) * self.loss  # decadimento
            _dbg("goodput", f"success: {n_bytes}B in {seconds:.2f}s -> goodput(EWMA)={self.goodput:.0f} B/s")
        elif not ok:
            self.consec_fail += 1
            self.loss = (1 - EWMA_RTT) * self.loss + EWMA_RTT * 1.0
            if self.consec_fail >= CB_FAILS:
                back = CB_BASE * (2 ** (self.consec_fail - CB_FAILS))
                self.cb_until = now + min(back, 300.0)
                _dbg("circuit-breaker", f"open: {self.consec_fail} consecutive failures, pause {min(back,300.0):.0f}s")
            else:
                _dbg("goodput", f"failed ({self.consec_fail}/{CB_FAILS} toward circuit breaker)")

    def note_429(self, retry_after, now=None):
        self.backoff_until = (now or time.time()) + max(int(retry_after), 60)
        _dbg("backoff-429", f"set a {max(int(retry_after),60)}s backoff")

    # --- sintesi ---
    def link_score(self):
        parts, weights, used = [], [], []
        if self.wifi and (self.wifi["rssi"] is not None or self.wifi["mbps"] is not None):
            r = rssi_score(self.wifi["rssi"]) if self.wifi["rssi"] is not None else 0.5
            m = mbps_score(self.wifi["mbps"]) if self.wifi["mbps"] is not None else 0.5
            parts.append(0.5 * r + 0.5 * m); weights.append(0.35)
            used.append(f"wifi={0.5*r+0.5*m:.2f}(w.35)")
        if self.goodput > 0:
            parts.append(goodput_norm(self.goodput)); weights.append(0.35)
            used.append(f"goodput={goodput_norm(self.goodput):.2f}(w.35)")
        if self.rtt is not None:
            parts.append(0.6 * rtt_score(self.rtt) + 0.4 * loss_score(self.loss)); weights.append(0.15)
            used.append(f"rtt/loss={0.6*rtt_score(self.rtt)+0.4*loss_score(self.loss):.2f}(w.15)")
        if self.cell:
            d = rsrp_score(self.cell["dbm"]) if self.cell.get("dbm") is not None else 0.5
            parts.append(0.6 * cell_type_score(self.cell.get("type")) + 0.4 * d); weights.append(0.15)
            used.append(f"cell={0.6*cell_type_score(self.cell.get('type'))+0.4*d:.2f}(w.15)")
        if not parts:
            _dbg("link-score", "no metrics available -> score 0.0")
            return 0.0
        score = sum(p * w for p, w in zip(parts, weights)) / sum(weights)
        _dbg("link-score", f"{' + '.join(used)} -> score={score:.2f}")
        return score

    def stability(self):
        return _clamp(1 - self.loss / 0.5, 0.3, 1.0)

    def is_up(self, now=None):
        now = now or time.time()
        if now < self.cb_until or now < self.backoff_until:
            _dbg("is_up", f"NO - paused (circuit breaker/429 backoff active for another {max(self.cb_until,self.backoff_until)-now:.0f}s)")
            return False
        # il telefono dichiara "nessuna cella": nessun uplink -> giu' (anche con WiFi ottimo)
        if self.cell and self.goodput <= 0 and cell_type_score(self.cell.get("type")) <= 0:
            _dbg("is_up", "NO - phone has no cellular uplink, even though hotspot Wi-Fi is good")
            return False
        if self.goodput > 0:
            up = self.goodput >= MIN_GOODPUT
            _dbg("is_up", f"{'YES' if up else 'NO'} - measured goodput {self.goodput:.0f} B/s (threshold {MIN_GOODPUT})")
            return up
        # nessuna misura: decidi dal punteggio; se non sappiamo nulla, concedi un
        # tentativo (probe) per scoprire se c'e' una finestra
        sc = self.link_score()
        if not (self.wifi or self.cell or self.rtt):
            _dbg("is_up", "YES - no known metrics, allowing a probe")
            return True
        up = sc >= MIN_SCORE
        _dbg("is_up", f"{'YES' if up else 'NO'} - link score {sc:.2f} (threshold {MIN_SCORE})")
        return up

    def budget_bytes(self, period_s, now=None):
        g = self.goodput if self.goodput > 0 else self.link_score() * DEFAULT_GOODPUT
        b = g * period_s * SAFETY * self.stability()
        b = int(_clamp(b, 0, MAX_PER_CYCLE))
        source = "measured goodput" if self.goodput > 0 else "link score (bootstrap)"
        _dbg("budget", f"{source}={g:.0f}B/s x window={period_s:.0f}s x safety={SAFETY} x stability={self.stability():.2f} -> {b}B")
        return b


# --------------------------- stima dimensioni ------------------------------ #
class SizeEstimator:
    """Predict variant sizes and learn from measured payload bytes per cell."""

    def __init__(self):
        self.bpc = {}     # label -> byte per "cella" (weather/dust)
        self.flat = {}    # label -> byte (osm/tomtom)
        self.def_bpc = 9.0
        self.base = 400.0

    def _hours(self, km, kmh):
        return int(_clamp(math.ceil(km / max(kmh, 1)) + 2, 2, 24 * 7))

    def cells(self, v, ctx):
        kmh = ctx.get("kmh", 90.0)
        remaining = max(1.0, ctx.get("route_total_km", 1.0) - ctx.get("car_km", 0.0))
        if v["kind"] == "weather":
            km = remaining if v["horizon"] == 0 else min(v["horizon"], remaining)
            hours = self._hours(km, kmh) if v["horizon"] == 0 else int(_clamp(math.ceil(km / max(kmh, 1)) + 2, 2, 24))
            series = len(v["models"]) * len(v["vars"])
            return v["n_wp"] * hours * (series + 1)
        if v["kind"] == "dust":
            km = min(v["horizon"], remaining)
            return v["n_wp"] * int(_clamp(math.ceil(km / max(kmh, 1)) + 2, 2, 24)) * 2
        return 0

    def predict(self, v, ctx):
        if v["kind"] in ("weather", "dust"):
            bpc = self.bpc.get(v["label"], self.def_bpc)
            return int(bpc * self.cells(v, ctx) + self.base)
        return int(self.flat.get(v["label"], v.get("default_bytes", 3000)))

    def learn(self, v, n_bytes, ctx):
        if v["kind"] in ("weather", "dust"):
            c = max(1, self.cells(v, ctx))
            bpc = (n_bytes - self.base) / c
            if bpc > 0:
                self.bpc[v["label"]] = bpc if v["label"] not in self.bpc else \
                    0.7 * self.bpc[v["label"]] + 0.3 * bpc
        else:
            self.flat[v["label"]] = n_bytes if v["label"] not in self.flat else \
                0.7 * self.flat[v["label"]] + 0.3 * n_bytes


# --------------------------- pianificatore --------------------------------- #
def urgency(age, c):
    if age is None:                       # mai scaricato: massima urgenza
        return 1.0
    if age < c["min_int"]:                # troppo fresco: non riscaricare
        return 0.0
    span = max(1.0, c["ttl"] - c["min_int"])
    return _clamp((age - c["min_int"]) / span)


def plan(now, ages, monitor, ctx, est):
    """Ritorna la lista ordinata di download da fare in questo ciclo."""
    if not monitor.is_up(now):
        _dbg("plan", "link unusable -> no downloads, map remains local")
        return []                          # offline: nessun download (mappa locale continua)
    budget = monitor.budget_bytes(ctx.get("period", 20.0), now)
    cand = []
    for c in classes_for(ctx):
        u = urgency(ages.get(c["id"]), c)
        if u <= 0:
            continue
        base = c["W"] * u                  # priorita' base (prox=1 per le classi 'ahead')
        cand.append((base, c, u))
    cand.sort(key=lambda x: x[0], reverse=True)
    _dbg("plan", f"budget={budget}B; candidates by priority: " +
        ", ".join(f"{c['id']}(W{c['W']}xurg{u:.2f}={b:.1f})" for b, c, u in cand))

    out, rem = [], budget
    for base, c, u in cand:
        chosen = None
        for v in c["variants"]:            # dalla piu' ricca alla piu' leggera
            b = est.predict(v, ctx)
            if b <= rem:
                chosen = (v, b); break
        if chosen is None:
            if c["critical"]:              # critica: forza la piu' leggera anche se sfora
                v = c["variants"][-1]; b_forced = est.predict(v, ctx)
                _dbg("plan", f"{c['id']}: no variant fits but class is CRITICAL -> force '{v['label']}' ({b_forced}B, over budget)")
                chosen = (v, b_forced)
            else:
                _dbg("plan", f"{c['id']}: no variant fits the remaining budget ({rem}B) -> deferred")
                continue
        else:
            _dbg("plan", f"{c['id']}: selected '{chosen[0]['label']}' ({chosen[1]}B); remaining budget: {rem-chosen[1]}B")
        v, b = chosen
        rem = max(0, rem - b)
        out.append({"class": c["id"], "name": c["name"], "variant": v["label"],
                    "bytes": b, "value": round(base * v["richness"], 2),
                    "age": ages.get(c["id"]), "spec": v})
    return out


# --------------------------- ciclo adattivo -------------------------------- #
def adaptive_cycle(monitor, ages, ctx, executors, est, now=None):
    """Pianifica ed esegue i download scelti; aggiorna monitor ed eta'.
    executors: dict class_id -> callable(spec, ctx) -> (bytes, seconds, ok, retry_after_or_None)
    ages: dict class_id -> timestamp ultimo download (aggiornato qui)."""
    now = now or time.time()
    jobs = plan(now, ages, monitor, ctx, est)
    report = []
    for j in jobs:
        ex = executors.get(j["class"])
        if not ex:
            _dbg("cycle", f"{j['class']}: no registered executor -> skipping")
            continue
        result = FetchResult.coerce(ex(j["spec"], ctx))
        b = result.bytes_downloaded
        sec = result.seconds
        ok = result.ok
        retry = result.retry_after
        _dbg("cycle", f"{j['class']}/{j['variant']}: {'ok' if ok else 'FAILED'} - {b}B in {sec:.2f}s" +
            (f" (retry-after {retry}s)" if retry else ""))
        monitor.note_fetch(b, sec, ok, now=now)
        if retry:
            monitor.note_429(retry, now=now)
        if ok:
            est.learn(j["spec"], b, ctx)
            ages[j["class"]] = now
        report.append({**j, "downloaded": b, "seconds": round(sec, 2), "ok": ok,
                       "error": result.error})
        if not ok:
            _dbg("cycle", f"stop cycle at first failure ({j['class']}); remaining jobs not attempted")
            break   # il link si e' rivelato inutilizzabile: non insistere in questo ciclo
    return report


# --------------------------- cache dei campi (merge) ----------------------- #
class FieldStore:
    """Store sparse route fields and merge partial downloads with local defaults."""

    def __init__(self, route_geom, kmh=90.0):
        import profile_engine as pe
        import weather_providers as wx
        self.pe, self.wx, self.kmh = pe, wx, kmh
        self.lats, self.lons, self.cum, _ = pe.resample(route_geom, 1.0)
        self.wp = wx._pick_waypoints(self.cum)
        self.wl = [self.lats[i] for i in self.wp]
        self.wo = [self.lons[i] for i in self.wp]
        self.wc = [self.cum[i] for i in self.wp]
        self.fields = {}        # campo -> lista allineata a wp (None se ignoto)
        self.ts = {}            # campo -> timestamp ultimo aggiornamento
        self._elev_done = False

    def _ensure(self, key):
        if key not in self.fields:
            self.fields[key] = [None] * len(self.wp)

    def car_wp_window(self, car_km, horizon, max_points=None):
        idx = [k for k, c in enumerate(self.wc)
               if horizon <= 0 or (car_km - 10 <= c <= car_km + horizon)]
        if not idx:
            idx = [min(range(len(self.wc)), key=lambda k: abs(self.wc[k] - car_km))]
        if max_points and len(idx) > max_points:
            if max_points == 1:
                idx = [idx[0]]
            else:
                idx = [idx[round(i * (len(idx) - 1) / (max_points - 1))]
                       for i in range(max_points)]
        return idx

    def ensure_elevation(self):
        if self._elev_done:
            return
        elev = self.wx._fetch_elevation(self.wl, self.wo)   # 1 sola volta (in cache)
        self._ensure("elev")
        for k, e in enumerate(elev):
            self.fields["elev"][k] = e
        self.ts["elev"] = time.time()
        self._elev_done = True

    def update_weather(self, sel_idx, models, vars_, store_fields, car_km=0.0):
        wl = [self.wl[k] for k in sel_idx]
        wo = [self.wo[k] for k in sel_idx]
        wc = [self.wc[k] for k in sel_idx]
        pts = self.wx.fetch_weather_points(
            wl, wo, wc, self.kmh, models, vars_, origin_km=car_km)
        for f in store_fields:
            self._ensure(f)
        for j, k in enumerate(sel_idx):
            for f in store_fields:
                self.fields[f][k] = pts[j].get(f)
        now = time.time()
        for f in store_fields:
            self.ts[f] = now

    def update_dust(self, sel_idx, car_km=0.0):
        wl = [self.wl[k] for k in sel_idx]
        wo = [self.wo[k] for k in sel_idx]
        wc = [self.wc[k] for k in sel_idx]
        vals = self.wx.fetch_dust_points(wl, wo, wc, self.kmh, origin_km=car_km)
        self._ensure("dust")
        for j, k in enumerate(sel_idx):
            self.fields["dust"][k] = vals[j]
        self.ts["dust"] = time.time()

    def provider(self, lats, lons, cum):
        out = self.pe._synth_fields(lats, lons, cum)     # base plausibile
        n = len(cum)
        for f, arr in self.fields.items():
            xs, ys = [], []
            for k, val in enumerate(arr):
                if val is not None:
                    xs.append(self.wc[k]); ys.append(val)
            if len(xs) >= 2:
                out[f] = [self.wx._interp1d(xs, ys, cum[i]) for i in range(n)]
            elif len(xs) == 1:
                out[f] = [ys[0]] * n
        return out


# --------------------------- sonda WiFi (Linux) ---------------------------- #
def probe_linux_wifi(iface="wlan0"):
    """Best-effort Linux Wi-Fi RSSI and negotiated-rate probe."""
    out = {}
    try:
        with open("/proc/net/wireless") as f:
            for line in f:
                if line.strip().startswith(iface):
                    cols = line.split()
                    # colonna 'level' e' l'RSSI in dBm
                    out["rssi"] = float(cols[3].rstrip("."))
                    break
    except Exception:
        pass
    try:
        import subprocess
        txt = subprocess.run(["iw", "dev", iface, "link"], capture_output=True,
                             text=True, timeout=2).stdout
        for line in txt.splitlines():
            s = line.strip()
            if s.startswith("signal:"):
                out["rssi"] = float(s.split()[1])
            elif s.startswith("tx bitrate:"):
                out["mbps"] = float(s.split()[2])
    except Exception:
        pass
    return out


def human_bytes(n):
    if n < 1024:
        return f"{n:.0f} B"
    if n < 1024 * 1024:
        return f"{n/1024:.1f} KB"
    return f"{n/1024/1024:.1f} MB"


# --------------------------- simulazione (demo) ---------------------------- #
def _sim_executor_factory(monitor):
    """Executor finto: 'trasferisce' i byte previsti al goodput simulato del link."""
    def make(bps_sim, fail=False):
        def ex(spec, ctx):
            b = ctx["_est"].predict(spec, ctx)
            if fail or bps_sim <= 0:
                return (0, ctx.get("period", 20.0), False, None)
            sec = b / bps_sim
            return (b, sec, True, None)
        return ex
    return make


def _run_demo():
    est = SizeEstimator()
    # tratta tipo WSC: 2600 km, auto a 300 km dalla partenza
    ctx = {"route_total_km": 2600.0, "car_km": 300.0, "kmh": 90.0, "period": 20.0,
           "tomtom": True, "_est": est}
    scenari = [
        ("LAN buona + 5G", dict(wifi=(-52, 300), cell=("5g", -80), rtt=(45, 0.0)), 600_000),
        ("4G pieno", dict(wifi=(-60, 72), cell=("4g", -95), rtt=(90, 0.01)), 120_000),
        ("Weak Wi-Fi (hot vehicle) + 3G", dict(wifi=(-84, 8), cell=("3g", -108), rtt=(300, 0.05)), 12_000),
        ("Bordo copertura (EDGE)", dict(wifi=(-70, 40), cell=("edge", -112), rtt=(700, 0.12)), 2_500),
        ("Desert: no cellular service", dict(wifi=(-58, 65), cell=("none", None), rtt=(None, None)), 0),
    ]
    for titolo, s, bps in scenari:
        mon = LinkMonitor()
        if s.get("wifi"):
            mon.sample_wifi(*s["wifi"])
        if s.get("cell"):
            mon.sample_cell(*s["cell"])
        if s.get("rtt") and s["rtt"][0] is not None:
            mon.sample_rtt(*s["rtt"])
        if bps > 0:            # come se avessimo gia' misurato il goodput
            mon.note_fetch(bps, 1.0, True)
        ages = {}              # nulla scaricato di recente -> tutto e' scaduto
        ex = {c["id"]: _sim_executor_factory(mon)(bps) for c in CLASSES}
        rep = adaptive_cycle(mon, ages, ctx, ex, est)
        budget = mon.budget_bytes(ctx["period"])
        print(f"\n=== {titolo} ===")
        print(f"  link {mon.link_score():.2f} · up={mon.is_up()} · "
              f"goodput {human_bytes(mon.goodput)}/s · budget/ciclo {human_bytes(budget)}")
        if not rep:
            print("  -> no download; map (vehicle + events) updated locally")
        got = [r for r in rep if r["ok"]]
        for r in rep:
            result = "ok" if r["ok"] else "FAILED -> stop cycle"
            print(f"  -> {r['name']:<32} [{r['variant']:<13}] "
                  f"{human_bytes(r['downloaded']):>9}  {r['seconds']:5.1f}s  {result}")
        skipped = [c["name"] for c in classes_for(ctx)
                   if c["id"] not in {r["class"] for r in rep}]
        if got and skipped:
            print(f"     deferred (insufficient bandwidth): {', '.join(skipped)}")
        if got:
            print(f"     total: {human_bytes(sum(r['downloaded'] for r in got))}")


if __name__ == "__main__":
    DEBUG = True   # eseguito come script: mostra tutte le stampe di debug dell'algoritmo
    print("ADAPTIVE SCHEDULER SIMULATION — behavior under changing link conditions")
    _run_demo()
