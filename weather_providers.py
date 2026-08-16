"""Fetch and blend route weather, elevation, and air-quality fields.

Requests use sparse waypoints and only the required forecast-hour window.
Elevation is cached, ECMWF/GFS/ICON values are blended, and HTTP 429 responses
surface as RateLimitError with their Retry-After value.
"""

from __future__ import annotations

import json
import math
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timedelta, timezone

import profile_engine as pe

FORECAST_URL = "https://api.open-meteo.com/v1/forecast"
ELEVATION_URL = "https://api.open-meteo.com/v1/elevation"
AIRQ_URL = "https://air-quality-api.open-meteo.com/v1/air-quality"

# meno modelli = meno costo (il salto di accuratezza maggiore e' da 1 a 2-3 modelli)
DEFAULT_MODELS = ["ecmwf_ifs025", "gfs_seamless", "icon_seamless"]
HOURLY_VARS = ["temperature_2m", "surface_pressure", "cloudcover", "precipitation",
               "shortwave_radiation", "windspeed_10m", "winddirection_10m",
               "windgusts_10m"]

WAYPOINT_STEP_KM = 40.0     # piu' rado -> meno punti -> meno costo
MAX_WAYPOINTS = 30
MIN_WAYPOINTS = 6           # anche su rotte corte campiona i punti intermedi
FORECAST_CHUNK = 30         # <= MAX_WAYPOINTS -> tipicamente 1 sola chiamata
ELEV_CHUNK = 100
MAX_HORIZON_DAYS = 7
USER_AGENT = "wsc-solar-weather/1.1 (route weather aggregator)"

SOURCE_LABELS = [
    "Open-Meteo Forecast (blended ECMWF, GFS, ICON)",
    "Open-Meteo Elevation (cached DEM)",
    "Open-Meteo Air Quality (dust)",
]

_ELEV_CACHE = {}
NET = {"bytes": 0}   # byte scaricati (aggiornato da _get_json), per lo scheduler adattivo


class RateLimitError(Exception):
    """HTTP 429: troppe richieste. .retry_after = secondi consigliati d'attesa."""
    def __init__(self, retry_after=60):
        super().__init__("HTTP 429 Too Many Requests")
        self.retry_after = int(retry_after)


# ------------------------------ HTTP (mockabile) --------------------------- #
def _get_json(url, timeout=25):
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            raw = r.read()
            NET["bytes"] += len(raw)
            return json.loads(raw.decode("utf-8"))
    except urllib.error.HTTPError as e:
        if e.code == 429:
            ra = e.headers.get("Retry-After") if e.headers else None
            try:
                ra = int(ra)
            except (TypeError, ValueError):
                ra = 60
            raise RateLimitError(ra)
        raise


def _as_list(data):
    return data if isinstance(data, list) else [data]


def _iso_to_epoch(s):
    try:
        return datetime.fromisoformat(s).replace(tzinfo=timezone.utc).timestamp()
    except ValueError:
        return datetime.strptime(s, "%Y-%m-%dT%H:%M").replace(tzinfo=timezone.utc).timestamp()


def _nearest_hour_index(times, target_epoch):
    best_i, best_d = 0, 1e18
    for i, t in enumerate(times):
        d = abs(_iso_to_epoch(t) - target_epoch)
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def _interp1d(xs, ys, x):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x: lo = mid
        else: hi = mid
    span = xs[hi] - xs[lo]
    f = 0.0 if span == 0 else (x - xs[lo]) / span
    return ys[lo] + (ys[hi] - ys[lo]) * f


def _pick_waypoints(cum, step_km=WAYPOINT_STEP_KM, max_pts=MAX_WAYPOINTS):
    n = len(cum)
    if n <= 2:
        return list(range(n))
    total = cum[-1]
    k = min(max_pts, max(MIN_WAYPOINTS, int(total / step_km) + 1))
    k = min(k, n)                                         # non piu' dei punti disponibili
    if k < 2:
        return list(range(n))
    targets = [total * i / (k - 1) for i in range(k)]     # equidistanti, estremi inclusi
    idx = []
    j = 0
    for t in targets:
        while j + 1 < n and cum[j + 1] < t:
            j += 1
        if j + 1 < n and abs(cum[j + 1] - t) < abs(cum[j] - t):
            idx.append(j + 1)
        else:
            idx.append(j)
    idx = sorted(set(idx))
    if idx[0] != 0:
        idx.insert(0, 0)
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return idx


def _chunks(seq, size):
    for i in range(0, len(seq), size):
        yield seq[i:i + size]


# ------------------------------ elevation (cache) -------------------------- #
def _fetch_elevation(wp_lats, wp_lons):
    key = tuple((round(a, 4), round(o, 4)) for a, o in zip(wp_lats, wp_lons))
    if key in _ELEV_CACHE:
        return _ELEV_CACHE[key]
    out = []
    for cl, co in zip(_chunks(wp_lats, ELEV_CHUNK), _chunks(wp_lons, ELEV_CHUNK)):
        q = urllib.parse.urlencode({"latitude": ",".join(f"{x:.5f}" for x in cl),
                                    "longitude": ",".join(f"{x:.5f}" for x in co)})
        data = _get_json(f"{ELEVATION_URL}?{q}")
        elev = data.get("elevation")
        if not elev or len(elev) != len(cl):
            raise RuntimeError("invalid elevation response")
        out.extend(elev)
    _ELEV_CACHE[key] = out
    if len(_ELEV_CACHE) > 8:
        _ELEV_CACHE.pop(next(iter(_ELEV_CACHE)))
    return out


# ------------------------------ forecast ----------------------------------- #
def _blend_point(hourly, hidx, models):
    def collect(var):
        vals = []
        for m in models:
            arr = hourly.get(f"{var}_{m}")
            if arr and hidx < len(arr) and arr[hidx] is not None:
                vals.append(arr[hidx])
        if not vals:
            arr = hourly.get(var)
            if arr and hidx < len(arr) and arr[hidx] is not None:
                vals.append(arr[hidx])
        return vals

    def avg(a, d=0.0):
        return sum(a) / len(a) if a else d

    spd = collect("windspeed_10m"); wdir = collect("winddirection_10m")
    us, vs = [], []
    for s, d in zip(spd, wdir):
        u, v = pe.dir_speed_to_uv(s, d); us.append(u); vs.append(v)
    return {"temp": avg(collect("temperature_2m"), 20.0),
            "press": avg(collect("surface_pressure"), 1013.0),
            "cloud": avg(collect("cloudcover"), 0.0),
            "precip": avg(collect("precipitation"), 0.0),
            "ghi": max(0.0, avg(collect("shortwave_radiation"), 0.0)),
            "gust": avg(collect("windgusts_10m"), 0.0),
            "wind_u": avg(us, 0.0), "wind_v": avg(vs, 0.0)}


def _fetch_forecast(wp_lats, wp_lons, wp_eta_s, models, sh, eh, now_epoch, hourly_vars=None):
    hourly_vars = hourly_vars or HOURLY_VARS
    pts = []
    for cl, co, ce in zip(_chunks(wp_lats, FORECAST_CHUNK),
                          _chunks(wp_lons, FORECAST_CHUNK),
                          _chunks(wp_eta_s, FORECAST_CHUNK)):
        params = {"latitude": ",".join(f"{x:.5f}" for x in cl),
                  "longitude": ",".join(f"{x:.5f}" for x in co),
                  "hourly": ",".join(hourly_vars), "models": ",".join(models),
                  "windspeed_unit": "ms", "timezone": "UTC",
                  "start_hour": sh, "end_hour": eh, "cell_selection": "nearest"}
        data = _get_json(f"{FORECAST_URL}?{urllib.parse.urlencode(params)}")
        if isinstance(data, dict) and data.get("error"):
            raise RuntimeError(f"open-meteo: {data.get('reason')}")
        locs = _as_list(data)
        if len(locs) != len(cl):
            raise RuntimeError("unexpected number of forecast locations")
        for loc, eta in zip(locs, ce):
            hourly = loc.get("hourly") or {}
            times = hourly.get("time") or []
            if not times:
                raise RuntimeError("hourly forecast is missing")
            pts.append(_blend_point(hourly, _nearest_hour_index(times, now_epoch + eta), models))
    return pts


def _fetch_dust(wp_lats, wp_lons, wp_eta_s, sh, eh, now_epoch, strict=False):
    out = []
    try:
        for cl, co, ce in zip(_chunks(wp_lats, FORECAST_CHUNK),
                              _chunks(wp_lons, FORECAST_CHUNK),
                              _chunks(wp_eta_s, FORECAST_CHUNK)):
            params = {"latitude": ",".join(f"{x:.5f}" for x in cl),
                      "longitude": ",".join(f"{x:.5f}" for x in co),
                      "hourly": "dust", "timezone": "UTC",
                      "start_hour": sh, "end_hour": eh}
            data = _get_json(f"{AIRQ_URL}?{urllib.parse.urlencode(params)}")
            for loc, eta in zip(_as_list(data), ce):
                hourly = loc.get("hourly") or {}
                times = hourly.get("time") or []; arr = hourly.get("dust") or []
                if times and arr:
                    hidx = _nearest_hour_index(times, now_epoch + eta)
                    out.append(float(arr[hidx]) if hidx < len(arr) and arr[hidx] is not None else 0.0)
                else:
                    out.append(0.0)
    except RateLimitError:
        raise
    except Exception as exc:
        if strict:
            raise RuntimeError(f"dust data is unavailable: {exc}") from exc
        return [0.0] * len(wp_lats)
    return out if len(out) == len(wp_lats) else [0.0] * len(wp_lats)


# ------------------------------ API ---------------------------------------- #
def _hours_window(cum, nominal_kmh, origin_km=0.0):
    now = datetime.now(timezone.utc)
    eta = [(max(0.0, c - origin_km) / max(nominal_kmh, 1.0)) * 3600.0 for c in cum]
    start = now.replace(minute=0, second=0, microsecond=0)
    hn = int(math.ceil((max(eta) if eta else 0) / 3600.0)) + 1
    end = start + timedelta(hours=min(hn, MAX_HORIZON_DAYS * 24))
    return (eta, now.timestamp(), start.strftime("%Y-%m-%dT%H:00"), end.strftime("%Y-%m-%dT%H:00"))


def fetch_weather_points(lats, lons, cum, nominal_kmh=90.0, models=None,
                         hourly_vars=None, origin_km=0.0):
    """Fetch forecast fields at exactly the supplied points without resampling."""
    models = models or DEFAULT_MODELS
    eta, now_epoch, sh, eh = _hours_window(cum, nominal_kmh, origin_km)
    return _fetch_forecast(list(lats), list(lons), eta, models, sh, eh, now_epoch,
                           hourly_vars=hourly_vars)


def fetch_dust_points(lats, lons, cum, nominal_kmh=90.0, origin_km=0.0, strict=True):
    """Fetch air-quality dust values at the supplied points."""
    eta, now_epoch, sh, eh = _hours_window(cum, nominal_kmh, origin_km)
    return _fetch_dust(list(lats), list(lons), eta, sh, eh, now_epoch, strict=strict)


def fetch_fields(lats, lons, cum, nominal_kmh=90.0, models=None, origin_km=0.0):
    models = models or DEFAULT_MODELS
    n = len(cum)
    idx = _pick_waypoints(cum)
    wp_lats = [lats[i] for i in idx]; wp_lons = [lons[i] for i in idx]
    wp_cum = [cum[i] for i in idx]
    wp_eta_s, now_epoch, sh, eh = _hours_window(wp_cum, nominal_kmh, origin_km)

    wp_elev = _fetch_elevation(wp_lats, wp_lons)          # in cache dopo la 1a volta

    try:
        wp_fc = _fetch_forecast(wp_lats, wp_lons, wp_eta_s, models, sh, eh, now_epoch)
    except RateLimitError:
        raise                                             # NON ritentare su 429
    except Exception as exc:
        try:                                              # fallback: modello singolo
            wp_fc = _fetch_forecast(wp_lats, wp_lons, wp_eta_s, ["best_match"], sh, eh, now_epoch)
        except RateLimitError:
            raise
        except Exception as exc2:
            raise RuntimeError(f"weather is unavailable: {exc2}") from exc

    wp_dust = _fetch_dust(wp_lats, wp_lons, wp_eta_s, sh, eh, now_epoch)

    wp = {"elev": wp_elev, "dust": wp_dust,
          "wind_u": [p["wind_u"] for p in wp_fc], "wind_v": [p["wind_v"] for p in wp_fc],
          "temp": [p["temp"] for p in wp_fc], "press": [p["press"] for p in wp_fc],
          "cloud": [p["cloud"] for p in wp_fc], "precip": [p["precip"] for p in wp_fc],
          "ghi": [p["ghi"] for p in wp_fc], "gust": [p["gust"] for p in wp_fc]}

    return {k: [_interp1d(wp_cum, ys, cum[i]) for i in range(n)] for k, ys in wp.items()}
