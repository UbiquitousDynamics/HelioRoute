"""Deterministic fake services and ground truth for unit tests and KPIs.

Context managers replace Open-Meteo, Overpass, and OSRM boundaries so blending,
interpolation, alerting, latency, and failure behavior can be tested offline.
"""

from __future__ import annotations

import time
import urllib.parse as up
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone

import map_service as ms
import road_hazards as rh
import weather_providers as wx
from service_state import new_service_state


def _times(h=8):
    t0 = datetime.now(timezone.utc).replace(minute=0, second=0, microsecond=0)
    return [(t0 + timedelta(hours=i)).strftime("%Y-%m-%dT%H:%M") for i in range(h)]


# ground-truth come funzione di (lat,lon): valori "veri" attesi
GT = {
    "temperature_2m": lambda lat, lon: 15.0 + 0.5 * lat,
    "surface_pressure": lambda lat, lon: 1013.0,
    "cloudcover": lambda lat, lon: 40.0,
    "precipitation": lambda lat, lon: 0.0,
    "shortwave_radiation": lambda lat, lon: 700.0,
    "windspeed_10m": lambda lat, lon: 6.0 + 0.2 * (lat - 25.0),
    "winddirection_10m": lambda lat, lon: 260.0,
    "windgusts_10m": lambda lat, lon: 9.0,
}
# bias per modello: SOMMA ZERO (blend esatto) ma NESSUNO nullo
# (cosi' il blend batte il miglior singolo modello)
BIAS = {
    "temperature_2m": {"ecmwf_ifs025": 1.2, "gfs_seamless": -0.9, "icon_seamless": -0.3, "best_match": 0.0},
    "windspeed_10m": {"ecmwf_ifs025": 0.9, "gfs_seamless": -0.6, "icon_seamless": -0.3, "best_match": 0.0},
}


def bias_of(var, model):
    return BIAS.get(var, {}).get(model, 0.0)


def best_single_mae(var):
    """MAE del miglior singolo modello (offset costante -> MAE=|bias|)."""
    b = BIAS.get(var, {})
    vals = [abs(v) for m, v in b.items() if m in wx.DEFAULT_MODELS]
    return min(vals) if vals else 0.0


def _coords(url):
    q = up.parse_qs(up.urlparse(url).query)
    lats = [float(x) for x in q["latitude"][0].split(",")]
    lons = [float(x) for x in q["longitude"][0].split(",")]
    return lats, lons


def make_get_json(latency=0.0, counters=None, models=None, elev_value=100.0,
                  null_models=None):
    """Ritorna una funzione compatibile con weather_providers._get_json.
    null_models: insieme di modelli che restituiscono None (per test robustezza)."""
    models = models or wx.DEFAULT_MODELS
    null_models = null_models or set()
    times = _times()

    def _get(url, timeout=25):
        if counters is not None:
            k = ("forecast" if "/forecast" in url else
                 "elevation" if "/elevation" in url else
                 "airq" if "air-quality" in url else "other")
            counters[k] = counters.get(k, 0) + 1
        if latency:
            time.sleep(latency)
        lats, lons = _coords(url)
        h = len(times)
        if "/elevation" in url:
            return {"elevation": [elev_value for _ in lats]}
        if "air-quality" in url:
            return [{"hourly": {"time": times, "dust": [30.0] * h}} for _ in lats]
        if "/forecast" in url:
            locs = []
            for la, lo in zip(lats, lons):
                hourly = {"time": times}
                for m in models:
                    for var, fn in GT.items():
                        if m in null_models:
                            hourly[f"{var}_{m}"] = [None] * h
                        else:
                            hourly[f"{var}_{m}"] = [fn(la, lo) + bias_of(var, m)] * h
                locs.append({"hourly": hourly})
            return locs
        raise RuntimeError("mock: url non gestito " + url)

    return _get


def gt_fields(lats, lons):
    """Ground-truth per punto, per confrontare l'accuratezza."""
    return {var: [fn(la, lo) for la, lo in zip(lats, lons)] for var, fn in GT.items()}


@contextmanager
def weather_mock(**kw):
    old = wx._get_json
    wx._ELEV_CACHE.clear()
    wx._get_json = make_get_json(**kw)
    try:
        yield
    finally:
        wx._get_json = old
        wx._ELEV_CACHE.clear()


@contextmanager
def weather_raises(exc):
    old = wx._get_json
    wx._ELEV_CACHE.clear()

    def _raise(url, timeout=25):
        raise exc
    wx._get_json = _raise
    try:
        yield
    finally:
        wx._get_json = old
        wx._ELEV_CACHE.clear()


@contextmanager
def overpass_mock(elements):
    old = rh._overpass
    rh._overpass = lambda q: {"elements": elements}
    try:
        yield
    finally:
        rh._overpass = old


@contextmanager
def osrm_mock(geom):
    old = ms.osrm_route
    ms.osrm_route = lambda a, b: geom
    try:
        yield
    finally:
        ms.osrm_route = old


def reset_service(route_geom):
    """Riporta lo stato globale del servizio a un punto noto per i test."""
    ms.REROUTE_ON = False        # nei test niente reroute automatico (eviter thread in sottofondo)
    state = new_service_state()
    state.update({
        "a": {"name": "A", "lat": route_geom[0][0], "lon": route_geom[0][1]},
        "b": {"name": "B", "lat": route_geom[-1][0], "lon": route_geom[-1][1]},
        "route_geom": route_geom, "route": route_geom,
    })
    ms.G.clear()
    ms.G.update(state)
