"""Discover roadworks and road hazards along a route.

OpenStreetMap data comes from Overpass without a key. Optional live traffic and
incident data comes from TomTom when a key or TOMTOM_KEY is provided. Results
are normalized to dictionaries containing source, type, label, severity,
coordinates, color, and optional delay.
"""

from __future__ import annotations

import json
import os
import urllib.parse
import urllib.request

OVERPASS_ENDPOINTS = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
]
NET = {"bytes": 0}   # byte scaricati (per lo scheduler adattivo)
USER_AGENT = "wsc-road-hazards/1.0"

HAZARD_IT = {
    "animal_crossing": "animal crossing", "falling_rocks": "falling rocks",
    "rockfall": "falling rocks", "curve": "dangerous curve", "dip": "dip",
    "ice": "ice", "children": "children", "slippery": "slippery road",
    "steep_incline": "steep incline", "queues_likely": "queues likely",
    "flooding": "flooding", "wind": "strong wind", "traffic_signals": "traffic signal",
}


# ------------------------------ HTTP (mockabile) --------------------------- #
def _get(url, timeout=25):
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        raw = r.read()
        NET["bytes"] += len(raw)
        return raw.decode("utf-8", "replace")


def _overpass(query):
    body = urllib.parse.urlencode({"data": query}).encode()
    last = None
    for ep in OVERPASS_ENDPOINTS:
        try:
            req = urllib.request.Request(ep, data=body, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=60) as r:
                raw = r.read()
                NET["bytes"] += len(raw)
                return json.loads(raw.decode("utf-8", "replace"))
        except Exception as exc:
            last = exc
            continue
    raise RuntimeError(f"Overpass is unreachable: {last}")


# ------------------------------ waypoint ----------------------------------- #
def _pick_waypoints(cum, step_km, max_pts):
    n = len(cum)
    if n <= 2:
        return list(range(n))
    step = max(step_km, cum[-1] / max_pts)
    idx = [0]; nextd = step
    for i in range(1, n):
        if cum[i] >= nextd:
            idx.append(i); nextd += step
    if idx[-1] != n - 1:
        idx.append(n - 1)
    return idx


# ------------------------------ OSM / Overpass ----------------------------- #
def _build_query(pts, radius_m):
    clauses = []
    for la, lo in pts:
        c = f"{la:.5f},{lo:.5f}"
        clauses += [
            f'way(around:{radius_m},{c})["highway"="construction"];',
            f'way(around:{radius_m},{c})["construction"];',
            f'node(around:{radius_m},{c})["highway"="construction"];',
            f'node(around:{radius_m},{c})["hazard"];',
            f'way(around:{radius_m},{c})["hazard"];',
        ]
    return "[out:json][timeout:60];(" + "".join(clauses) + ");out center tags 400;"


def fetch_osm_hazards(lats, lons, cum, radius_m=300, step_km=12, max_pts=45):
    idx = _pick_waypoints(cum, step_km, max_pts)
    pts = [(lats[i], lons[i]) for i in idx]
    data = _overpass(_build_query(pts, radius_m))
    out = []
    seen = set()
    for el in data.get("elements", []):
        tags = el.get("tags", {}) or {}
        la = el.get("lat"); lo = el.get("lon")
        if la is None and "center" in el:
            la = el["center"].get("lat"); lo = el["center"].get("lon")
        if la is None or lo is None:
            continue
        if "hazard" in tags:
            typ = "hazard"
            hv = tags.get("hazard", "")
            label = "Hazard: " + HAZARD_IT.get(hv, hv.replace("_", " ") or "reported")
            sev, color = 2, "#ef6a5a"
        elif tags.get("highway") == "construction" or "construction" in tags:
            typ = "roadwork"
            label = "Roadworks" + (f" ({tags.get('name')})" if tags.get("name") else "")
            sev, color = 1, "#f2c14e"
        else:
            continue
        key = (round(la, 4), round(lo, 4), typ)
        if key in seen:
            continue
        seen.add(key)
        out.append({"source": "osm", "type": typ, "label": label, "sev": sev,
                    "lat": round(la, 5), "lon": round(lo, 5), "color": color})
    return out


# ------------------------------ TomTom (opzionale) ------------------------- #
_TOMTOM_CAT = {  # iconCategory -> (type, label, severity)
    0: ("unknown", "Traffic event", 1), 1: ("accident", "Accident", 3),
    2: ("fog", "Fog", 1), 3: ("hazard", "Hazardous conditions", 2),
    4: ("rain", "Rain", 1), 5: ("ice", "Ice", 2),
    6: ("jam", "Queue / slowdown", 2), 7: ("lane_closed", "Lane closed", 2),
    8: ("road_closed", "Road closed", 3), 9: ("roadworks", "Roadworks", 1),
    10: ("wind", "Wind", 1), 11: ("flooding", "Flooding", 2),
    14: ("broken_vehicle", "Broken-down vehicle / obstruction", 2),
}


def fetch_tomtom_incidents(min_lon, min_lat, max_lon, max_lat, key, lang="en-US", strict=False):
    """Fetch live incidents inside a bounding box; return [] without a key."""
    if not key:
        return []
    fields = ("{incidents{type,geometry{type,coordinates},"
              "properties{iconCategory,magnitudeOfDelay,delay,events{description}}}}")
    params = {"key": key, "bbox": f"{min_lon},{min_lat},{max_lon},{max_lat}",
              "fields": fields, "language": lang, "timeValidityFilter": "present"}
    url = "https://api.tomtom.com/traffic/services/5/incidentDetails?" + urllib.parse.urlencode(params)
    try:
        data = json.loads(_get(url, timeout=20))
    except Exception as exc:
        if strict:
            raise RuntimeError(f"TomTom is unavailable: {exc}") from exc
        return []
    out = []
    for inc in data.get("incidents", []):
        props = inc.get("properties", {}) or {}
        cat = props.get("iconCategory", 0)
        typ, label, sev = _TOMTOM_CAT.get(cat, ("unknown", "Traffic event", 1))
        geom = inc.get("geometry", {}) or {}
        coords = geom.get("coordinates") or []
        # prendo il primo punto della geometria (lon,lat)
        pt = None
        if geom.get("type") == "Point" and coords:
            pt = coords
        elif coords:
            first = coords[0]
            pt = first[0] if isinstance(first[0], list) else first
        if not pt:
            continue
        desc = ""
        evs = props.get("events") or []
        if evs and isinstance(evs, list):
            desc = evs[0].get("description", "")
        delay = props.get("delay")
        lab = label + (f" — {desc}" if desc else "")
        out.append({"source": "tomtom", "type": typ, "label": lab, "sev": sev,
                    "lat": round(pt[1], 5), "lon": round(pt[0], 5),
                    "color": "#ef6a5a" if sev >= 3 else ("#ef9a6a" if sev == 2 else "#f2c14e"),
                    "delay": delay})
    return out


def fetch_all(lats, lons, cum, tomtom_key=None, radius_m=300):
    """OSM sempre; TomTom solo se c'e' la chiave. Ritorna lista unita."""
    hazards = []
    try:
        hazards += fetch_osm_hazards(lats, lons, cum, radius_m=radius_m)
    except Exception as exc:
        print(f"[hazards] OSM is unavailable: {exc}", flush=True)
    key = tomtom_key or os.environ.get("TOMTOM_KEY")
    if key:
        try:
            min_lat, max_lat = min(lats), max(lats)
            min_lon, max_lon = min(lons), max(lons)
            hazards += fetch_tomtom_incidents(min_lon, min_lat, max_lon, max_lat, key)
        except Exception as exc:
            print(f"[hazards] TomTom is unavailable: {exc}", flush=True)
    return hazards
