"""
map_service.py
==============
Live map service and per-kilometre profile using global weather aggregated from
free services (see weather_providers.py: multi-model Open-Meteo, Elevation, and
Air Quality). The vehicle follows an OSRM road route. Periodic weather updates
produce alerts, toast messages, and desktop notifications.

Incoming UDP:
  setup {type,a{name,lat,lon},b{...}}  -> road route and initial live profile
  telem {type,lat,lon,speed,heading,t} -> GPS position, speed, and heading

HTTP:
  /         complete dashboard
  /profile  route, per-kilometre data, sources, version, and weather status
  /route    road geometry for the simulator
  /state    live vehicle, progress, alerts, events, and sources

Requires network access to Open-Meteo, OSRM, and OSM/CARTO tiles.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
import urllib.request
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import profile_engine as pe
import weather_providers as wx
import road_hazards as rh
from console_utils import safe_print
from service_state import new_service_state
from service_config import parse_config
from service_models import FetchResult
from geo_utils import (
    route_point_distance_m as _route_point_dist_m,
    segment_distance_m as _seg_dist_m,
    simplify_route,
)
from alert_policy import (
    POINT_CFG,
    WX_CFG,
    WX_DEFAULT,
    nearby_alerts,
    notify_config as _notify_cfg,
    notify_window_km as _notify_window_km,
)

LOCK = threading.Lock()
NOMINAL_KMH = 90.0
HAZARD_RADIUS = 300
HAZARD_ON_ROUTE_M = 400.0   # a hazard is on-route within this distance of the polyline
TOMTOM_KEY = None
# --- reroute / rilevazione fuori-rotta ---
REROUTE_ON = True
REROUTE_M = 120.0          # distanza (m) dalla rotta oltre cui il veicolo e' "fuori rotta"
REROUTE_MIN_SAMPLES = 3    # campioni consecutivi oltre soglia prima di ricalcolare
REROUTE_COOLDOWN = 30.0    # secondi minimi fra due reroute (anti-rimbalzo)
FALLBACK_RETRY_S = 120.0   # secondi fra due tentativi di riottenere la rotta reale (se in ripiego)
SOURCES = wx.SOURCE_LABELS + [
    "OpenStreetMap / Overpass (roadworks and hazards)",
    "OpenStreetMap / CARTO (map)", "OSRM (road route)", "Nominatim (geocoding)",
]
G = new_service_state()
TRAIL_MAX = 6000


# ------------------------------ OSRM --------------------------------------- #
def osrm_route(a, b):
    coords = f"{a['lon']},{a['lat']};{b['lon']},{b['lat']}"
    url = ("https://router.project-osrm.org/route/v1/driving/" + coords +
           "?overview=full&geometries=geojson")
    try:
        req = urllib.request.Request(url, headers={"User-Agent": "wsc-map/1.0"})
        with urllib.request.urlopen(req, timeout=12) as r:
            j = json.load(r)
        cc = j["routes"][0]["geometry"]["coordinates"]
        return [[c[1], c[0]] for c in cc]
    except Exception as exc:
        print(f"[osrm] unavailable ({exc}); using a straight line from A to B", flush=True)
        return None


# ---------------------- allerte / eventi ----------------------------------- #
def _add_event(sev, text, score=0.0, dist=None):
    G["event_seq"] += 1
    ev = {"id": G["event_seq"], "t": time.time(), "sev": sev, "text": text, "score": round(score, 1)}
    if dist is not None:
        ev["dist"] = round(dist, 1)
    G["events"].append(ev)
    if len(G["events"]) > 60:
        del G["events"][0:len(G["events"]) - 60]
    safe_print(f"[event] {text}", flush=True)


def _notify_nearby(veh_km, speed_kmh):
    """Emit relevant proximity events; call while holding LOCK."""
    near = nearby_alerts(G["alerts"], veh_km, speed_kmh)
    prev = G["notified"]; used = [False] * len(prev)
    levels = {1: "advisory", 2: "warning", 3: "alarm"}
    for a in near:
        m = None
        for i, p in enumerate(prev):
            if not used[i] and p["type"] == a["type"] and a["km0"] <= p["km1"] and p["km0"] <= a["km1"]:
                m = i; break
        dd = "in progress" if a["dist"] <= 0.05 else f"in {a['dist']:.1f} km"
        if m is None:
            _add_event(a["sev"], f"⚠ {a['label']} — {levels.get(a['sev'], '')} · {dd} · priority {a['score']}",
                       score=a["score"], dist=a["dist"])
        else:
            used[m] = True
            if a["sev"] > prev[m]["sev"]:
                _add_event(a["sev"], f"⚠ {a['label']} — intensifying · {dd} · priority {a['score']}",
                           score=a["score"], dist=a["dist"])
    for i, p in enumerate(prev):                    # non piu' vicine: passate/uscite
        if not used[i] and p["sev"] >= 2:
            _add_event(1, f"✓ {p['label']} — passed", score=0.0)
    G["notified"] = [{"type": a["type"], "label": a["label"], "km0": a["km0"],
                      "km1": a["km1"], "sev": a["sev"]} for a in near]


def _provider(lats, lons, cum):
    with LOCK:
        progress_km = G.get("progress_km", 0.0)
    return wx.fetch_fields(
        lats, lons, cum, nominal_kmh=NOMINAL_KMH, origin_km=progress_km)


# Provider usato da recompute(): normalmente scarica il meteo (rete); in modalita'
# --adaptive lo scheduler lo sostituisce con la cache dei campi (nessuna rete qui:
# gli scaricamenti li fa lo scheduler, recompute ricostruisce solo profilo+allerte).
PROVIDER = _provider


def _hazards_as_alerts(cols, hazards=None):
    la, lo, dd = cols["lat"], cols["lon"], cols["dist"]
    out = []
    for h in G["hazards"] if hazards is None else hazards:
        bi, bd = 0, 1e18
        for i in range(len(la)):
            d = (la[i] - h["lat"]) ** 2 + (lo[i] - h["lon"]) ** 2
            if d < bd:
                bd, bi = d, i
        km = round(dd[bi])
        out.append({"type": h["type"], "label": h["label"], "sev": h["sev"],
                    "km0": km, "km1": km, "lat": h["lat"], "lon": h["lon"],
                    "kind": "point", "color": h.get("color", "#f2c14e")})
    return out


def recompute():
    """Recompute the profile with blended live weather outside the state lock."""
    with LOCK:
        geom = G["route_geom"]
        route_revision = G.get("route_revision", 0)
        hazards = list(G["hazards"])
        backoff_until = G.get("weather_backoff_until", 0.0)
    if not geom:
        return
    if time.time() < backoff_until:
        return  # in backoff dopo un 429: non martellare l'API
    try:
        cols, dist = pe.build_profile(geom, 1.0, provider=PROVIDER)
    except wx.RateLimitError as e:
        wait = max(e.retry_after, 120)
        with LOCK:
            G["weather_backoff_until"] = time.time() + wait
            G["weather_err"] = f"Open-Meteo rate limit (429): retrying in about {wait}s"
        print(f"[weather] 429 Too Many Requests; waiting {wait}s (keeping previous data)", flush=True)
        return
    except Exception as exc:
        with LOCK:
            G["weather_ok"] = False; G["weather_err"] = str(exc)
        print(f"[weather] fetch failed: {exc}", flush=True)
        return
    mp = [{"lat": cols["lat"][i], "lon": cols["lon"][i], "dist": cols["dist"][i],
           "dSpeed": cols["dSpeed"][i], "vPred": cols["vPred"][i], "pv": cols["pv"][i],
           "cloud": cols["cloud"][i], "precip": cols["precip"][i], "dust": cols["dust"][i],
           "along": cols["along"][i]} for i in range(0, len(cols["dist"]), 2)]
    alerts = pe.detect_alerts(cols) + _hazards_as_alerts(cols, hazards)
    alerts.sort(key=lambda a: (a["km0"], -a["sev"]))
    with LOCK:
        if G.get("route_revision", 0) != route_revision:
            return
        G["cols"] = cols; G["map"] = mp; G["dist"] = round(dist, 1)
        G["version"] += 1
        G["alerts"] = alerts
        veh = G["veh"]
        if veh:                       # notifiche pesate per prossimita' (popup + terminale)
            i = _nearest_idx(cols, veh["lat"], veh["lon"])
            _notify_nearby(cols["dist"][i], G["speed"])
        G["weather_ok"] = True; G["weather_err"] = None; G["weather_backoff_until"] = 0.0
        G["updated"] = time.time(); G["ready"] = True


def build_route_and_profile(a, b, reroute=False, request_revision=None):
    """Build an OSRM/fallback route and profile, preserving live state on reroute."""
    with LOCK:
        build_revision = (request_revision if request_revision is not None
                          else G.get("route_request_revision", 0))
        G["building"] = True
        G["building_revision"] = build_revision
    try:
        r = osrm_route(a, b)
        geom = r or [[a["lat"], a["lon"]], [b["lat"], b["lon"]]]
        fallback = r is None
        light = simplify_route(geom, eps_m=10.0)   # segue le curve reali, pochi punti
        with LOCK:
            if (request_revision is not None and
                    G.get("route_request_revision", 0) != request_revision):
                return
            G["route_revision"] = G.get("route_revision", 0) + 1
            G["route_geom"] = geom; G["route"] = light; G["route_fallback"] = fallback
            G["weather_err"] = None; G["offroute_count"] = 0; G["progress_km"] = 0.0
            if not reroute:
                G["a"] = a
                G["veh"] = {"name": a.get("name", "vehicle"), "lat": a["lat"], "lon": a["lon"]}
                G["trail"] = [[a["lat"], a["lon"]]]; G["count"] = 0
                G["events"] = []; G["event_seq"] = 0
                G["version"] = 0; G["ready"] = False
            # in reroute NON azzero ready/version/scia: la pagina continua a mostrare
            # il profilo precedente finche' recompute() non pubblica il nuovo (version++)
        tag = "reroute" if reroute else "route"
        print(f"[{tag}] route with {len(geom)} points{' (straight-line fallback)' if fallback else ''}; recomputing…", flush=True)
        recompute()
        if G["ready"]:
            print(f"[profile] ready: {G['dist']:.0f} km, {len(G['alerts'])} alerts", flush=True)
        elif not reroute:
            print("[profile] weather unavailable on the first attempt; retrying periodically", flush=True)
        threading.Thread(target=fetch_hazards_and_store, daemon=True).start()
    finally:
        with LOCK:
            if G.get("building_revision") == build_revision:
                G["building"] = False
                G["building_revision"] = None


def _dist_to_route_m(lat, lon):
    with LOCK:
        geom = G["route_geom"]
    return _route_point_dist_m(lat, lon, geom)


def _filter_on_route(hazards, geom, max_m=None):
    """Keep hazards in the route corridor and reject nearby parallel-road items."""
    lim = HAZARD_ON_ROUTE_M if max_m is None else max_m
    keep = []
    for h in hazards:
        d = _route_point_dist_m(h.get("lat"), h.get("lon"), geom)
        if d is not None and d <= lim:
            keep.append(h)
    return keep


def maybe_reroute(lat, lon):
    """Retry a fallback route or reroute after consecutive off-route samples."""
    if not REROUTE_ON:
        return
    with LOCK:
        b = G["b"]; building = G["building"]; fallback = G["route_fallback"]
        have_route = bool(G["route_geom"]); last = G["last_reroute"]
        last_fb = G["last_fallback_try"]; name = G["veh"]["name"] if G["veh"] else "vehicle"
    if b is None or building:
        return
    now = time.time()
    need, reason = False, ""
    if not have_route:
        need, reason = True, "route missing"
    elif fallback:
        # ritenta la rotta reale, ma di rado (non ad ogni pacchetto, per i limiti API)
        if now - last_fb >= FALLBACK_RETRY_S:
            with LOCK:
                G["last_fallback_try"] = now
            need, reason = True, "fallback route: retrying the road route"
    else:
        d = _dist_to_route_m(lat, lon)
        with LOCK:
            G["offroute_count"] = G["offroute_count"] + 1 if (d and d > REROUTE_M) else 0
            cnt = G["offroute_count"]
        if cnt >= REROUTE_MIN_SAMPLES and now - last >= REROUTE_COOLDOWN:
            need, reason = True, f"off route by ~{d:.0f} m"
    if not need:
        return
    with LOCK:
        G["last_reroute"] = now; G["offroute_count"] = 0
        G["route_request_revision"] = G.get("route_request_revision", 0) + 1
        request_revision = G["route_request_revision"]
    print(f"[reroute] {reason}; recomputing from the current position to B", flush=True)
    a = {"name": name, "lat": lat, "lon": lon}
    threading.Thread(target=build_route_and_profile, args=(a, b),
                     kwargs={"reroute": True, "request_revision": request_revision}, daemon=True).start()


def _validated_point(value, field_name):
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must be an object")
    try:
        lat = float(value["lat"])
        lon = float(value["lon"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} requires numeric lat/lon") from exc
    if not math.isfinite(lat) or not math.isfinite(lon):
        raise ValueError(f"{field_name} coordinates must be finite")
    if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
        raise ValueError(f"{field_name} coordinates out of range")
    name = str(value.get("name") or field_name)[:160]
    return {"name": name, "lat": lat, "lon": lon}


def handle_message(msg):
    if not isinstance(msg, dict):
        raise ValueError("message must be a JSON object")
    typ = msg.get("type")
    if typ == "setup":
        a = _validated_point(msg.get("a"), "a")
        b = _validated_point(msg.get("b"), "b")
        with LOCK:
            G["a"] = a; G["b"] = b; G["ready"] = False
            G["route_request_revision"] = G.get("route_request_revision", 0) + 1
            request_revision = G["route_request_revision"]
        print(f"[setup] A={a} B={b}", flush=True)
        threading.Thread(target=build_route_and_profile,
                         args=(a, b), kwargs={"request_revision": request_revision},
                         daemon=True).start()
    elif typ == "telem":
        lat = float(msg["lat"]); lon = float(msg["lon"])
        if (not math.isfinite(lat) or not math.isfinite(lon) or
                not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0):
            raise ValueError("telemetry coordinates out of range")
        with LOCK:
            name = G["veh"]["name"] if G["veh"] else "vehicle"
            G["veh"] = {"name": name, "lat": lat, "lon": lon}
            G["speed"] = float(msg.get("speed", 0.0))
            G["heading"] = float(msg.get("heading", 0.0))
            G["t"] = float(msg.get("t", time.time())); G["count"] += 1
            tr = G["trail"]
            if not tr or tr[-1] != [lat, lon]:
                tr.append([lat, lon])
                if len(tr) > TRAIL_MAX:
                    del tr[0:len(tr) - TRAIL_MAX]
            now = time.time()         # notifiche di prossimita' (throttle ~1s), a lock tenuto
            if G["cols"]:
                progress_km = G["cols"]["dist"][_nearest_idx(G["cols"], lat, lon)]
                G["progress_km"] = progress_km
                if now - G.get("last_notify", 0.0) >= 1.0:
                    G["last_notify"] = now
                    _notify_nearby(progress_km, G["speed"])
        maybe_reroute(lat, lon)      # ricalcola la rotta se serve (ripiego o fuori rotta)
    elif typ == "link":
        # campioni di qualita' rete dal companion sul telefono (cella/RTT) o dal device (WiFi)
        with LOCK:
            if msg.get("cell_type") is not None or msg.get("cell_dbm") is not None:
                G["link_cell"] = (msg.get("cell_type"), msg.get("cell_dbm"))
            if msg.get("rssi") is not None or msg.get("mbps") is not None:
                G["link_wifi"] = (msg.get("rssi"), msg.get("mbps"))
            if msg.get("rtt_ms") is not None:
                G["link_rtt"] = (float(msg["rtt_ms"]), float(msg.get("loss", 0.0)))
    else:
        raise ValueError(f"unknown message type: {typ!r}")


def udp_listener(port, host="127.0.0.1"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    print(f"[udp] listening on {host}:{port}", flush=True)
    while True:
        try:
            data, _ = s.recvfrom(65535)
            handle_message(json.loads(data.decode("utf-8")))
        except Exception as exc:
            print(f"[udp] ignored message: {exc}", flush=True)


def weather_updater(period):
    print(f"[weather] live update every {period:.0f}s", flush=True)
    while True:
        time.sleep(period)
        if G["route_geom"]:
            recompute()
            if G["weather_ok"]:
                print(f"[weather] updated · version {G['version']} · {len(G['alerts'])} alerts", flush=True)


def fetch_hazards_and_store():
    with LOCK:
        geom = G["route_geom"]
        route_revision = G.get("route_revision", 0)
    if not geom:
        return
    la, lo, cum, _ = pe.resample(geom, 2.0)
    try:
        hz = rh.fetch_all(la, lo, cum, tomtom_key=TOMTOM_KEY, radius_m=HAZARD_RADIUS)
    except Exception as exc:
        print(f"[hazards] error: {exc}", flush=True)
        return
    n_raw = len(hz)
    hz = _filter_on_route(hz, geom)          # scarta quelli fuori dal corridoio della rotta
    with LOCK:
        if G.get("route_revision", 0) != route_revision:
            return
        G["hazards"] = hz
    n_rw = sum(1 for h in hz if h["type"] in ("roadwork", "roadworks"))
    off = n_raw - len(hz)
    print(f"[hazards] {len(hz)} items ({n_rw} roadworks) along the route"
          + (f"; discarded {off} off-route items" if off else ""), flush=True)
    recompute()


def hazard_updater(period):
    while True:
        time.sleep(period)
        if G["route_geom"]:
            fetch_hazards_and_store()


def adaptive_updater(period=20.0):
    """Schedule data downloads by link quality while local map state remains live."""
    global PROVIDER
    import comm_scheduler as cs
    print("[adaptive] waiting for a route: send setup from vehicle_sim to start cycles…", flush=True)
    while not G["route_geom"]:
        time.sleep(0.5)
    with LOCK:
        route_geom = G["route_geom"]
        store_revision = G.get("route_revision", 0)
    store = cs.FieldStore(route_geom, NOMINAL_KMH)
    PROVIDER = store.provider          # recompute ricostruisce dalla cache (niente rete qui)
    mon = cs.LinkMonitor(); est = cs.SizeEstimator(); ages = {}
    print(f"[adaptive] scheduler active ({period:.0f}s cycle) — map/events remain live locally", flush=True)

    def _car_km():
        v = G.get("veh")
        if v and G.get("cols"):
            i = nearest_km(v["lat"], v["lon"])
            if i is not None:
                return G["cols"]["dist"][i]
        return 0.0

    def _ctx():
        return {"route_total_km": G.get("dist") or store.cum[-1], "car_km": _car_km(),
                "kmh": NOMINAL_KMH, "period": period, "tomtom": bool(TOMTOM_KEY), "_est": est}

    def _ahead(horizon, c):
        cu = store.cum
        idx = [i for i in range(len(cu)) if c["car_km"] - 5 <= cu[i] <= c["car_km"] + horizon]
        if not idx:
            idx = list(range(len(cu)))
        return [store.lats[i] for i in idx], [store.lons[i] for i in idx], [cu[i] for i in idx]

    def ex_weather(spec, c):
        wx.NET["bytes"] = 0; t0 = time.time()
        try:
            store.ensure_elevation()
            sel = store.car_wp_window(c["car_km"], spec["horizon"], spec.get("n_wp"))
            store.update_weather(sel, spec["models"], spec["vars"],
                                 spec["store_fields"], car_km=c["car_km"])
            return FetchResult(wx.NET["bytes"], time.time() - t0, True)
        except wx.RateLimitError as e:
            return FetchResult(wx.NET["bytes"], time.time() - t0, False, e.retry_after,
                               str(e))
        except Exception as exc:
            print(f"[adaptive] weather failed: {exc}", flush=True)
            return FetchResult(wx.NET["bytes"], time.time() - t0, False, error=str(exc))

    def ex_dust(spec, c):
        wx.NET["bytes"] = 0; t0 = time.time()
        try:
            sel = store.car_wp_window(c["car_km"], spec["horizon"], spec.get("n_wp"))
            store.update_dust(sel, car_km=c["car_km"])
            return FetchResult(wx.NET["bytes"], time.time() - t0, True)
        except wx.RateLimitError as e:
            return FetchResult(wx.NET["bytes"], time.time() - t0, False, e.retry_after,
                               str(e))
        except Exception as exc:
            return FetchResult(wx.NET["bytes"], time.time() - t0, False, error=str(exc))

    def ex_haz(spec, c):
        rh.NET["bytes"] = 0; t0 = time.time()
        try:
            la, lo, cu = _ahead(spec["horizon"], c)
            hz = rh.fetch_osm_hazards(la, lo, cu, radius_m=spec.get("radius", HAZARD_RADIUS))
            hz = _filter_on_route(hz, G["route_geom"])   # solo dentro il corridoio della rotta
            with LOCK:
                G["hazards"] = hz
            return FetchResult(rh.NET["bytes"], time.time() - t0, True)
        except Exception as exc:
            print(f"[adaptive] roadworks failed: {exc}", flush=True)
            return FetchResult(rh.NET["bytes"], time.time() - t0, False, error=str(exc))

    def ex_traffic(spec, c):
        if not TOMTOM_KEY:
            return FetchResult(0, 0.0, False, error="TomTom key missing")
        rh.NET["bytes"] = 0; t0 = time.time()
        try:
            la, lo, _ = _ahead(spec["horizon"], c)
            inc = rh.fetch_tomtom_incidents(
                min(lo), min(la), max(lo), max(la), TOMTOM_KEY, strict=True)
            inc = _filter_on_route(inc, G["route_geom"])  # scarta gli incidenti fuori rotta (bbox)
            with LOCK:
                G["hazards"] = [h for h in G["hazards"] if h.get("source") != "tomtom"] + inc
            return FetchResult(rh.NET["bytes"], time.time() - t0, True)
        except Exception as exc:
            return FetchResult(rh.NET["bytes"], time.time() - t0, False, error=str(exc))

    execs = {"weather_ahead": ex_weather, "weather_full": ex_weather,
             "dust": ex_dust, "hazards": ex_haz, "traffic": ex_traffic}

    while True:
        with LOCK:
            current_revision = G.get("route_revision", 0)
            current_geom = G.get("route_geom")
        if current_geom and current_revision != store_revision:
            store = cs.FieldStore(current_geom, NOMINAL_KMH)
            PROVIDER = store.provider
            ages.clear()
            store_revision = current_revision
            print(f"[adaptive] cache rebuilt for route r{store_revision}", flush=True)
        w = cs.probe_linux_wifi()          # funziona su Linux; su Windows torna {}
        if not w:
            lw = G.get("link_wifi")        # fallback: WiFi inviato via messaggio "link"
            if lw:
                w = {"rssi": lw[0], "mbps": lw[1]}
        if w:
            mon.sample_wifi(w.get("rssi"), w.get("mbps"))
        lc = G.get("link_cell")
        if lc and lc[0] is not None:
            mon.sample_cell(lc[0], lc[1])
        lr = G.get("link_rtt")
        if lr:
            mon.sample_rtt(lr[0], lr[1])
        if G["route_geom"]:
            rep = cs.adaptive_cycle(mon, ages, _ctx(), execs, est)
            got = [r for r in rep if r["ok"]]
            if got:
                recompute()             # un solo ricalcolo profilo+allerte per ciclo
                tot = sum(r["downloaded"] for r in got)
                classi = ", ".join(r["variant"] for r in got)
                print(f"[adaptive] link {mon.link_score():.2f} · downloaded {len(got)}/{len(rep)} "
                      f"({cs.human_bytes(tot)}: {classi}) · v{G['version']}", flush=True)
            else:
                reason = "link down" if not mon.is_up() else "data still fresh"
                print(f"[adaptive] link {mon.link_score():.2f} · no download ({reason}) · "
                      f"map/events live locally", flush=True)
        time.sleep(period)


def _nearest_idx(cols, lat, lon):
    la, lo = cols["lat"], cols["lon"]
    best_i, best_d = 0, 1e18
    for i in range(len(la)):
        d = (la[i] - lat) ** 2 + (lo[i] - lon) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def nearest_km(lat, lon):
    cols = G["cols"]
    if not cols:
        return None
    return _nearest_idx(cols, lat, lon)


def state_snapshot():
    with LOCK:
        now = time.time()
        snap = {"veh": G["veh"], "a": G["a"], "b": G["b"],
                "speed": round(G["speed"], 1), "heading": round(G["heading"], 1),
                "t": G["t"], "count": G["count"], "trail": G["trail"][-2000:],
                "connected": (G["count"] > 0 and (now - G["t"]) < 5.0),
                "age": round(now - G["t"], 1) if G["t"] else None,
                "version": G["version"], "alerts": G["alerts"], "events": G["events"][-25:],
                "weatherOk": G["weather_ok"], "updated": G["updated"], "sources": SOURCES,
                "km": None, "vPredHere": None, "alongHere": None, "pvHere": None,
                "cloudHere": None, "elevHere": None}
        if G["veh"] and G["cols"]:
            i = nearest_km(G["veh"]["lat"], G["veh"]["lon"])
            if i is not None:
                c = G["cols"]
                snap["km"] = c["dist"][i]; snap["vPredHere"] = c["vPred"][i]
                snap["alongHere"] = c["along"][i]; snap["pvHere"] = c["pv"][i]
                snap["cloudHere"] = c["cloud"][i]; snap["elevHere"] = c["elev"][i]
        return snap


def profile_payload():
    with LOCK:
        if not G["ready"]:
            return {"ready": False, "weatherOk": G["weather_ok"],
                    "weatherErr": G["weather_err"], "sources": SOURCES}
        return {"ready": True, "version": G["version"], "a": G["a"], "b": G["b"],
                "dist": G["dist"], "route": G["route"], "map": G["map"], "cols": G["cols"],
                "hazards": G["hazards"],
                "weatherOk": G["weather_ok"], "updated": G["updated"], "sources": SOURCES}


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, ctype, body):
        self.send_response(code); self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store"); self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/state":
            self._send(200, "application/json", json.dumps(state_snapshot()).encode())
        elif path == "/profile":
            self._send(200, "application/json", json.dumps(profile_payload()).encode())
        elif path == "/route":
            with LOCK:
                pl = {"ready": G["ready"], "geom": G["route"]}
            self._send(200, "application/json", json.dumps(pl).encode())
        elif path in ("/", "/index", "/index.html"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode("utf-8"))
        else:
            self._send(404, "text/plain", b"not found")

    def log_message(self, *a):
        pass


PAGE = r"""<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>WSC · Live weather and alerts</title>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css">
<script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
<script src="https://cdn.plot.ly/plotly-2.27.0.min.js" charset="utf-8"></script>
<style>
  :root{--ink:#e9eef3;--muted:#8b98a6;--line:#22303f;--panel:#141c26;--panel2:#0f161f;
        --wind:#54d6e6;--good:#63d18c;--adverse:#ef6a5a;
        --mono:ui-monospace,SFMono-Regular,"SF Mono",Menlo,Consolas,monospace;--sans:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;}
  *{box-sizing:border-box}
  body{margin:0;background:radial-gradient(120% 80% at 70% -10%,#16202d 0%,#0d1420 45%,#0a0f18 100%);color:var(--ink);font-family:var(--sans);padding:16px}
  .wrap{max-width:1120px;margin:0 auto}
  h1{font-size:17px;letter-spacing:.14em;text-transform:uppercase;margin:0;font-weight:700}
  .sub{color:var(--muted);font-size:12px;margin:6px 0 0}
  .card{background:linear-gradient(180deg,var(--panel),var(--panel2));border:1px solid var(--line);border-radius:14px;padding:12px 14px;margin-top:14px}
  .card h2{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin:0 0 6px;font-weight:700}
  #map{height:min(56vh,520px);min-height:340px;border-radius:10px;background:#0a0f18;position:relative}
  #chart{width:100%;height:660px}
  .hint{font-family:var(--mono);font-size:11px;color:var(--muted);margin:0 0 8px}
  .leaflet-container{background:#0a0f18;font-family:var(--mono)}
  .leaflet-control-attribution{background:rgba(10,15,24,.75)!important;color:#7d8894}
  .leaflet-control-attribution a{color:#8b98a6}
  .leaflet-control-layers{background:#141c26;color:var(--ink);border:1px solid var(--line);font-family:var(--mono);font-size:11px}
  .leaflet-control-layers-expanded{padding:8px 10px}
  .veh{width:40px;height:40px;transition:transform .3s linear}.veh svg,.dest svg{display:block}
  .wx{background:none;border:none}
  #hud{position:absolute;top:12px;left:12px;z-index:1000;min-width:210px;background:rgba(15,22,31,.92);border:1px solid var(--line);border-radius:12px;padding:11px 13px;font-family:var(--mono)}
  #hud .big{font-size:30px;font-weight:600;line-height:1;color:var(--wind)}#hud .big small{font-size:11px;color:var(--muted);font-weight:400}
  #hud .row{font-size:12px;margin-top:5px}#hud .lab{color:var(--muted)}#hud b{color:#c7d2dc}
  #stat{margin-top:7px;font-size:12px;display:flex;align-items:center;gap:7px}
  #dot{width:9px;height:9px;border-radius:50%;background:var(--adverse)}#dot.on{background:var(--good)}
  #alerts{position:absolute;bottom:12px;right:12px;z-index:1000;width:238px;max-height:56%;overflow:auto;background:rgba(15,22,31,.92);border:1px solid var(--line);border-radius:12px;padding:9px 11px;font-family:var(--mono)}
  #alerts .t{font-size:10px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted);margin-bottom:6px}
  .al{display:flex;gap:8px;align-items:flex-start;font-size:11.5px;margin:5px 0;padding-left:8px;border-left:3px solid var(--muted)}
  .al small{color:var(--muted)}
  .legendbox{font-family:var(--mono);font-size:11px;color:var(--ink);background:rgba(15,22,31,.9);border:1px solid var(--line);border-radius:8px;padding:7px 9px;line-height:1.7}
  .legendbox .row{display:flex;align-items:center;gap:7px}.legendbox .sw{width:20px;height:4px;border-radius:2px}
  #toasts{position:fixed;top:16px;right:16px;z-index:3000;display:flex;flex-direction:column;gap:8px;max-width:320px}
  .toast{background:rgba(20,28,38,.97);border:1px solid var(--line);border-left-width:4px;border-radius:10px;padding:10px 12px;font-family:var(--mono);font-size:12px;box-shadow:0 8px 26px rgba(0,0,0,.5);animation:sl .25s ease}
  @keyframes sl{from{transform:translateX(20px);opacity:0}to{transform:none;opacity:1}}
  .toast .h{font-weight:700;margin-bottom:2px}
  .foot{color:var(--muted);font-size:11px;margin-top:14px;font-family:var(--mono)}
  .metaline{font-family:var(--mono);font-size:11px;color:var(--muted);margin-top:6px}
  #warn{display:none;margin-top:10px;font-family:var(--mono);font-size:12px;color:#ffcaa6;background:rgba(70,40,20,.5);border:1px solid #6a4a2a;border-radius:8px;padding:8px 10px}
</style></head>
<body>
<div id="toasts"></div>
<div class="wrap">
  <div style="display:flex;justify-content:space-between;align-items:baseline;gap:12px;flex-wrap:wrap">
    <h1>WSC · Live weather</h1>
    <span class="sub" id="banner">waiting for setup (send A and B from the simulator)…</span>
  </div>
  <div id="warn"></div>
  <div class="card">
    <h2>Map — live vehicle, route conditions, weather, and alerts</h2>
    <div id="map">
      <div id="hud">
        <div class="big"><span id="spd">–</span> <small>measured km/h</small></div>
        <div class="row"><span class="lab">heading</span> <b id="hdg">–</b></div>
        <div class="row"><span class="lab">position (WGS84)</span> <b id="pos">–</b></div>
        <div class="row"><span class="lab">km / predicted</span> <b id="prog">–</b></div>
        <div class="row"><span class="lab">current elevation</span> <b id="elev">–</b></div>
        <div class="row"><span class="lab">→</span> <b id="dest">–</b> · <b id="dist">–</b></div>
        <div id="stat"><span id="dot"></span><span id="stattxt">waiting…</span></div>
      </div>
      <div id="alerts"><div class="t">Alerts along the route</div><div id="allist">—</div></div>
    </div>
    <div class="metaline" id="meta">weather: —</div>
  </div>
  <div class="card">
    <h2>Per-kilometre profile — speed, wind effect, solar charging, and elevation</h2>
    <p class="hint">the vertical line follows the vehicle · refreshed after each weather fetch · zoom: drag/scroll</p>
    <div id="chart"></div>
  </div>
  <div class="card">
    <h2>Live data — measured speed and vehicle elevation</h2>
    <p class="hint">live telemetry series · onboard measured speed · elevation from the profile at the current position</p>
    <div id="live" style="height:240px"></div>
  </div>
  <div class="foot" id="foot"></div>
</div>
<script>
if('Notification' in window && Notification.permission==='default'){try{Notification.requestPermission();}catch(e){}}
const map=L.map('map',{zoomControl:false,scrollWheelZoom:true,preferCanvas:true}).setView([20,10],3);
L.tileLayer('https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',{subdomains:'abcd',maxZoom:19,
  attribution:'&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> · &copy; <a href="https://carto.com/attributions">CARTO</a>'}).addTo(map);

const COLORINGS={
  pv:{label:'Solar charging (PV)',fn:v=>v>=700?'#63d18c':v>=500?'#8fd0a0':v>=350?'#f2c14e':v>=200?'#ef9a6a':'#ef6a5a',
      legend:[['≥700 W','#63d18c'],['500–700','#8fd0a0'],['350–500','#f2c14e'],['200–350','#ef9a6a'],['<200 W','#ef6a5a']]},
  wind:{label:'Wind effect',fn:v=>v>=8?'#63d18c':v>=3?'#8fd0a0':v>-3?'#c9cf7a':v>-8?'#ef9a6a':'#ef6a5a',
      legend:[['+≥8 km/h','#63d18c'],['+3…8','#8fd0a0'],['−3…+3','#c9cf7a'],['−8…−3','#ef9a6a'],['≤−8','#ef6a5a']]},
  speed:{label:'Predicted speed',fn:v=>v>=125?'#63d18c':v>=110?'#8fd0a0':v>=95?'#f2c14e':v>=80?'#ef9a6a':'#ef6a5a',
      legend:[['≥125 km/h','#63d18c'],['110–125','#8fd0a0'],['95–110','#f2c14e'],['80–95','#ef9a6a'],['<80','#ef6a5a']]},
};
const CKARR={pv:'pv',wind:'dSpeed',speed:'vPred'};
function coloredLayerGeo(route,cols,ck){
  // disegna la rotta lungo i vertici REALI della strada (nessun taglio delle curve),
  // colorando ogni tratto col valore per-km piu' vicino (indice km = distanza cumulata)
  const fn=COLORINGS[ck].fn,arr=cols[CKARR[ck]],nd=cols.dist.length,g=L.layerGroup();
  if(!route||route.length<2){return g;}
  L.polyline(route,{color:'#000',weight:9,opacity:.18}).addTo(g);
  const cum=[0];for(let i=1;i<route.length;i++)cum[i]=cum[i-1]+hav(route[i-1],route[i]);
  const colAt=cd=>fn(arr[Math.max(0,Math.min(nd-1,Math.round(cd)))]);
  let s=0,cur=colAt((cum[0]+cum[1])/2);
  for(let i=1;i<route.length;i++){const c=colAt((cum[i-1]+cum[i])/2);
    if(c!==cur){const seg=route.slice(s,i+1);if(seg.length>=2)L.polyline(seg,{color:cur,weight:4.5,opacity:.95}).addTo(g);s=i;cur=c;}}
  const tail=route.slice(s);if(tail.length>=2)L.polyline(tail,{color:cur,weight:4.5,opacity:.95}).addTo(g);
  return g;}
function zoneLayer(pts,key,thr,color,o){const g=L.layerGroup();let last=-1e9;
  pts.forEach(p=>{const val=p[key];if(val>thr&&p.dist-last>=o.every){L.circle([p.lat,p.lon],
    {radius:o.rBase+val*o.rScale,color,weight:0,fillColor:color,fillOpacity:Math.min(.45,o.oBase+val*o.oScale)}).addTo(g);last=p.dist;}});return g;}
function wxGlyph(k){if(k==='sun')return '<circle cx=14 cy=14 r=6 fill="#ffcb3d"/>'+[0,45,90,135,180,225,270,315].map(a=>{const r=a*Math.PI/180;return `<line x1=${14+8*Math.cos(r)} y1=${14+8*Math.sin(r)} x2=${14+11*Math.cos(r)} y2=${14+11*Math.sin(r)} stroke="#ffcb3d" stroke-width=1.6/>`;}).join('');
  if(k==='partly')return '<circle cx=10 cy=10 r=4.5 fill="#ffcb3d"/><ellipse cx=16 cy=18 rx=9 ry=5.5 fill="#9aa7b3"/><ellipse cx=10 cy=19 rx=6 ry=4.5 fill="#9aa7b3"/>';
  if(k==='cloud')return '<ellipse cx=14 cy=15 rx=10 ry=6 fill="#9aa7b3"/><ellipse cx=8 cy=16 rx=6 ry=5 fill="#9aa7b3"/><ellipse cx=19 cy=16 rx=6 ry=5 fill="#9aa7b3"/>';
  if(k==='rain')return '<ellipse cx=14 cy=12 rx=10 ry=6 fill="#8a97a3"/>'+[8,14,20].map(x=>`<line x1=${x} y1=19 x2=${x-2} y2=25 stroke="#4aa3e0" stroke-width=2/>`).join('');
  if(k==='dust')return [10,15,20].map(y=>`<path d="M3 ${y} q6 -3 11 0 t11 0" fill="none" stroke="#c9a06a" stroke-width=1.8/>`).join('');return '';}
function pickKind(p){if(p.precip>0.3)return'rain';if(p.cloud>65)return'cloud';if(p.dust>150)return'dust';if(p.cloud>30)return'partly';return'sun';}
const vehIconMk=h=>L.divIcon({className:'',iconSize:[40,40],iconAnchor:[20,20],html:'<div class="veh" style="transform:rotate('+h+'deg)"><svg width=40 height=40 viewBox="0 0 40 40"><circle cx=20 cy=20 r=8 fill="#54d6e6" fill-opacity=.22/><path d="M20 5 L28 27 L20 22 L12 27 Z" fill="#54d6e6" stroke="#0b1119" stroke-width=1.2/></svg></div>'});
const destIcon=L.divIcon({className:'',iconSize:[30,38],iconAnchor:[15,37],html:'<div class="dest"><svg width=30 height=38 viewBox="0 0 30 38" style="filter:drop-shadow(0 0 4px #000)"><path d="M15 37 C4 22 2 16 2 12 A13 13 0 0 1 28 12 C28 16 26 22 15 37 Z" fill="#ef6a5a" stroke="#0b1119" stroke-width=1.4/><circle cx=15 cy=12 r=5 fill="#fff"/></svg></div>'});
const startIcon=L.divIcon({className:'',iconSize:[14,14],iconAnchor:[7,7],html:'<div style="width:12px;height:12px;border-radius:50%;background:#63d18c;border:2px solid #0b1119;filter:drop-shadow(0 0 3px #000)"></div>'});
function hazIcon(type,sev){const col=sev>=3?'#ef6a5a':(sev===2?'#ef9a6a':'#f2c14e');let inner;
  if(type==='roadwork'||type==='roadworks')inner='<path d="M14 4 L24 22 L4 22 Z" fill="'+col+'" stroke="#0b1119" stroke-width=1.2/><rect x=11 y=12 width=6 height=6 fill="#111" transform="rotate(45 14 15)"/>';
  else inner='<path d="M14 3 L25 23 L3 23 Z" fill="'+col+'" stroke="#0b1119" stroke-width=1.4/><rect x=13 y=10 width=2 height=7 fill="#111"/><rect x=13 y=19 width=2 height=2 fill="#111"/>';
  return L.divIcon({className:'wx',html:'<svg width=26 height=26 viewBox="0 0 28 28" style="filter:drop-shadow(0 0 3px #000)">'+inner+'</svg>',iconSize:[26,26],iconAnchor:[13,20]});}
const SEVCOL={1:'#f2c14e',2:'#ef9a6a',3:'#ef6a5a'};
function esc(value){return String(value??'').replace(/[&<>"']/g,ch=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));}

let profileLoaded=false,chartReady=false,lastVersion=-1,LAYOUT=null,lastKm=0;
let liveInit=false,liveReady=false,LIVE_LAYOUT=null;const LIVE_MAX=720;
let startM=null,destM=null,vehM=null,trailL=null,remL=null,fitted=false;
let profLayers=[],control=null,legend=null;
let seenEvents=new Set(),firstState=false;
function hav(a,b){const R=6371.0088,t=x=>x*Math.PI/180;const dφ=t(b[0]-a[0]),dλ=t(b[1]-a[1]);
  const s=Math.sin(dφ/2)**2+Math.cos(t(a[0]))*Math.cos(t(b[0]))*Math.sin(dλ/2)**2;return 2*R*Math.asin(Math.sqrt(s));}

function buildTraces(C){
  const d=C.dist,dPos=C.dSpeed.map(v=>v>0?v:null),dNeg=C.dSpeed.map(v=>v<0?v:null);
  const cd=d.map((_,i)=>[C.windSpeed[i],C.windDir[i],C.along[i],C.grade[i],C.pv[i],C.cloud[i],C.gust[i],C.etaH[i]]);
  return [
    {x:d,y:C.elev,name:'elevation (m)',xaxis:'x',yaxis:'y5',mode:'lines',line:{color:'#c98b4a',width:1.1},fill:'tozeroy',fillcolor:'rgba(201,139,74,.14)',hoverinfo:'skip'},
    {x:d,y:C.pv,name:'PV (W)',xaxis:'x',yaxis:'y3',mode:'lines',line:{color:'#ffcb3d',width:1.6},fill:'tozeroy',fillcolor:'rgba(255,203,61,.16)',hoverinfo:'skip'},
    {x:d,y:C.cloud,name:'cloud (%)',xaxis:'x',yaxis:'y4',mode:'lines',line:{color:'#9aa7b3',width:1.2,dash:'dot'},hoverinfo:'skip'},
    {x:d,y:dPos,name:'wind speeds up',xaxis:'x',yaxis:'y2',mode:'lines',line:{color:'#63d18c',width:0.5},fill:'tozeroy',fillcolor:'rgba(99,209,140,.45)',hoverinfo:'skip'},
    {x:d,y:dNeg,name:'wind slows down',xaxis:'x',yaxis:'y2',mode:'lines',line:{color:'#ef6a5a',width:0.5},fill:'tozeroy',fillcolor:'rgba(239,106,90,.45)',hoverinfo:'skip'},
    {x:[d[0],d[d.length-1]],y:[130,130],name:'130 cap',xaxis:'x',yaxis:'y',mode:'lines',line:{color:'#ef6a5a',width:1,dash:'dash'},hoverinfo:'skip'},
    {x:d,y:C.vNoWind,name:'no wind',xaxis:'x',yaxis:'y',mode:'lines',line:{color:'#8b98a6',width:1.2,dash:'dot'},hoverinfo:'skip'},
    {x:d,y:C.vPred,name:'predicted speed',xaxis:'x',yaxis:'y',mode:'lines',line:{color:'#54d6e6',width:2},customdata:cd,
      hovertemplate:'km %{x:.0f}<br>speed <b>%{y:.0f} km/h</b><br>wind %{customdata[0]:.1f} m/s from %{customdata[1]:.0f}° · gusts %{customdata[6]:.0f}<br>tail/head %{customdata[2]:+.1f} · PV %{customdata[4]:.0f} W · cloud %{customdata[5]:.0f}%<br>grade %{customdata[3]:+.2f}% · ETA %{customdata[7]:.1f} h<extra></extra>'}
  ];
}
function buildLayout(a,b,dist){const anns=[];
  if(a)anns.push({x:0,y:1,xref:'x',yref:'paper',yanchor:'bottom',text:(a.name||'A').split(' ')[0],showarrow:false,font:{size:9,color:'#63d18c'}});
  if(b)anns.push({x:dist,y:1,xref:'x',yref:'paper',yanchor:'bottom',text:(b.name||'B').split(' ')[0],showarrow:false,font:{size:9,color:'#ef6a5a'}});
  return {paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{color:'#c7d2dc',family:'var(--mono)',size:11},margin:{l:56,r:52,t:20,b:40},hovermode:'x unified',showlegend:true,
    legend:{orientation:'h',y:1.10,x:0,font:{size:10},bgcolor:'rgba(0,0,0,0)'},
    xaxis:{domain:[0,1],anchor:'y5',title:{text:'distance (km)',font:{size:11}},gridcolor:'#182430',showspikes:true,spikecolor:'#54d6e6',spikethickness:1,spikemode:'across'},
    yaxis:{domain:[0.76,1],title:{text:'km/h',font:{size:11}},gridcolor:'#182430',range:[0,138]},
    yaxis2:{domain:[0.545,0.71],title:{text:'Δ km/h',font:{size:11}},gridcolor:'#182430',zerolinecolor:'#33465a'},
    yaxis3:{domain:[0.29,0.49],title:{text:'PV (W)',font:{size:11}},gridcolor:'#182430'},
    yaxis4:{domain:[0.29,0.49],overlaying:'y3',side:'right',range:[0,100],title:{text:'cloud %',font:{size:10}},showgrid:false},
    yaxis5:{domain:[0,0.22],title:{text:'elevation (m)',font:{size:11}},gridcolor:'#182430'},
    annotations:anns,shapes:[{type:'line',xref:'x',yref:'paper',x0:lastKm,x1:lastKm,y0:0,y1:1,line:{color:'#fff',width:1.5},opacity:0.7}]};
}
function moveCursor(km){if(chartReady&&km!=null){lastKm=km;Plotly.relayout('chart',{'shapes[0].x0':km,'shapes[0].x1':km});}}
function initLive(){
  LIVE_LAYOUT={paper_bgcolor:'rgba(0,0,0,0)',plot_bgcolor:'rgba(0,0,0,0)',font:{color:'#c7d2dc',family:'var(--mono)',size:11},
    margin:{l:52,r:54,t:24,b:34},showlegend:true,legend:{orientation:'h',y:1.16,x:0,font:{size:10},bgcolor:'rgba(0,0,0,0)'},
    xaxis:{type:'date',title:{text:'time',font:{size:10}},gridcolor:'#182430'},
    yaxis:{title:{text:'measured km/h',font:{size:11}},gridcolor:'#182430',range:[0,138]},
    yaxis2:{title:{text:'elevation (m)',font:{size:11}},overlaying:'y',side:'right',showgrid:false}};
  Plotly.newPlot('live',[
    {x:[],y:[],name:'measured speed (km/h)',mode:'lines',line:{color:'#54d6e6',width:2},yaxis:'y'},
    {x:[],y:[],name:'elevation (m)',mode:'lines',line:{color:'#c98b4a',width:1.4},fill:'tozeroy',fillcolor:'rgba(201,139,74,.14)',yaxis:'y2'}
  ],LIVE_LAYOUT,{responsive:true,displaylogo:false,displayModeBar:false}).then(()=>{liveReady=true;});
}
function pushLive(v,e){if(!liveReady)return;const t=new Date();
  Plotly.extendTraces('live',{x:[[t],[t]],y:[[v==null?null:v],[e==null?null:e]]},[0,1],LIVE_MAX);}
function removeProfLayers(){profLayers.forEach(l=>map.removeLayer(l));profLayers=[];if(control){map.removeControl(control);control=null;}if(legend){map.removeControl(legend);legend=null;}}
function buildProfLayers(p){const pts=p.map;
  const pvL=coloredLayerGeo(p.route,p.cols,'pv'),windL=coloredLayerGeo(p.route,p.cols,'wind'),spdL=coloredLayerGeo(p.route,p.cols,'speed');
  const rainL=zoneLayer(pts,'precip',0.2,'#4aa3e0',{rBase:9000,rScale:9000,oBase:.14,oScale:.10,every:8});
  const cloudL=zoneLayer(pts,'cloud',55,'#8a97a3',{rBase:12000,rScale:150,oBase:.08,oScale:.003,every:16});
  const dustL=zoneLayer(pts,'dust',80,'#c9a06a',{rBase:12000,rScale:20,oBase:.08,oScale:.0012,every:16});
  const glyphL=L.layerGroup();let lg=-1e9;
  pts.forEach(q=>{if(q.dist-lg>=Math.max(80,p.dist/28)){glyphL.addLayer(L.marker([q.lat,q.lon],{icon:L.divIcon({className:'wx',
    html:`<svg width=28 height=28 viewBox="0 0 28 28" style="filter:drop-shadow(0 0 3px #000)">${wxGlyph(pickKind(q))}</svg>`,iconSize:[28,28],iconAnchor:[14,32]})}));lg=q.dist;}});
  const hzL=L.layerGroup();
  (p.hazards||[]).forEach(h=>{hzL.addLayer(L.marker([h.lat,h.lon],{icon:hazIcon(h.type,h.sev)}).bindPopup(esc(h.label)+'<br><small>'+(h.source==='tomtom'?'live traffic (TomTom)':'OpenStreetMap')+'</small>'));});
  pvL.addTo(map);rainL.addTo(map);glyphL.addTo(map);hzL.addTo(map);
  profLayers=[pvL,windL,spdL,rainL,cloudL,dustL,glyphL,hzL];
  control=L.control.layers({[COLORINGS.pv.label]:pvL,[COLORINGS.wind.label]:windL,[COLORINGS.speed.label]:spdL},
    {'Roadworks / hazards':hzL,'Rain':rainL,'Cloud':cloudL,'Dust / haze':dustL,'Weather symbols':glyphL},{collapsed:false,position:'topright'}).addTo(map);
  legend=L.control({position:'bottomleft'});
  legend.onAdd=function(){this._d=L.DomUtil.create('div','legendbox');this.upd('pv');return this._d;};
  legend.upd=function(k){const c=COLORINGS[k];this._d.innerHTML='<div style="color:#8b98a6;margin-bottom:3px">'+c.label+'</div>'+c.legend.map(([t,col])=>`<div class="row"><span class="sw" style="background:${col}"></span>${t}</div>`).join('');};
  legend.addTo(map);
  const kf=l=>l===COLORINGS.pv.label?'pv':l===COLORINGS.wind.label?'wind':'speed';
  map.on('baselayerchange',e=>legend.upd(kf(e.name)));
}
function setMeta(p){const ts=p.updated?new Date(p.updated*1000).toLocaleTimeString():'—';const nh=(p.hazards||[]).length;
  document.getElementById('meta').innerHTML='live weather · updated '+ts+' · version '+p.version+' · '+nh+' roadworks/hazards along the route'+
    '<br><span style="color:#6f7d8a">Sources: '+(p.sources||[]).join(' · ')+'</span>';}
function showWarn(msg){const w=document.getElementById('warn');if(msg){w.style.display='block';w.textContent=msg;}else{w.style.display='none';}}

async function loadProfile(){
  let p;try{p=await(await fetch('/profile',{cache:'no-store'})).json();}catch(e){return setTimeout(loadProfile,1200);}
  if(!p.ready){
    document.getElementById('banner').textContent='downloading live weather data…';
    showWarn(p.weatherErr?('Weather unavailable: '+p.weatherErr+' — retrying…'):null);
    return setTimeout(loadProfile,1500);
  }
  showWarn(null);lastVersion=p.version;
  document.getElementById('banner').textContent=`${p.a?p.a.name:''} → ${p.b?p.b.name:''} · ${p.dist} km · ${p.cols.dist.length} points`;
  LAYOUT=buildLayout(p.a,p.b,p.dist);
  Plotly.newPlot('chart',buildTraces(p.cols),LAYOUT,{responsive:true,scrollZoom:true,displaylogo:false,modeBarButtonsToRemove:['lasso2d','select2d','autoScale2d']}).then(()=>{chartReady=true;});
  if(p.a){startM=L.marker([p.a.lat,p.a.lon],{icon:startIcon}).addTo(map).bindPopup('Start: '+esc(p.a.name));}
  if(p.b){destM=L.marker([p.b.lat,p.b.lon],{icon:destIcon}).addTo(map).bindPopup('Destination: '+esc(p.b.name));}
  buildProfLayers(p);map.fitBounds(L.latLngBounds(p.route).pad(0.12));fitted=true;profileLoaded=true;
  setMeta(p);
  document.getElementById('foot').textContent='Weather, elevation, and solar charging use blended live data (multiple averaged models), refreshed periodically; road route via OSRM. Alerts derive from forecast values. Power = PV + battery; 130 km/h cap.';
}
async function reloadProfile(){
  let p;try{p=await(await fetch('/profile',{cache:'no-store'})).json();}catch(e){return;}
  if(!p.ready)return;lastVersion=p.version;
  if(chartReady)Plotly.react('chart',buildTraces(p.cols),LAYOUT);
  removeProfLayers();buildProfLayers(p);setMeta(p);
}
function toast(text,sev){const box=document.getElementById('toasts');const d=document.createElement('div');
  d.className='toast';d.style.borderLeftColor=SEVCOL[sev]||'#8b98a6';
  d.innerHTML=`<div class="h" style="color:${SEVCOL[sev]||'#c7d2dc'}">Weather alert</div>${esc(text)}`;
  box.appendChild(d);setTimeout(()=>d.remove(),9000);}
function notify(text,sev){toast(text,sev);if('Notification' in window && Notification.permission==='granted' && sev>=2){try{new Notification('WSC weather alert',{body:text});}catch(e){}}}
function renderAlerts(alerts,km){const box=document.getElementById('allist');
  if(!alerts||!alerts.length){box.innerHTML='<span style="color:#8b98a6">no alerts</span>';return;}
  const cur=km||0,ahead=alerts.filter(a=>a.km1>=cur-5).sort((x,y)=>x.km0-y.km0).slice(0,7);
  if(!ahead.length){box.innerHTML='<span style="color:#8b98a6">no alerts ahead</span>';return;}
  box.innerHTML=ahead.map(a=>{const dahead=Math.max(0,a.km0-cur);
    return `<div class="al" style="border-left-color:${SEVCOL[a.sev]}"><div><b>${esc(a.label)}</b><br><small>km ${a.km0}–${a.km1} · ${dahead>0?('in '+dahead.toFixed(0)+' km'):'in progress'}</small></div></div>`;}).join('');}

async function pollState(){
  let s;try{s=await(await fetch('/state',{cache:'no-store'})).json();}catch(e){return;}
  if(profileLoaded && s.version!==lastVersion) reloadProfile();
  if(profileLoaded) showWarn(s.weatherOk?null:'Latest weather update failed — showing previous data and retrying…');
  if(s.events){if(!firstState){s.events.forEach(e=>seenEvents.add(e.id));firstState=true;}
    else{s.events.forEach(e=>{if(!seenEvents.has(e.id)){seenEvents.add(e.id);notify(e.text,e.sev);}});}}
  renderAlerts(s.alerts,s.km);
  const dot=document.getElementById('dot'),st=document.getElementById('stattxt');
  if(!s.veh){st.textContent='waiting for telemetry…';dot.className='';return;}
  const ll=[s.veh.lat,s.veh.lon];
  if(!vehM){vehM=L.marker(ll,{icon:vehIconMk(s.heading),zIndexOffset:1000}).addTo(map);}
  else{vehM.setLatLng(ll);const el=vehM.getElement();const v=el&&el.querySelector('.veh');if(v)v.style.transform='rotate('+s.heading+'deg)';}
  if(s.trail&&s.trail.length){if(!trailL){trailL=L.polyline(s.trail,{color:'#54d6e6',weight:3,opacity:.9}).addTo(map);}else trailL.setLatLngs(s.trail);}
  if(s.b){const r=[ll,[s.b.lat,s.b.lon]];if(!remL){remL=L.polyline(r,{color:'#fff',weight:1.5,dashArray:'3 7',opacity:.4}).addTo(map);}else remL.setLatLngs(r);}
  if(!fitted&&s.b){map.fitBounds(L.latLngBounds([ll,[s.b.lat,s.b.lon]]).pad(0.3));fitted=true;}
  document.getElementById('spd').textContent=s.speed!=null?s.speed.toFixed(0):'–';
  document.getElementById('hdg').textContent=s.heading!=null?s.heading.toFixed(0)+'°':'–';
  document.getElementById('pos').textContent=ll[0].toFixed(5)+', '+ll[1].toFixed(5);
  document.getElementById('prog').textContent=(s.km!=null?s.km.toFixed(0)+' km':'–')+' / '+(s.vPredHere!=null?s.vPredHere.toFixed(0)+' km/h':'–');
  document.getElementById('elev').textContent=s.elevHere!=null?s.elevHere.toFixed(0)+' m':'–';
  if(!liveInit){liveInit=true;initLive();}
  pushLive(s.speed,s.elevHere);
  if(s.b){document.getElementById('dist').textContent=hav(ll,[s.b.lat,s.b.lon]).toFixed(0)+' km to B';document.getElementById('dest').textContent=s.b.name||'';}
  if(s.connected){dot.className='on';st.textContent='connected · '+(s.age!=null?s.age+'s ago':'now')+(s.alongHere!=null?(' · '+(s.alongHere>0?'tailwind +':'headwind ')+s.alongHere.toFixed(1)+' m/s'):'');}
  else{dot.className='';st.textContent=(s.count>0?'no recent data ('+s.age+'s)':'waiting for telemetry…');}
  moveCursor(s.km);
}
loadProfile();setInterval(pollState,500);
</script>
</body></html>
"""


def main(argv=None):
    global NOMINAL_KMH, HAZARD_RADIUS, TOMTOM_KEY, REROUTE_ON, REROUTE_M, REROUTE_COOLDOWN
    config = parse_config(argv)
    NOMINAL_KMH = config.nominal_kmh
    HAZARD_RADIUS = config.hazard_radius
    TOMTOM_KEY = config.tomtom_key
    REROUTE_ON = config.reroute
    REROUTE_M = config.reroute_m
    REROUTE_COOLDOWN = config.reroute_cooldown
    if TOMTOM_KEY and "TomTom Traffic (incidenti/code live)" not in SOURCES:
        SOURCES.append("TomTom Traffic (incidenti/code live)")
    threading.Thread(target=udp_listener, args=(config.udp_port, config.udp_host), daemon=True).start()
    if config.adaptive:
        if config.debug_scheduler:
            import comm_scheduler as _cs
            _cs.DEBUG = True
        print(f"[main] adaptive scheduler: ON{' + debug' if config.debug_scheduler else ''} "
              f"({config.adaptive_period:.0f}s cycle). Scheduler logs appear after the first setup.", flush=True)
        threading.Thread(target=adaptive_updater, args=(config.adaptive_period,), daemon=True).start()
    else:
        print("[main] adaptive scheduler: OFF (periodic mode). "
              "Enable it with: --adaptive --debug-scheduler", flush=True)
        threading.Thread(target=weather_updater, args=(config.weather_period,), daemon=True).start()
        threading.Thread(target=hazard_updater, args=(config.hazard_period,), daemon=True).start()
    httpd = ThreadingHTTPServer((config.http_host, config.http_port), Handler)
    print(f"[http] map available at http://{config.http_host}:{config.http_port}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.shutdown()


if __name__ == "__main__":
    main()
