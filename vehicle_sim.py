"""
vehicle_sim.py
==============
Simulates a vehicle travelling from A to B and sends data to map_service.py.

The vehicle follows a real road. After setup, it requests the OSRM road geometry
from GET /route and moves along that polyline. If the route is unavailable, it
falls back to a straight line from A to B.

Communication:
  - setup and per-second telemetry -> UDP (WGS84 position, speed, heading);
  - road geometry -> HTTP GET /route.

At runtime, enter a number to change cruise speed, `dest <city>` to change the
destination, or `q` to quit.

Esempi:
  python vehicle_sim.py "Darwin" "Adelaide" --speed 95
  python vehicle_sim.py "Alice Springs" "Coober Pedy" --speed 110 --host 127.0.0.1 --udp-port 9999 --http-port 8000
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import os
import random
import re
import socket
import sys
import threading
import tempfile
import time
import urllib.parse
import urllib.request

from geo_utils import R_EARTH_KM, bearing_deg as bearing, haversine_km

GAZ = {
    "darwin": (-12.4634, 130.8456), "katherine": (-14.4650, 132.2635),
    "daly waters": (-16.2530, 133.3700), "tennant creek": (-19.6497, 134.1897),
    "alice springs": (-23.6980, 133.8807), "kulgera": (-25.8419, 133.3020),
    "coober pedy": (-29.0135, 134.7544), "glendambo": (-30.9736, 135.7500),
    "port augusta": (-32.4930, 137.7648), "adelaide": (-34.9285, 138.6007),
    "sydney": (-33.8688, 151.2093), "melbourne": (-37.8136, 144.9631),
    "brisbane": (-27.4698, 153.0251), "perth": (-31.9523, 115.8613),
    # --- Italia (per test/uso in patria; risoluzione offline immediata) ---
    "bologna": (44.4949, 11.3426), "bari": (41.1171, 16.8719),
    "modena": (44.6471, 10.9252), "milano": (45.4642, 9.1900),
    "roma": (41.9028, 12.4964), "firenze": (43.7696, 11.2558),
    "napoli": (40.8518, 14.2681), "torino": (45.0703, 7.6869),
    "venezia": (45.4408, 12.3155), "genova": (44.4056, 8.9463),
    "palermo": (38.1157, 13.3615), "ancona": (43.6158, 13.5189),
    "pescara": (42.4643, 14.2142), "rimini": (44.0678, 12.5695),
    "ravenna": (44.4184, 12.2035), "ferrara": (44.8381, 11.6198),
    "padova": (45.4064, 11.8768), "verona": (45.4384, 10.9916),
    "bolzano": (46.4983, 11.3548), "trieste": (45.6495, 13.7768),
    "perugia": (43.1107, 12.3908), "pisa": (43.7228, 10.4017),
    "cagliari": (39.2238, 9.1217), "catania": (37.5079, 15.0830),
    "parma": (44.8015, 10.3279), "reggio emilia": (44.6989, 10.6297),
    "brescia": (45.5416, 10.2118), "bergamo": (45.6983, 9.6773),
}
_LATLON = re.compile(r"^\s*(-?\d+(?:\.\d+)?)\s*,\s*(-?\d+(?:\.\d+)?)\s*$")


_GEO_CACHE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "geocode_cache.json")


def _load_geo_cache():
    try:
        with open(_GEO_CACHE_PATH, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return {}


def _save_geo_cache(cache):
    temp_path = None
    try:
        directory = os.path.dirname(_GEO_CACHE_PATH)
        with tempfile.NamedTemporaryFile(
                "w", encoding="utf-8", dir=directory, delete=False) as f:
            temp_path = f.name
            json.dump(cache, f, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp_path, _GEO_CACHE_PATH)
        return True
    except Exception as exc:
        logging.getLogger(__name__).warning("unable to save geocode cache: %s", exc)
        if temp_path:
            try:
                os.unlink(temp_path)
            except OSError:
                pass
        return False


def geocode(name):
    m = _LATLON.match(name)
    if m:
        return float(m.group(1)), float(m.group(2))
    key = name.strip().lower()
    if key in GAZ:                       # gazetteer interno (sempre offline)
        return GAZ[key]
    cache = _load_geo_cache()
    if key in cache:                     # gia' risolta in passato (offline OK)
        return tuple(cache[key])
    url = "https://nominatim.openstreetmap.org/search?" + urllib.parse.urlencode(
        {"q": name, "format": "json", "limit": 1})
    req = urllib.request.Request(url, headers={"User-Agent": "wsc-vehicle-sim/1.0"})
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.load(resp)
        if data:
            latlon = (float(data[0]["lat"]), float(data[0]["lon"]))
            cache[key] = list(latlon); _save_geo_cache(cache)   # memorizza per l'offline
            return latlon
    except Exception as exc:
        print(f"[geocode] Nominatim non raggiungibile per '{name}': {exc}", flush=True)
    raise SystemExit(
        f"Unable to geocode '{name}' offline.\n"
        f"  - Use direct coordinates: \"lat,lon\" (for example \"44.4949,11.3426\")\n"
        f"  - Or resolve the name once with internet access; it will be saved in\n"
        f"    {os.path.basename(_GEO_CACHE_PATH)} and will then work offline.")


def move(lat, lon, brg, dist_km):
    d = dist_km / R_EARTH_KM
    b = math.radians(brg); p1 = math.radians(lat); l1 = math.radians(lon)
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(b))
    l2 = l1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), ((math.degrees(l2) + 540) % 360) - 180


def send(sock, addr, obj):
    sock.sendto(json.dumps(obj).encode("utf-8"), addr)


def fetch_route(base_url, tries=20, delay=1.0):
    """Poll the service for ready road geometry and return it, or None."""
    for _ in range(tries):
        try:
            with urllib.request.urlopen(base_url + "/route", timeout=5) as r:
                j = json.load(r)
            if j.get("ready") and j.get("geom"):
                return [[float(a), float(b)] for a, b in j["geom"]]
        except Exception:
            pass
        time.sleep(delay)
    return None


def stdin_loop(ctl):
    for line in sys.stdin:
        s = line.strip()
        if not s:
            continue
        if s.lower() in ("q", "quit", "exit"):
            ctl["stop"] = True; break
        if s.lower().startswith("dest "):
            try:
                lat, lon = geocode(s[5:].strip())
                ctl["new_dest"] = (s[5:].strip(), lat, lon)
                print(f"[input] new destination: {s[5:].strip()} -> {lat:.4f},{lon:.4f}", flush=True)
            except SystemExit as e:
                print(e, flush=True)
            continue
        try:
            ctl["cruise"] = max(0.0, float(s))
            print(f"[input] new cruise speed: {ctl['cruise']:.0f} km/h", flush=True)
        except ValueError:
            print("[input] command? (number, 'dest <city>', or 'q')", flush=True)


# ---- movimento lungo una polilinea (strada) ---- #
class PathWalker:
    def __init__(self, geom):
        self.g = geom
        self.seg = [haversine_km(*geom[i], *geom[i + 1]) for i in range(len(geom) - 1)]
        self.i = 0          # indice segmento
        self.into = 0.0     # km percorsi nel segmento
        self.done = False

    def advance(self, dist_km):
        while dist_km > 0 and self.i < len(self.seg):
            rem = self.seg[self.i] - self.into
            if dist_km < rem:
                self.into += dist_km; dist_km = 0
            else:
                dist_km -= rem; self.i += 1; self.into = 0.0
        if self.i >= len(self.seg):
            self.done = True
            return self.g[-1][0], self.g[-1][1], self._last_heading()
        a = self.g[self.i]; b = self.g[self.i + 1]
        f = (self.into / self.seg[self.i]) if self.seg[self.i] > 0 else 0.0
        lat = a[0] + (b[0] - a[0]) * f; lon = a[1] + (b[1] - a[1]) * f
        return lat, lon, bearing(a[0], a[1], b[0], b[1])

    def _last_heading(self):
        a = self.g[-2]; b = self.g[-1]
        return bearing(a[0], a[1], b[0], b[1])


def main():
    ap = argparse.ArgumentParser(description="UDP vehicle simulator that follows a road route.")
    ap.add_argument("city_a"); ap.add_argument("city_b")
    ap.add_argument("--speed", type=float, default=95.0)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--udp-port", type=int, default=9999)
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--rate", type=float, default=1.0)
    ap.add_argument("--accel", type=float, default=8.0)
    ap.add_argument("--duration", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--no-road", action="store_true", help="force a straight line from A to B")
    ap.add_argument("--fake-link", action="store_true",
                    help="send simulated network samples (Wi-Fi RSSI/bitrate, cellular type/signal, RTT) "
                         "to demonstrate the adaptive scheduler without a phone companion")
    ap.add_argument("--fake-link-period", type=float, default=5.0,
                    help="seconds between simulated network samples")
    args = ap.parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    la, lo = geocode(args.city_a); lb, lob = geocode(args.city_b)
    print(f"[geocode] A '{args.city_a}' -> {la:.5f},{lo:.5f}", flush=True)
    print(f"[geocode] B '{args.city_b}' -> {lb:.5f},{lob:.5f}", flush=True)

    addr = (args.host, args.udp_port)
    base = f"http://{args.host}:{args.http_port}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    a_name, b_name = args.city_a, args.city_b
    send(sock, addr, {"type": "setup",
                      "a": {"name": a_name, "lat": la, "lon": lo},
                      "b": {"name": b_name, "lat": lb, "lon": lob}})

    if args.fake_link:
        def _fake_link_loop():
            # cicla su condizioni di rete plausibili (dal 4G pieno al deserto) cosi'
            # nei log dello scheduler si vedono variare rssi/rtt/cella/goodput
            scen = [
                {"cell_type": "4g", "cell_dbm": -95, "rssi": -58, "mbps": 72, "rtt_ms": 90, "loss": 0.01},
                {"cell_type": "3g", "cell_dbm": -108, "rssi": -80, "mbps": 18, "rtt_ms": 260, "loss": 0.04},
                {"cell_type": "edge", "cell_dbm": -112, "rssi": -70, "mbps": 8, "rtt_ms": 650, "loss": 0.10},
                {"cell_type": "none", "cell_dbm": None, "rssi": -60, "mbps": 65, "rtt_ms": None, "loss": 0.0},
            ]
            i = 0
            while True:
                s = dict(scen[i % len(scen)]); s["type"] = "link"
                send(sock, addr, s)
                print(f"[fake-link] sent: cellular={s['cell_type']} rssi={s['rssi']}dBm "
                      f"bitrate={s['mbps']}Mbps rtt={s['rtt_ms']}ms", flush=True)
                i += 1
                time.sleep(args.fake_link_period)
        threading.Thread(target=_fake_link_loop, daemon=True).start()
    print(f"[udp] setup sent to {args.host}:{args.udp_port}", flush=True)

    # geometria stradale dal servizio
    geom = None
    if not args.no_road:
        print("[route] requesting road geometry from the service…", flush=True)
        geom = fetch_route(base)
        if geom:
            print(f"[route] received {len(geom)} points — the vehicle will follow the road", flush=True)
        else:
            print("[route] unavailable — falling back to a straight line from A to B", flush=True)
    if not geom:
        geom = [[la, lo], [lb, lob]]
    walker = PathWalker(geom)

    ctl = {"cruise": args.speed, "new_dest": None, "stop": False}
    threading.Thread(target=stdin_loop, args=(ctl,), daemon=True).start()

    v = 0.0; dt = args.rate; t0 = time.time()
    cur_lat, cur_lon = la, lo
    print("[sim] started (number=speed, 'dest <city>'=destination, 'q'=stop)", flush=True)

    while True:
        if ctl["stop"]:
            print("[sim] stop", flush=True); break
        if ctl["new_dest"]:
            b_name, lb, lob = ctl["new_dest"]; ctl["new_dest"] = None
            send(sock, addr, {"type": "setup",
                              "a": {"name": a_name, "lat": cur_lat, "lon": cur_lon},
                              "b": {"name": b_name, "lat": lb, "lon": lob}})
            new_geom = None if args.no_road else fetch_route(base, tries=25, delay=0.8)
            geom = new_geom or [[cur_lat, cur_lon], [lb, lob]]
            walker = PathWalker(geom)
            print(f"[route] route updated toward {b_name}", flush=True)

        # aggiorna velocita': rampa + rumore
        target = ctl["cruise"]
        if v < target: v = min(target, v + args.accel * dt)
        else: v = max(target, v - 1.5 * args.accel * dt)
        v_rep = max(0.0, v + random.uniform(-2.0, 2.0))

        step_km = v_rep * (dt / 3600.0)
        cur_lat, cur_lon, hdg = walker.advance(step_km)
        dist_to_b = haversine_km(cur_lat, cur_lon, lb, lob)

        send(sock, addr, {"type": "telem", "lat": cur_lat, "lon": cur_lon,
                          "speed": v_rep, "heading": hdg, "t": time.time()})
        print(f"[telem] {cur_lat:8.5f},{cur_lon:9.5f}  v={v_rep:5.1f}  hdg={hdg:5.1f}  d->B={dist_to_b:7.1f} km",
              flush=True)

        if walker.done or dist_to_b < 0.2:
            send(sock, addr, {"type": "telem", "lat": lb, "lon": lob, "speed": 0.0,
                              "heading": hdg, "t": time.time()})
            print("[sim] destination reached", flush=True); break
        if args.duration and (time.time() - t0) >= args.duration:
            print("[sim] duration reached", flush=True); break
        time.sleep(dt)


if __name__ == "__main__":
    main()
