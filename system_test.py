"""
system_test.py — end-to-end service system test.

Recreates the complete UDP/HTTP production flow deterministically in-process:
  1. A-to-B setup with mocked OSRM route, weather profile, and Overpass hazards;
  2. telemetry along the road through PathWalker while reading /state;
  3. route progress, road adherence, alerts, /profile, and /state checks;
  4. a weather change that publishes a new version and proximity event.

Usage: python system_test.py
Returns a boolean result for run_evaluation.py.
"""

from __future__ import annotations

import time

import mocks
import map_service as ms
import vehicle_sim as vs
import weather_providers as wx
from console_utils import safe_print

A = {"name": "Bologna", "lat": 44.4949, "lon": 11.3426}
B = {"name": "Modena", "lat": 44.6471, "lon": 10.9252}
GEOM = [[44.4949, 11.3426], [44.52, 11.28], [44.56, 11.18],
        [44.60, 11.05], [44.6471, 10.9252]]
HAZARDS = [
    {"type": "way", "center": {"lat": 44.56, "lon": 11.18},
     "tags": {"highway": "construction", "name": "SS9"}},
    {"type": "node", "lat": 44.60, "lon": 11.05, "tags": {"hazard": "falling_rocks"}},
]


def _dev_m(lat, lon, geom):
    import math
    lat0, lon0 = geom[0]

    def xy(la, lo):
        return ((lo - lon0) * 111320 * math.cos(math.radians(lat0)), (la - lat0) * 110540)
    p = xy(lat, lon); best = 1e18
    for i in range(len(geom) - 1):
        a = xy(*geom[i]); b = xy(*geom[i + 1])
        dx, dy = b[0] - a[0], b[1] - a[1]; L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
        best = min(best, math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy)))
    return best


def run_system_test(verbose=True):
    steps = []

    def check(name, cond, detail=""):
        steps.append((name, bool(cond), detail))
        if verbose:
            safe_print(f"  {'✔' if cond else '✗'} {name}" + (f" — {detail}" if detail else ""))
        return bool(cond)

    with mocks.osrm_mock(GEOM), mocks.weather_mock(), mocks.overpass_mock(HAZARDS):
        # --- 1) setup: rotta + profilo + cantieri (come i gestori UDP/HTTP) ---
        mocks.reset_service(GEOM)
        ms.G["a"] = A; ms.G["b"] = B
        ms.recompute()                     # profilo con meteo reale (mock)
        ms.fetch_hazards_and_store()       # cantieri/pericoli (mock) + re-merge
        prof = ms.profile_payload()

        check("profile ready", prof.get("ready") and prof["dist"] > 0,
              f"dist={prof['dist']} km, points={len(prof['cols']['dist'])}")
        check("route and map available", len(prof["route"]) >= 2 and len(prof["map"]) > 5)
        check("roadworks/hazards loaded", len(prof.get("hazards", [])) == 2)
        haz_types = {a["type"] for a in ms.G["alerts"]}
        check("alerts include roadwork and hazard",
              "roadwork" in haz_types and "hazard" in haz_types,
              f"alert types: {sorted(haz_types)}")
        v0 = ms.G["version"]

        # --- 2) telemetria lungo la STRADA + lettura /state ---
        walker = vs.PathWalker(GEOM)
        total = sum(walker.seg)
        kms, devs = [], []
        arrived = False
        for _ in range(400):
            lat, lon, hdg = walker.advance(total / 60.0)  # ~60 passi
            ms.handle_message({"type": "telem", "lat": lat, "lon": lon,
                               "speed": 92.0, "heading": hdg, "t": time.time()})
            s = ms.state_snapshot()
            if s["km"] is not None:
                kms.append(s["km"])
            devs.append(_dev_m(lat, lon, GEOM))
            if walker.done:
                arrived = True
                break

        check("telemetry connected and route progress advances", arrived and len(kms) > 5
              and kms[-1] > kms[0], f"km {kms[0]:.0f} -> {kms[-1]:.0f}")
        check("final km is close to route length", abs(kms[-1] - prof["dist"]) < 25,
              f"final {kms[-1]:.0f} vs {prof['dist']:.0f} km")
        check("vehicle follows the road", max(devs) < 30.0,
              f"maximum deviation {max(devs):.1f} m")
        s = ms.state_snapshot()
        check("/state is consistent (version, alerts, sources)",
              s["version"] == v0 and isinstance(s["alerts"], list) and s["sources"],
              f"version {s['version']}")

        # --- 3) aggiornamento meteo: nuova cella di pioggia -> nuova versione + evento ---
        # riporto il veicolo vicino alla partenza: la pioggia (a meta' rotta) sara' "davanti"
        # e dentro la finestra meteo, cosi' la notifica di prossimita' scatta
        ms.handle_message({"type": "telem", "lat": A["lat"], "lon": A["lon"],
                           "speed": 80.0, "heading": 0.0, "t": time.time()})
        ms.G["notified"] = []
        ev_before = len(ms.G["events"])
        old_precip = mocks.GT["precipitation"]
        mocks.GT["precipitation"] = lambda lat, lon: 6.0 if 44.54 < lat < 44.63 else 0.0
        wx._ELEV_CACHE.clear()
        try:
            ms.recompute()
        finally:
            mocks.GT["precipitation"] = old_precip
        check("weather update increments version", ms.G["version"] == v0 + 1,
              f"{v0} -> {ms.G['version']}")
        check("new rain alert detected",
              any(a["type"] == "rain" for a in ms.G["alerts"]))
        check("proximity notification event generated", len(ms.G["events"]) > ev_before,
              f"events {ev_before} -> {len(ms.G['events'])}")

    ok = all(c for _, c, _ in steps)
    return ok, steps


if __name__ == "__main__":
    safe_print("\nSYSTEM TEST — end-to-end\n" + "-" * 40)
    ok, steps = run_system_test()
    npass = sum(1 for _, c, _ in steps if c)
    safe_print("-" * 40)
    safe_print(f"RESULT: {'PASS' if ok else 'FAIL'} ({npass}/{len(steps)} checks)\n")
    raise SystemExit(0 if ok else 1)
