"""
test_units.py — unit test di tutti i moduli del sistema live.
Esecuzione diretta:  python -m unittest test_units -v
Girano tutti OFFLINE grazie ai mock in mocks.py.
"""

from __future__ import annotations

import math
import os
import tempfile
import unittest
import urllib.error

import mocks
import profile_engine as pe
import weather_providers as wx
import road_hazards as rh
import map_service as ms
import vehicle_sim as vs
import service_config


# ============================ profile_engine =============================== #
class TestGeoPhysics(unittest.TestCase):
    def test_haversine_known(self):
        d = pe.haversine_km(-12.4634, 130.8456, -34.9285, 138.6007)
        self.assertTrue(2600 < d < 2800, d)  # Darwin->Adelaide in linea d'aria

    def test_bearing_cardinals(self):
        self.assertAlmostEqual(pe.bearing_deg(0, 0, 1, 0), 0.0, delta=0.5)   # nord
        self.assertAlmostEqual(pe.bearing_deg(0, 0, 0, 1), 90.0, delta=0.5)  # est

    def test_resample_spacing_and_endpoints(self):
        la, lo, cum, head = pe.resample([[0, 0], [0, 1]], 1.0)
        self.assertGreater(len(cum), 100)
        self.assertEqual(cum, sorted(cum))                 # monotono
        self.assertAlmostEqual(la[-1], 0.0); self.assertAlmostEqual(lo[-1], 1.0)  # ultimo incluso
        steps = [cum[i + 1] - cum[i] for i in range(len(cum) - 2)]
        self.assertAlmostEqual(sum(steps) / len(steps), 1.0, delta=0.15)

    def test_resample_is_strictly_increasing_across_bends(self):
        _la, _lo, cum, _head = pe.resample(
            [[0.0, 0.0], [0.0, 0.03], [0.03, 0.03]], step_km=1.0)
        steps = [cum[i + 1] - cum[i] for i in range(len(cum) - 1)]
        self.assertTrue(all(step > 0 for step in steps), steps)
        self.assertTrue(all(abs(step - 1.0) < 1e-9 for step in steps[:-1]), steps)

    def test_resample_empty_and_invalid_step(self):
        self.assertEqual(pe.resample([]), ([], [], [], []))
        with self.assertRaises(ValueError):
            pe.resample([[0.0, 0.0], [0.0, 1.0]], step_km=0)

    def test_resample_dense_polyline_length(self):
        pts = [[0, 0], [0, 0.5], [0.3, 0.5]]
        la, lo, cum, head = pe.resample(pts, 1.0)
        exact = pe.haversine_km(0, 0, 0, 0.5) + pe.haversine_km(0, 0.5, 0.3, 0.5)
        self.assertAlmostEqual(cum[-1], exact, delta=2.0)

    def test_uv_dir_roundtrip(self):
        u, v = pe.dir_speed_to_uv(10.0, 270.0)
        spd, d = pe.uv_to_speed_dir(u, v)
        self.assertAlmostEqual(spd, 10.0, delta=1e-6)
        self.assertAlmostEqual(d, 270.0, delta=1e-6)

    def test_scale_to_car_reduces(self):
        u, v = pe.scale_to_car(10.0, 0.0)
        self.assertTrue(0 < u < 10.0)

    def test_air_density_sea_level(self):
        self.assertAlmostEqual(pe.air_density(15.0, 1013.25), 1.225, delta=0.03)

    def test_solve_speed_tailwind_faster(self):
        rho = 1.2
        tail = pe.solve_speed(1500, 5.0, 0.0, 0.0, rho)
        head = pe.solve_speed(1500, -5.0, 0.0, 0.0, rho)
        self.assertGreater(tail, head)

    def test_solve_speed_cap(self):
        self.assertLessEqual(pe.solve_speed(1e7, 0, 0, 0, 1.2), pe.V_MAX + 1e-6)

    def test_solve_speed_neutral_between(self):
        rho = 1.2
        tail = pe.solve_speed(1500, 5.0, 0.0, 0.0, rho)
        neu = pe.solve_speed(1500, 0.0, 0.0, 0.0, rho)
        head = pe.solve_speed(1500, -5.0, 0.0, 0.0, rho)
        self.assertTrue(head <= neu <= tail)

    def test_solve_speed_matches_scanning_reference(self):
        def reference(P, along, cross, grade, rho):
            CdA, mass, crr = pe.VEH["CdA"], pe.VEH["m"], pe.VEH["Crr"]

            def residual(v):
                airspeed = math.hypot(v - along, cross)
                required = (0.5 * rho * CdA * airspeed * (v - along) * v
                            + crr * mass * pe.G * v + mass * pe.G * grade * v)
                return required - P

            previous_v, previous_f = 0.0, residual(0.0)
            for i in range(1, 201):
                value = pe.V_MAX * i / 200
                current_f = residual(value)
                if previous_f <= 0 < current_f:
                    low, high = previous_v, value
                    for _ in range(40):
                        middle = (low + high) / 2
                        if residual(middle) > 0:
                            high = middle
                        else:
                            low = middle
                    return (low + high) / 2
                previous_v, previous_f = value, current_f
            return pe.V_MAX

        cases = [
            (500, -12, 8, 0.06, 1.25), (500, 5, 0, 0, 1.0),
            (1500, -5, 8, -0.06, 1.25), (1500, 12, 0, 0.06, 1.0),
            (4000, 0, 8, 0, 1.25), (4000, 12, 8, 0.06, 1.0),
        ]
        for case in cases:
            self.assertAlmostEqual(pe.solve_speed(*case), reference(*case), places=8)

    def test_realize_monotonic_eta(self):
        la, lo, cum, head = pe.resample([[0, 0], [0, 0.5]], 1.0)
        vt = [25.0] * len(cum)
        vr, eta = pe.realize(cum, vt)
        self.assertEqual(eta, sorted(eta))
        self.assertTrue(all(v <= 25.0 + 1e-6 for v in vr))

    def test_realize_uses_actual_sample_distances(self):
        _vr, eta = pe.realize([0.0, 2.5, 5.0], [5.0, 30.0, 30.0])
        self.assertGreater(eta[1], 450.0)

    def test_realize_validates_shapes(self):
        self.assertEqual(pe.realize([], []), ([], []))
        with self.assertRaisesRegex(ValueError, "same length"):
            pe.realize([0.0, 1.0], [10.0])


class TestBuildProfile(unittest.TestCase):
    def test_rejects_degenerate_route(self):
        with self.assertRaisesRegex(ValueError, "distinct points"):
            pe.build_profile([])
        with self.assertRaisesRegex(ValueError, "distinct points"):
            pe.build_profile([[44.0, 11.0], [44.0, 11.0]])

    def test_validates_provider_schema(self):
        with self.assertRaisesRegex(ValueError, "fields missing"):
            pe.build_profile([[44.0, 11.0], [44.1, 11.1]], provider=lambda *_: {})

        def wrong_length(lats, _lons, _cum):
            return {key: [] for key in ("wind_u", "wind_v", "temp", "press",
                                        "cloud", "precip", "ghi", "gust",
                                        "elev", "dust")}

        with self.assertRaisesRegex(ValueError, "invalid length"):
            pe.build_profile([[44.0, 11.0], [44.1, 11.1]], provider=wrong_length)

    def test_synth_shape_and_keys(self):
        cols, dist = pe.build_profile([[44.5, 11.3], [44.65, 10.9]], 1.0)
        self.assertGreater(dist, 0)
        n = len(cols["dist"])
        for k in ("vPred", "along", "cross", "pv", "cloud", "gust", "temp", "elev", "grade"):
            self.assertEqual(len(cols[k]), n, k)

    def test_provider_overrides_fields(self):
        def provider(lats, lons, cum):
            n = len(cum)
            return {"wind_u": [0.0] * n, "wind_v": [-8.0] * n, "temp": [30.0] * n,
                    "press": [1000.0] * n, "cloud": [10.0] * n, "precip": [0.0] * n,
                    "ghi": [900.0] * n, "gust": [12.0] * n, "elev": [50.0] * n,
                    "dust": [0.0] * n}
        cols, _ = pe.build_profile([[0, 0], [1.0, 0]], 1.0, provider=provider)
        self.assertTrue(all(t == 30.0 for t in cols["temp"]))
        self.assertTrue(all(g == 12.0 for g in cols["gust"]))


class TestDetectAlerts(unittest.TestCase):
    def _cols(self, n=60, mut=None):
        c = {k: [0.0] * n for k in ("dist", "lat", "lon", "precip", "gust",
                                    "cross", "dust", "pv", "temp")}
        for i in range(n):
            c["dist"][i] = float(i); c["lat"][i] = 44 + 0.001 * i; c["lon"][i] = 11.0
            c["pv"][i] = 800.0; c["temp"][i] = 25.0; c["gust"][i] = 8.0
        if mut:
            mut(c)
        return c

    def test_rain(self):
        def m(c):
            for i in range(10, 30):
                c["precip"][i] = 6.0; c["gust"][i] = 16.0
        al = [a for a in pe.detect_alerts(self._cols(mut=m)) if a["type"] == "rain"]
        self.assertEqual(len(al), 1); self.assertEqual(al[0]["sev"], 3)

    def test_wind_gust(self):
        def m(c):
            for i in range(20, 40):
                c["gust"][i] = 25.0
        al = [a for a in pe.detect_alerts(self._cols(mut=m)) if a["type"] == "wind"]
        self.assertEqual(len(al), 1); self.assertEqual(al[0]["sev"], 3)

    def test_crosswind(self):
        def m(c):
            for i in range(5, 20):
                c["cross"][i] = 10.0
        al = [a for a in pe.detect_alerts(self._cols(mut=m)) if a["type"] == "cross"]
        self.assertTrue(al and al[0]["sev"] == 2)

    def test_dust(self):
        def m(c):
            for i in range(30, 45):
                c["dust"][i] = 400.0
        al = [a for a in pe.detect_alerts(self._cols(mut=m)) if a["type"] == "dust"]
        self.assertTrue(al and al[0]["sev"] == 2)

    def test_pv_and_heat(self):
        def m(c):
            for i in range(0, 20):
                c["pv"][i] = 200.0
            for i in range(40, 55):
                c["temp"][i] = 46.0
        al = pe.detect_alerts(self._cols(mut=m))
        self.assertTrue(any(a["type"] == "pv" for a in al))
        self.assertTrue(any(a["type"] == "heat" and a["sev"] == 2 for a in al))

    def test_benign_empty(self):
        self.assertEqual(pe.detect_alerts(self._cols()), [])


# ============================ weather_providers ============================ #
class TestWeatherHelpers(unittest.TestCase):
    def test_pick_waypoints_bounds(self):
        cum = [float(i) for i in range(3000)]
        idx = wx._pick_waypoints(cum)
        self.assertLessEqual(len(idx), wx.MAX_WAYPOINTS)
        self.assertEqual(idx[0], 0); self.assertEqual(idx[-1], len(cum) - 1)

    def test_interp1d_linear_and_clamp(self):
        xs = [0.0, 10.0, 20.0]; ys = [0.0, 10.0, 20.0]
        self.assertAlmostEqual(wx._interp1d(xs, ys, 5.0), 5.0)
        self.assertEqual(wx._interp1d(xs, ys, -3.0), 0.0)   # clamp
        self.assertEqual(wx._interp1d(xs, ys, 99.0), 20.0)  # clamp

    def test_nearest_hour_index(self):
        from datetime import datetime, timezone
        times = mocks._times()
        tgt = wx._iso_to_epoch(times[3])
        self.assertEqual(wx._nearest_hour_index(times, tgt), 3)

    def test_hours_window_uses_remaining_distance(self):
        eta, _now, _start, _end = wx._hours_window(
            [990.0, 1000.0, 1050.0], 90.0, origin_km=1000.0)
        self.assertEqual(eta[0], 0.0)
        self.assertEqual(eta[1], 0.0)
        self.assertAlmostEqual(eta[2], 2000.0)

    def test_blend_point_mean_and_skip_null(self):
        h = len(mocks._times())
        hourly = {"temperature_2m_ecmwf_ifs025": [21.0] * h,
                  "temperature_2m_gfs_seamless": [19.0] * h,
                  "temperature_2m_icon_seamless": [None] * h,
                  "windspeed_10m_ecmwf_ifs025": [10.0] * h,
                  "winddirection_10m_ecmwf_ifs025": [270.0] * h}
        b = wx._blend_point(hourly, 0, wx.DEFAULT_MODELS)
        self.assertAlmostEqual(b["temp"], 20.0, delta=1e-6)  # media di 21 e 19 (icon None saltato)

    def test_get_json_maps_429(self):
        import weather_providers
        real = weather_providers.urllib.request.urlopen

        def boom(req, timeout=25):
            raise urllib.error.HTTPError(req.full_url, 429, "Too Many", {"Retry-After": "42"}, None)
        weather_providers.urllib.request.urlopen = boom
        try:
            with self.assertRaises(wx.RateLimitError) as cm:
                wx._get_json("https://api.open-meteo.com/v1/forecast?x=1")
            self.assertEqual(cm.exception.retry_after, 42)
        finally:
            weather_providers.urllib.request.urlopen = real

    def test_fetch_fields_shapes_and_blend(self):
        route = [[10, 0], [40, 0]]
        la, lo, cum, _ = pe.resample(route, 1.0)
        with mocks.weather_mock():
            F = wx.fetch_fields(la, lo, cum, 90.0)
        n = len(cum)
        for k in ("wind_u", "wind_v", "temp", "press", "cloud", "precip", "ghi", "gust", "elev", "dust"):
            self.assertEqual(len(F[k]), n, k)
        gt = mocks.gt_fields(la, lo)
        mae = sum(abs(F["temp"][i] - gt["temperature_2m"][i]) for i in range(n)) / n
        self.assertLess(mae, 0.05)  # blending (bias a somma zero) ~ esatto

    def test_elevation_cache(self):
        route = [[10, 0], [40, 0]]
        la, lo, cum, _ = pe.resample(route, 1.0)
        counters = {}
        with mocks.weather_mock(counters=counters):
            wx.fetch_fields(la, lo, cum, 90.0)
            wx.fetch_fields(la, lo, cum, 90.0)
        self.assertEqual(counters.get("elevation", 0), 1)  # quota scaricata una volta sola

    def test_forecast_429_no_fallback(self):
        route = [[10, 0], [40, 0]]
        la, lo, cum, _ = pe.resample(route, 1.0)
        with mocks.weather_raises(wx.RateLimitError(30)):
            with self.assertRaises(wx.RateLimitError):
                wx.fetch_fields(la, lo, cum, 90.0)

    def test_dust_points_surface_provider_failure(self):
        with mocks.weather_raises(OSError("down")):
            with self.assertRaisesRegex(RuntimeError, "dust data is unavailable"):
                wx.fetch_dust_points([44.0], [11.0], [0.0])

    def test_robust_to_null_models(self):
        route = [[10, 0], [40, 0]]
        la, lo, cum, _ = pe.resample(route, 1.0)
        with mocks.weather_mock(null_models={"ecmwf_ifs025", "gfs_seamless"}):
            F = wx.fetch_fields(la, lo, cum, 90.0)
        self.assertTrue(all(math.isfinite(x) for x in F["temp"]))  # nessun NaN


# ============================ road_hazards ================================= #
class TestHazards(unittest.TestCase):
    def test_build_query_has_clauses(self):
        q = rh._build_query([(44.5, 11.3)], 300)
        self.assertIn("construction", q); self.assertIn("hazard", q); self.assertIn("around:300", q)

    def test_parse_osm(self):
        els = [
            {"type": "way", "center": {"lat": 44.55, "lon": 11.15},
             "tags": {"highway": "construction", "name": "SS9"}},
            {"type": "node", "lat": 44.60, "lon": 11.05, "tags": {"hazard": "falling_rocks"}},
            {"type": "way", "center": {"lat": 44.55, "lon": 11.15},
             "tags": {"highway": "construction"}},  # duplicato -> dedup
        ]
        with mocks.overpass_mock(els):
            hz = rh.fetch_osm_hazards([44.5, 44.65], [11.3, 10.9], [0.0, 40.0])
        types = sorted(h["type"] for h in hz)
        self.assertEqual(types, ["hazard", "roadwork"])   # 2 elementi (dedup applicato)
        self.assertTrue(any("falling rocks" in h["label"] for h in hz))

    def test_overpass_counts_response_bytes(self):
        class Resp:
            def __enter__(self): return self
            def __exit__(self, *args): return False
            def read(self): return b'{"elements": []}'

        import road_hazards
        real = road_hazards.urllib.request.urlopen
        road_hazards.NET["bytes"] = 0
        road_hazards.urllib.request.urlopen = lambda *a, **k: Resp()
        try:
            self.assertEqual(road_hazards._overpass("query"), {"elements": []})
            self.assertEqual(road_hazards.NET["bytes"], len(b'{"elements": []}'))
        finally:
            road_hazards.urllib.request.urlopen = real

    def test_overpass_tries_all_endpoints(self):
        import road_hazards
        real = road_hazards.urllib.request.urlopen
        calls = {"n": 0}

        def boom(req, timeout=60):
            calls["n"] += 1
            raise OSError("down")
        road_hazards.urllib.request.urlopen = boom
        try:
            with self.assertRaises(RuntimeError):
                rh._overpass("[out:json];out;")
            self.assertEqual(calls["n"], len(rh.OVERPASS_ENDPOINTS))
        finally:
            road_hazards.urllib.request.urlopen = real

    def test_tomtom_disabled_without_key(self):
        self.assertEqual(rh.fetch_tomtom_incidents(0, 0, 1, 1, ""), [])

    def test_tomtom_strict_surfaces_failure(self):
        old = rh._get
        rh._get = lambda *a, **k: (_ for _ in ()).throw(OSError("down"))
        try:
            with self.assertRaisesRegex(RuntimeError, "TomTom is unavailable"):
                rh.fetch_tomtom_incidents(0, 0, 1, 1, "KEY", strict=True)
        finally:
            rh._get = old

    def test_tomtom_parse(self):
        canned = {"incidents": [
            {"geometry": {"type": "Point", "coordinates": [11.1, 44.5]},
             "properties": {"iconCategory": 1, "events": [{"description": "Accident"}]}},
            {"geometry": {"type": "LineString", "coordinates": [[11.2, 44.6], [11.3, 44.7]]},
             "properties": {"iconCategory": 6, "events": [{"description": "Coda"}]}},
        ]}
        import json as _json
        old = rh._get
        rh._get = lambda url, timeout=20: _json.dumps(canned)
        try:
            inc = rh.fetch_tomtom_incidents(11, 44, 12, 45, "KEY")
        finally:
            rh._get = old
        self.assertEqual(len(inc), 2)
        self.assertTrue(any(i["type"] == "accident" and i["sev"] == 3 for i in inc))
        self.assertTrue(any(i["type"] == "jam" for i in inc))


# ============================ map_service ================================== #
class TestMapService(unittest.TestCase):
    def setUp(self):
        mocks.reset_service([[44.4949, 11.3426], [44.6471, 10.9252]])

    def test_recompute_success(self):
        with mocks.weather_mock():
            ms.recompute()
        self.assertTrue(ms.G["ready"] and ms.G["weather_ok"])
        self.assertEqual(ms.G["version"], 1)

    def test_recompute_merges_hazards(self):
        ms.G["hazards"] = [{"type": "roadwork", "label": "Roadworks", "sev": 1,
                            "lat": 44.55, "lon": 11.13, "color": "#f2c14e"}]
        with mocks.weather_mock():
            ms.recompute()
        self.assertTrue(any(a["type"] == "roadwork" for a in ms.G["alerts"]))
        rw = [a for a in ms.G["alerts"] if a["type"] == "roadwork"][0]
        self.assertIsInstance(rw["km0"], int)

    def test_recompute_429_backoff(self):
        import time
        with mocks.weather_mock():
            ms.recompute()
        v = ms.G["version"]
        with mocks.weather_raises(wx.RateLimitError(30)):
            ms.recompute()
            self.assertGreater(ms.G["weather_backoff_until"], time.time())
            self.assertTrue(ms.G["weather_ok"])          # mantiene i dati precedenti
            self.assertEqual(ms.G["version"], v)         # nessun nuovo profilo
            calls = {"n": 0}
            orig = wx._get_json

            def counting(url, timeout=25):
                calls["n"] += 1
                return orig(url, timeout)
            wx._get_json = counting
            ms.recompute()                                # in backoff -> niente chiamate
            self.assertEqual(calls["n"], 0)

    def test_recompute_discards_stale_route_result(self):
        old_provider = ms.PROVIDER

        def provider(lats, lons, cum):
            with ms.LOCK:
                ms.G["route_revision"] += 1
            return pe._synth_fields(lats, lons, cum)

        ms.PROVIDER = provider
        try:
            ms.recompute()
        finally:
            ms.PROVIDER = old_provider
        self.assertEqual(ms.G["version"], 0)
        self.assertIsNone(ms.G["cols"])

    def test_obsolete_route_request_cannot_publish(self):
        original = list(ms.G["route_geom"])
        ms.G["route_request_revision"] = 2
        with mocks.osrm_mock([[10.0, 10.0], [11.0, 11.0]]):
            ms.build_route_and_profile(ms.G["a"], ms.G["b"], request_revision=1)
        self.assertEqual(ms.G["route_geom"], original)

    def test_old_route_worker_cannot_clear_new_building_flag(self):
        ms.G["route_request_revision"] = 1

        def superseded(_a, _b):
            with ms.LOCK:
                ms.G["route_request_revision"] = 2
                ms.G["building_revision"] = 2
                ms.G["building"] = True
            return [[10.0, 10.0], [11.0, 11.0]]

        old = ms.osrm_route
        ms.osrm_route = superseded
        try:
            ms.build_route_and_profile(ms.G["a"], ms.G["b"], request_revision=1)
        finally:
            ms.osrm_route = old
        self.assertTrue(ms.G["building"])
        self.assertEqual(ms.G["building_revision"], 2)

    def test_simplify_route(self):
        import math as _m
        geom = [[44.5 + 0.05 * _m.sin(_m.pi * k / 199), 11.0 + 0.05 * (1 - _m.cos(_m.pi * k / 199))]
                for k in range(200)]                       # arco denso e curvo
        simp = ms.simplify_route(geom, eps_m=10.0)
        self.assertLess(len(simp), len(geom))              # meno punti
        self.assertGreaterEqual(len(simp), 2)
        # ogni punto originale resta entro ~la tolleranza dalla polilinea semplificata
        def dev(p):
            return min(ms._route_point_dist_m(p[0], p[1], [simp[i], simp[i + 1]])
                       for i in range(len(simp) - 1))
        self.assertLess(max(dev(p) for p in geom), 12.0)   # segue la curva reale
        self.assertEqual(len(ms.simplify_route([[44.5, 11.0], [44.7, 11.3]])), 2)  # retta invariata

    def test_filter_on_route(self):
        geom = [[44.50 + 0.001 * i, 11.34] for i in range(101)]   # linea lungo il meridiano
        haz = [
            {"type": "roadwork", "lat": 44.55, "lon": 11.34, "label": "on"},    # sulla rotta
            {"type": "accident", "lat": 44.55, "lon": 11.40, "label": "off"},   # ~4.7 km a lato
        ]
        kept = ms._filter_on_route(haz, geom)
        labels = [h["label"] for h in kept]
        self.assertIn("on", labels)
        self.assertNotIn("off", labels)
        self.assertEqual(ms._filter_on_route(haz, None), [])   # senza rotta: nulla

    def test_notify_window_and_score(self):
        wx_win = ms._notify_window_km(ms.WX_CFG["cross"], 2, 90.0)
        pt_win = ms._notify_window_km(ms.POINT_CFG, 2, 90.0)
        self.assertGreater(wx_win, 20)      # vento trasversale: decine di km
        self.assertLess(pt_win, 3)          # ostacolo: ~1-2 km
        self.assertGreater(wx_win, pt_win * 8)

    def test_notify_nearby_proximity_gate(self):
        ms.G["events"] = []; ms.G["notified"] = []
        ms.G["alerts"] = [
            {"type": "roadwork", "label": "Roadworks", "sev": 1, "km0": 8, "km1": 8,
             "kind": "point", "lat": 0, "lon": 0},                          # ostacolo lontano (8 km)
            {"type": "rain", "label": "Rain", "sev": 2, "km0": 20, "km1": 30},
        ]
        ms._notify_nearby(veh_km=0.0, speed_kmh=90.0)
        texts = " ".join(e["text"] for e in ms.G["events"])
        self.assertIn("Rain", texts)
        self.assertNotIn("Roadworks", texts)
        self.assertTrue(all("score" in e for e in ms.G["events"]))

    def test_notify_nearby_dedup_and_pass(self):
        ms.G["events"] = []; ms.G["notified"] = []
        ms.G["alerts"] = [{"type": "rain", "label": "Rain", "sev": 2, "km0": 20, "km1": 30}]
        ms._notify_nearby(0.0, 90.0)
        n1 = len(ms.G["events"])
        ms._notify_nearby(1.0, 90.0)             # stessa allerta ancora vicina -> niente doppione
        self.assertEqual(len(ms.G["events"]), n1)
        ms._notify_nearby(60.0, 90.0)            # superata (oltre km1) -> evento "superata"
        self.assertTrue(any("passed" in e["text"] for e in ms.G["events"]))
        self.assertEqual(ms.G["notified"], [])

    def test_notify_nearby_point_close(self):
        ms.G["events"] = []; ms.G["notified"] = []
        ms.G["alerts"] = [{"type": "accident", "label": "Accident", "sev": 3, "km0": 1, "km1": 1,
                           "kind": "point", "lat": 0, "lon": 0}]
        ms._notify_nearby(0.3, 60.0)             # ostacolo a ~0.7 km -> notificato
        self.assertTrue(any("Accident" in e["text"] for e in ms.G["events"]))

    def test_event_output_survives_non_unicode_console(self):
        import io
        import console_utils
        old = console_utils.sys.stdout
        raw = io.BytesIO()
        console_utils.sys.stdout = io.TextIOWrapper(raw, encoding="cp1252", errors="strict")
        try:
            ms._add_event(2, "⚠ test")
            console_utils.sys.stdout.flush()
        finally:
            console_utils.sys.stdout.detach()
            console_utils.sys.stdout = old
        self.assertIn(b"test", raw.getvalue())

    def test_state_snapshot_progress(self):
        with mocks.weather_mock():
            ms.recompute()
        ms.handle_message({"type": "telem", "lat": 44.55, "lon": 11.2,
                           "speed": 92.0, "heading": 200.0, "t": __import__("time").time()})
        s = ms.state_snapshot()
        self.assertIsNotNone(s["km"]); self.assertIsNotNone(s["vPredHere"])
        self.assertTrue(s["connected"])

    def test_trail_cap(self):
        for i in range(ms.TRAIL_MAX + 500):
            ms.handle_message({"type": "telem", "lat": 44.5 + i * 1e-6, "lon": 11.0,
                               "speed": 90, "heading": 180, "t": 1.0})
        self.assertLessEqual(len(ms.G["trail"]), ms.TRAIL_MAX)

    def test_nearest_km(self):
        with mocks.weather_mock():
            ms.recompute()
        i = ms.nearest_km(ms.G["cols"]["lat"][10], ms.G["cols"]["lon"][10])
        self.assertEqual(i, 10)

    def test_osrm_fallback_returns_none(self):
        import map_service
        real = map_service.urllib.request.urlopen

        def boom(req, timeout=12):
            raise OSError("down")
        map_service.urllib.request.urlopen = boom
        try:
            self.assertIsNone(ms.osrm_route({"lat": 0, "lon": 0}, {"lat": 1, "lon": 1}))
        finally:
            map_service.urllib.request.urlopen = real

    def test_page_tokens(self):
        p = ms.PAGE
        for tok in ("zoomControl:false", "/state", "/profile", "hazIcon",
                    "Roadworks / hazards", "measured km/h", "function esc(value)"):
            self.assertIn(tok, p, tok)

    def test_message_validation_rejects_invalid_input(self):
        for message in (
            [],
            {"type": "unknown"},
            {"type": "setup", "a": {"lat": 95, "lon": 0}, "b": {"lat": 0, "lon": 0}},
            {"type": "telem", "lat": float("nan"), "lon": 0},
        ):
            with self.assertRaises(ValueError):
                ms.handle_message(message)

    def test_real_http_endpoints_and_exact_paths(self):
        import threading
        import urllib.request
        server = ms.ThreadingHTTPServer(("127.0.0.1", 0), ms.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_address[1]}"
        try:
            with urllib.request.urlopen(base + "/state?fresh=1", timeout=3) as response:
                payload = __import__("json").load(response)
            self.assertIn("version", payload)
            with self.assertRaises(urllib.error.HTTPError) as caught:
                urllib.request.urlopen(base + "/state-invalid", timeout=3)
            self.assertEqual(caught.exception.code, 404)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=3)


# ============================ configuration ================================ #
class TestServiceConfig(unittest.TestCase):
    def test_safe_defaults(self):
        cfg = service_config.parse_config([])
        self.assertEqual(cfg.http_host, "127.0.0.1")
        self.assertEqual(cfg.udp_host, "127.0.0.1")
        self.assertTrue(cfg.reroute)

    def test_flags_and_validation(self):
        cfg = service_config.parse_config([
            "--adaptive", "--no-reroute", "--http-host", "0.0.0.0"])
        self.assertTrue(cfg.adaptive)
        self.assertFalse(cfg.reroute)
        self.assertEqual(cfg.http_host, "0.0.0.0")
        with self.assertRaises(ValueError):
            service_config.parse_config(["--weather-period", "0"])


# ============================ vehicle_sim ================================== #
class TestVehicleSim(unittest.TestCase):
    def test_geocode_cache_save_is_atomic(self):
        real_path = vs._GEO_CACHE_PATH
        try:
            with tempfile.TemporaryDirectory() as directory:
                vs._GEO_CACHE_PATH = os.path.join(directory, "cache.json")
                self.assertTrue(vs._save_geo_cache({"bologna": [44.49, 11.34]}))
                self.assertEqual(vs._load_geo_cache(), {"bologna": [44.49, 11.34]})
                self.assertEqual(os.listdir(directory), ["cache.json"])
        finally:
            vs._GEO_CACHE_PATH = real_path

    def test_geocode_gazetteer_and_latlon(self):
        self.assertEqual(vs.geocode("Darwin"), vs.GAZ["darwin"])
        self.assertEqual(vs.geocode("-12.5, 130.8"), (-12.5, 130.8))

    def test_move_and_bearing(self):
        lat, lon = vs.move(0.0, 0.0, 90.0, 111.19)  # ~1 grado a est
        self.assertAlmostEqual(lat, 0.0, delta=1e-6)
        self.assertTrue(0.9 < lon < 1.1)

    def test_pathwalker_follows(self):
        geom = [[0, 0], [0, 0.5], [0.3, 0.5]]
        w = vs.PathWalker(geom)
        tot = sum(w.seg)
        lat, lon, hdg = w.advance(tot * 0.5)
        self.assertFalse(w.done)
        d = vs.haversine_km(0, 0, lat, lon)
        self.assertTrue(0 < d < tot)
        lat2, lon2, _ = w.advance(tot)  # oltre la fine
        self.assertTrue(w.done)
        self.assertAlmostEqual(lat2, geom[-1][0], delta=1e-9)

    def test_fetch_route_polls(self):
        import vehicle_sim
        import io
        real = vehicle_sim.urllib.request.urlopen

        class R(io.BytesIO):
            def __enter__(self): return self
            def __exit__(self, *a): return False

        def ok(url, timeout=5):
            return R(b'{"ready": true, "geom": [[1.0, 2.0], [3.0, 4.0]]}')
        vehicle_sim.urllib.request.urlopen = ok
        try:
            geom = vs.fetch_route("http://x", tries=1, delay=0)
            self.assertEqual(geom, [[1.0, 2.0], [3.0, 4.0]])
        finally:
            vehicle_sim.urllib.request.urlopen = real


import comm_scheduler as cs


# ============================ comm_scheduler =============================== #
class TestCommScheduler(unittest.TestCase):
    def _ctx(self, car_km=300.0, total=2600.0, tomtom=True, period=20.0):
        return {"route_total_km": total, "car_km": car_km, "kmh": 90.0,
                "period": period, "tomtom": tomtom, "_est": cs.SizeEstimator()}

    def test_link_score_monotonic(self):
        lo = cs.LinkMonitor(); lo.sample_wifi(-88, 6); lo.sample_cell("3g", -110)
        hi = cs.LinkMonitor(); hi.sample_wifi(-50, 200); hi.sample_cell("5g", -80)
        self.assertLess(lo.link_score(), hi.link_score())

    def test_is_up_no_cell(self):
        m = cs.LinkMonitor(); m.sample_wifi(-55, 120); m.sample_cell("none", None)
        self.assertFalse(m.is_up())          # WiFi ottimo ma niente cella -> giu'
        m2 = cs.LinkMonitor(); m2.sample_wifi(-55, 120); m2.sample_cell("4g", -95)
        m2.note_fetch(200_000, 1.0, True)
        self.assertTrue(m2.is_up())

    def test_budget_scales_with_goodput(self):
        slow = cs.LinkMonitor(); slow.note_fetch(5_000, 1.0, True)
        fast = cs.LinkMonitor(); fast.note_fetch(500_000, 1.0, True)
        self.assertLess(slow.budget_bytes(20), fast.budget_bytes(20))

    def test_urgency(self):
        c = {"min_int": 180, "ttl": 600}
        self.assertEqual(cs.urgency(None, c), 1.0)      # mai scaricato
        self.assertEqual(cs.urgency(60, c), 0.0)        # troppo fresco
        self.assertEqual(cs.urgency(600, c), 1.0)       # scaduto
        self.assertTrue(0 < cs.urgency(390, c) < 1)     # a meta' vita

    def test_size_rich_gt_lite(self):
        est = cs.SizeEstimator(); ctx = self._ctx()
        wa = next(c for c in cs.CLASSES if c["id"] == "weather_ahead")
        rich = est.predict(wa["variants"][0], ctx)
        lite = est.predict(wa["variants"][-1], ctx)
        self.assertGreater(rich, lite)

    def test_size_estimator_learns(self):
        est = cs.SizeEstimator(); ctx = self._ctx()
        v = next(c for c in cs.CLASSES if c["id"] == "weather_full")["variants"][0]
        before = est.predict(v, ctx)
        est.learn(v, before * 3, ctx)           # il payload reale e' molto piu' grande
        self.assertGreater(est.predict(v, ctx), before)

    def test_plan_offline_empty(self):
        m = cs.LinkMonitor(); m.sample_cell("none", None)   # is_up False
        self.assertEqual(cs.plan(1000.0, {}, m, self._ctx(), cs.SizeEstimator()), [])

    def test_plan_degrades_and_keeps_critical(self):
        est = cs.SizeEstimator(); ctx = self._ctx()
        # link ottimo -> il meteo intera-rotta e' nella variante piu' ricca
        good = cs.LinkMonitor(); good.note_fetch(600_000, 1.0, True)
        pg = cs.plan(1e9, {}, good, ctx, est)
        full = next((j for j in pg if j["class"] == "weather_full"), None)
        self.assertIsNotNone(full); self.assertEqual(full["variant"], "full_rich")
        # link scarso -> weather_full degradato o rimandato, ma il tratto davanti resta
        weak = cs.LinkMonitor(); weak.note_fetch(2_000, 1.0, True); weak.sample_rtt(700, 0.12)
        pw = cs.plan(1e9, {}, weak, ctx, est)
        self.assertIn("weather_ahead", {j["class"] for j in pw})   # critico presente
        fw = next((j for j in pw if j["class"] == "weather_full"), None)
        if fw is not None:
            self.assertNotEqual(fw["variant"], "full_rich")        # non la piu' ricca

    def test_adaptive_cycle_aborts_on_failure(self):
        est = cs.SizeEstimator(); ctx = self._ctx()
        m = cs.LinkMonitor(); m.note_fetch(50_000, 1.0, True)
        calls = {"n": 0}

        def failing(spec, c):
            calls["n"] += 1
            return (0, 1.0, False, None)     # sempre fallito
        execs = {cid: failing for cid in ("weather_ahead", "weather_full",
                                          "hazards", "dust", "traffic")}
        ages = {}
        rep = cs.adaptive_cycle(m, ages, ctx, execs, est)
        self.assertEqual(calls["n"], 1)      # si ferma al primo fallimento
        self.assertEqual(ages, {})           # nessuna eta' aggiornata

    def test_adaptive_cycle_accepts_typed_fetch_result(self):
        from service_models import FetchResult
        monitor = cs.LinkMonitor()
        monitor.note_fetch(200_000, 1.0, True)
        ctx = self._ctx()
        executors = {
            c["id"]: (lambda spec, context: FetchResult(1200, 0.1, True))
            for c in cs.CLASSES
        }
        report = cs.adaptive_cycle(monitor, {}, ctx, executors, cs.SizeEstimator())
        self.assertTrue(report)
        self.assertTrue(all(item["ok"] and item["error"] is None for item in report))

    def test_fieldstore_provider_builds_profile(self):
        geom = [[-12.46, 130.84], [-20.0, 134.0], [-34.93, 138.60]]
        store = cs.FieldStore(geom, kmh=90.0)
        with mocks.weather_mock():
            store.ensure_elevation()
            allwp = list(range(len(store.wp)))
            v = next(c for c in cs.CLASSES if c["id"] == "weather_full")["variants"][0]
            store.update_weather(allwp, v["models"], v["vars"], v["store_fields"])
        cols, dist = pe.build_profile(geom, 1.0, provider=store.provider)
        self.assertGreater(dist, 2000)
        for k in ("vPred", "windSpeed", "ghi", "pv", "elev"):
            self.assertEqual(len(cols[k]), len(cols["dist"]))
        # detect_alerts gira senza errori sui campi fusi
        self.assertIsInstance(pe.detect_alerts(cols), list)

    def test_fieldstore_partial_update_full_length(self):
        geom = [[-12.46, 130.84], [-25.0, 133.0], [-34.93, 138.60]]
        store = cs.FieldStore(geom, kmh=90.0)
        with mocks.weather_mock():
            sel = store.car_wp_window(car_km=100.0, horizon=150)  # solo tratto davanti
            self.assertTrue(0 < len(sel) < len(store.wp))
            v = next(c for c in cs.CLASSES if c["id"] == "weather_ahead")["variants"][0]
            store.update_weather(sel, v["models"], v["vars"], v["store_fields"])
        F = store.provider(store.lats, store.lons, store.cum)
        for k in ("wind_u", "ghi", "temp", "elev"):
            self.assertEqual(len(F[k]), len(store.cum))   # provider sempre a lunghezza piena

    def test_fieldstore_honors_variant_waypoint_limit(self):
        store = cs.FieldStore([[0.0, 0.0], [0.0, 10.0]], 90.0)
        selected = store.car_wp_window(0.0, 0.0, max_points=12)
        self.assertEqual(len(selected), 12)
        self.assertEqual(selected[0], 0)
        self.assertEqual(selected[-1], len(store.wp) - 1)


class TestReroute(unittest.TestCase):
    def setUp(self):
        self.geom = [[44.50 + 0.001 * i, 11.34] for i in range(101)]  # ~11 km lungo il meridiano
        ms.G.update({"route_geom": self.geom, "route": self.geom,
                     "b": {"name": "B", "lat": 44.60, "lon": 11.34},
                     "veh": {"name": "auto", "lat": 44.55, "lon": 11.34},
                     "building": False, "route_fallback": False,
                     "offroute_count": 0, "last_reroute": 0.0, "last_fallback_try": 0.0})
        ms.REROUTE_ON = True; ms.REROUTE_M = 120.0
        ms.REROUTE_MIN_SAMPLES = 3; ms.REROUTE_COOLDOWN = 30.0

    def tearDown(self):
        ms.REROUTE_ON = False           # non lasciare l'auto-reroute attivo per altri test
        ms.G.update({"route_fallback": False, "building": False, "offroute_count": 0,
                     "last_reroute": 0.0, "last_fallback_try": 0.0})

    def _launches(self, fn):
        import types
        orig = ms.threading
        n = {"c": 0}

        class _FakeThread:
            def __init__(self, *a, **k): n["c"] += 1
            def start(self): pass
        ms.threading = types.SimpleNamespace(Thread=_FakeThread)
        try:
            fn()
        finally:
            ms.threading = orig
        return n["c"]

    def test_dist_to_route_perpendicular(self):
        self.assertLess(ms._dist_to_route_m(44.55, 11.34), 1.0)        # sulla rotta
        self.assertGreater(ms._dist_to_route_m(44.55, 11.342), 120.0)  # ~160 m di lato

    def test_offroute_debounce(self):
        self.assertEqual(self._launches(lambda: [ms.maybe_reroute(44.55, 11.342) for _ in range(2)]), 0)
        self.assertEqual(ms.G["offroute_count"], 2)
        self.assertEqual(self._launches(lambda: ms.maybe_reroute(44.55, 11.342)), 1)  # 3° -> scatta

    def test_onroute_resets_counter(self):
        ms.G["offroute_count"] = 2
        self._launches(lambda: ms.maybe_reroute(44.55, 11.34))
        self.assertEqual(ms.G["offroute_count"], 0)

    def test_fallback_retry(self):
        ms.G.update({"route_fallback": True, "last_fallback_try": 0.0})
        self.assertEqual(self._launches(lambda: ms.maybe_reroute(44.55, 11.34)), 1)

    def test_disabled(self):
        ms.REROUTE_ON = False
        try:
            self.assertEqual(self._launches(lambda: ms.maybe_reroute(44.55, 12.0)), 0)
        finally:
            ms.REROUTE_ON = True


if __name__ == "__main__":
    unittest.main(verbosity=2)
