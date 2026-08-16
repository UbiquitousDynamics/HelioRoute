"""Measure deterministic offline performance indicators and compare KPI targets."""

from __future__ import annotations

import math
import time
import unittest

import mocks
import map_service as ms
import profile_engine as pe
import vehicle_sim as vs
import weather_providers as wx

LONG = [[-12.4634, 130.8456], [-34.9285, 138.6007]]   # ~2600 km dopo resample
STRAIGHT = [[10.0, 0.0], [40.0, 0.0]]                  # meridiano: GT lineare in cum

# ---- definizione KPI: (id, nome, categoria, unita', operatore, target, chiave) ----
KPI_SPECS = [
    ("K01", "Unit tests passed", "Correctness", "%", "ge", 100.0, "unit_pct"),
    ("K02", "Alerts — F1", "Correctness", "ratio", "ge", 0.90, "f1"),
    ("K03", "Alerts — precision", "Correctness", "ratio", "ge", 0.90, "prec"),
    ("K04", "Alerts — recall", "Correctness", "ratio", "ge", 0.90, "rec"),
    ("K05", "Temperature MAE (blend)", "Accuracy", "°C", "le", 0.20, "temp_mae"),
    ("K06", "Wind MAE (blend)", "Accuracy", "m/s", "le", 0.20, "wind_mae"),
    ("K07", "Blend gain over best model", "Accuracy", "x", "ge", 2.0, "blend_improve"),
    ("K08", "build_profile P95 (2600 km)", "Latency", "ms", "le", 1500.0, "build_p95"),
    ("K09", "detect_alerts P95", "Latency", "ms", "le", 200.0, "alert_p95"),
    ("K10", "/state P95", "Latency", "ms", "le", 50.0, "state_p95"),
    ("K11", "recompute E2E P95 (150ms network)", "Latency", "ms", "le", 3000.0, "e2e_p95"),
    ("K12", "Telemetry throughput", "Throughput", "msg/s", "ge", 1000.0, "throughput"),
    ("K13", "External calls per refresh", "Efficiency", "n", "le", 3.0, "calls_refresh"),
    ("K14", "Elevation calls over 3 refreshes", "Efficiency", "n", "le", 1.0, "elev_calls"),
    ("K15", "Correct 429 backoff", "Robustness", "bool", "ge", 1.0, "ok_429"),
    ("K16", "Missing-model resilience", "Robustness", "bool", "ge", 1.0, "ok_null"),
    ("K17", "OSRM error fallback", "Robustness", "bool", "ge", 1.0, "ok_osrm"),
    ("K18", "Bounded trail memory", "Robustness", "bool", "ge", 1.0, "ok_trail"),
    ("K19", "Vehicle deviation from road", "Fidelity", "m", "le", 5.0, "pathdev"),
    ("K20", "Speed model (monotonicity + cap)", "Fidelity", "bool", "ge", 1.0, "ok_speed"),
]


def pctl(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * p
    f = int(k)
    return xs[f] if f == k else xs[f] + (xs[f + 1] - xs[f]) * (k - f)


def _times_ms(fn, reps):
    out = []
    for _ in range(reps):
        t = time.perf_counter(); fn(); out.append((time.perf_counter() - t) * 1000.0)
    return out


# ------------------------------ misure ------------------------------------- #
def measure_unit_tests():
    suite = unittest.TestLoader().loadTestsFromName("test_units")
    res = unittest.TestResult()
    suite.run(res)
    total = res.testsRun or 1
    failed = len(res.failures) + len(res.errors)
    return {"unit_pct": 100.0 * (total - failed) / total,
            "unit_total": total, "unit_failed": failed}


def measure_blend():
    la, lo, cum, _ = pe.resample(STRAIGHT, 1.0)
    with mocks.weather_mock():
        F = wx.fetch_fields(la, lo, cum, 90.0)
    gt = mocks.gt_fields(la, lo)
    n = len(cum)
    temp_mae = sum(abs(F["temp"][i] - gt["temperature_2m"][i]) for i in range(n)) / n
    wind_mae = sum(abs(math.hypot(F["wind_u"][i], F["wind_v"][i]) - gt["windspeed_10m"][i])
                   for i in range(n)) / n
    best = mocks.best_single_mae("temperature_2m")
    improve = best / max(temp_mae, 1e-9)
    return {"temp_mae": temp_mae, "wind_mae": wind_mae, "blend_improve": improve,
            "best_single_temp_mae": best}


def _scen(n, mut):
    c = {k: [0.0] * n for k in ("dist", "lat", "lon", "precip", "gust",
                                "cross", "dust", "pv", "temp")}
    for i in range(n):
        c["dist"][i] = float(i); c["lat"][i] = 44 + 0.001 * i; c["lon"][i] = 11.0
        c["pv"][i] = 800.0; c["temp"][i] = 25.0; c["gust"][i] = 8.0
    mut(c)
    return c


def measure_alert_quality():
    def rng(c, key, a, b, val):
        for i in range(a, b + 1):
            c[key][i] = val

    scenarios = []
    scenarios.append((_scen(400, lambda c: (rng(c, "precip", 100, 140, 6.0),
                                            rng(c, "gust", 100, 140, 16.0))),
                      [("rain", 100, 140)]))
    scenarios.append((_scen(200, lambda c: rng(c, "gust", 50, 80, 25.0)),
                      [("wind", 50, 80)]))
    scenarios.append((_scen(400, lambda c: rng(c, "cross", 300, 320, 10.0)),
                      [("cross", 300, 320)]))
    scenarios.append((_scen(300, lambda c: rng(c, "dust", 200, 230, 400.0)),
                      [("dust", 200, 230)]))
    scenarios.append((_scen(100, lambda c: (rng(c, "precip", 10, 30, 5.0),
                                            rng(c, "pv", 10, 30, 200.0))),
                      [("rain", 10, 30), ("pv", 10, 30)]))
    scenarios.append((_scen(80, lambda c: rng(c, "temp", 40, 55, 46.0)),
                      [("heat", 40, 55)]))
    scenarios.append((_scen(60, lambda c: None), []))  # benigno

    def overlap(a0, a1, b0, b1):
        return a0 <= b1 and b0 <= a1

    tp = fp = fn = 0
    for cols, expected in scenarios:
        pred = pe.detect_alerts(cols)
        used = [False] * len(expected)
        for p in pred:
            hit = False
            for i, (et, e0, e1) in enumerate(expected):
                if not used[i] and p["type"] == et and overlap(p["km0"], p["km1"], e0, e1):
                    used[i] = True; hit = True; break
            tp += 1 if hit else 0
            fp += 0 if hit else 1
        fn += used.count(False)
    prec = tp / (tp + fp) if (tp + fp) else 1.0
    rec = tp / (tp + fn) if (tp + fn) else 1.0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    return {"prec": prec, "rec": rec, "f1": f1}


def measure_latencies(long_cols):
    build_p95 = pctl(_times_ms(lambda: pe.build_profile(LONG, 1.0), 15), 0.95)
    alert_p95 = pctl(_times_ms(lambda: pe.detect_alerts(long_cols), 50), 0.95)
    # /state
    mocks.reset_service(LONG)
    ms.G["cols"] = long_cols
    ms.G["veh"] = {"name": "v", "lat": long_cols["lat"][len(long_cols["lat"]) // 2],
                   "lon": long_cols["lon"][len(long_cols["lat"]) // 2]}
    ms.G["count"] = 1; ms.G["t"] = time.time()
    state_p95 = pctl(_times_ms(ms.state_snapshot, 200), 0.95)
    # recompute end-to-end con latenza di rete simulata
    mocks.reset_service(LONG)
    with mocks.weather_mock(latency=0.15):
        e2e_p95 = pctl(_times_ms(ms.recompute, 5), 0.95)
    return {"build_p95": build_p95, "alert_p95": alert_p95,
            "state_p95": state_p95, "e2e_p95": e2e_p95}


def measure_throughput():
    mocks.reset_service(LONG)
    n = 5000
    t = time.perf_counter()
    for i in range(n):
        ms.handle_message({"type": "telem", "lat": -20 + i * 1e-6, "lon": 134.0,
                           "speed": 90.0, "heading": 180.0, "t": 1.0})
    dt = time.perf_counter() - t
    return {"throughput": n / dt if dt > 0 else float("inf")}


def measure_efficiency():
    la, lo, cum, _ = pe.resample(STRAIGHT, 1.0)
    counters = {}
    with mocks.weather_mock(counters=counters):
        wx.fetch_fields(la, lo, cum, 90.0)
    calls_refresh = sum(counters.get(k, 0) for k in ("forecast", "elevation", "airq"))
    counters2 = {}
    with mocks.weather_mock(counters=counters2):
        for _ in range(3):
            wx.fetch_fields(la, lo, cum, 90.0)
    return {"calls_refresh": calls_refresh, "elev_calls": counters2.get("elevation", 0)}


def measure_robustness():
    # 429 backoff
    ok_429 = 0.0
    try:
        mocks.reset_service(STRAIGHT)
        with mocks.weather_mock():
            ms.recompute()
        v = ms.G["version"]; had = ms.G["weather_ok"]
        with mocks.weather_raises(wx.RateLimitError(30)):
            ms.recompute()
            back = ms.G["weather_backoff_until"] > time.time()
            same = ms.G["version"] == v
            kept = ms.G["weather_ok"] == had
            calls = {"n": 0}
            orig = wx._get_json
            wx._get_json = lambda u, timeout=25: (calls.__setitem__("n", calls["n"] + 1), orig(u))[1]
            ms.recompute()
            noflood = calls["n"] == 0
            wx._get_json = orig
        ok_429 = 1.0 if (back and same and kept and noflood) else 0.0
    except Exception:
        ok_429 = 0.0

    # resilienza a modelli mancanti
    ok_null = 0.0
    try:
        la, lo, cum, _ = pe.resample(STRAIGHT, 1.0)
        with mocks.weather_mock(null_models={"ecmwf_ifs025", "gfs_seamless"}):
            F = wx.fetch_fields(la, lo, cum, 90.0)
        ok_null = 1.0 if all(math.isfinite(x) for x in F["temp"] + F["wind_u"]) else 0.0
    except Exception:
        ok_null = 0.0

    # fallback OSRM
    ok_osrm = 0.0
    try:
        import map_service
        real = map_service.urllib.request.urlopen

        def boom(req, timeout=12):
            raise OSError("down")
        map_service.urllib.request.urlopen = boom
        try:
            ok_osrm = 1.0 if ms.osrm_route({"lat": 0, "lon": 0}, {"lat": 1, "lon": 1}) is None else 0.0
        finally:
            map_service.urllib.request.urlopen = real
    except Exception:
        ok_osrm = 0.0

    # scia limitata
    ok_trail = 0.0
    try:
        mocks.reset_service(STRAIGHT)
        for i in range(ms.TRAIL_MAX + 800):
            ms.handle_message({"type": "telem", "lat": -20 + i * 1e-6, "lon": 134.0,
                               "speed": 90, "heading": 180, "t": 1.0})
        ok_trail = 1.0 if len(ms.G["trail"]) <= ms.TRAIL_MAX else 0.0
    except Exception:
        ok_trail = 0.0

    return {"ok_429": ok_429, "ok_null": ok_null, "ok_osrm": ok_osrm, "ok_trail": ok_trail}


def _to_xy(lat, lon, lat0, lon0):
    return ((lon - lon0) * 111320.0 * math.cos(math.radians(lat0)),
            (lat - lat0) * 110540.0)


def _pt_seg_m(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def measure_fidelity():
    # PathWalker: deviazione massima dalla polilinea
    geom = [[0.0, 0.0], [0.0, 0.5], [0.3, 0.5], [0.3, 1.0]]
    lat0, lon0 = geom[0]
    poly = [_to_xy(la, lo, lat0, lon0) for la, lo in geom]
    w = vs.PathWalker(geom)
    tot = sum(w.seg)
    steps = int(tot) + 1
    maxdev = 0.0
    for _ in range(steps):
        la, lo, _h = w.advance(1.0)
        p = _to_xy(la, lo, lat0, lon0)
        d = min(_pt_seg_m(p, poly[i], poly[i + 1]) for i in range(len(poly) - 1))
        maxdev = max(maxdev, d)
        if w.done:
            break

    # sanita' fisica del modello velocita'
    rho = 1.2
    tail = pe.solve_speed(1500, 5.0, 0.0, 0.0, rho)
    neu = pe.solve_speed(1500, 0.0, 0.0, 0.0, rho)
    head = pe.solve_speed(1500, -5.0, 0.0, 0.0, rho)
    cap = pe.solve_speed(1e7, 0.0, 0.0, 0.0, rho)
    ok_speed = 1.0 if (tail > neu > head and cap <= pe.V_MAX + 1e-6) else 0.0
    return {"pathdev": maxdev, "ok_speed": ok_speed}


def compute_metrics():
    M = {}
    long_cols, _ = pe.build_profile(LONG, 1.0)
    for fn in (measure_unit_tests, measure_blend, measure_alert_quality,
               measure_throughput, measure_efficiency, measure_robustness,
               measure_fidelity):
        try:
            M.update(fn())
        except Exception as exc:  # una misura fallita non blocca il report
            print(f"[kpi] measurement {fn.__name__} failed: {exc}", flush=True)
    try:
        M.update(measure_latencies(long_cols))
    except Exception as exc:
        print(f"[kpi] latency measurements failed: {exc}", flush=True)
    return M


def _cmp(op, value, target):
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return False
    return value <= target if op == "le" else value >= target


def run():
    M = compute_metrics()
    results = []
    for kid, name, cat, unit, op, target, key in KPI_SPECS:
        val = M.get(key)
        results.append({"id": kid, "name": name, "category": cat, "unit": unit,
                        "op": op, "target": target, "value": val,
                        "status": "PASS" if _cmp(op, val, target) else
                                  ("ERR" if val is None else "FAIL")})
    return results, M


if __name__ == "__main__":
    res, M = run()
    for r in res:
        v = "n/d" if r["value"] is None else f"{r['value']:.3f}"
        print(f"{r['id']} {r['status']:4} {r['name']:38} = {v} {r['unit']} "
              f"(target {r['op']} {r['target']})")
