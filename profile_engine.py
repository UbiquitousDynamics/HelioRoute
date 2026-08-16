"""Per-kilometre route physics and weather-alert profile engine.

`build_profile` accepts either an external field provider or deterministic
synthetic fields for offline use. `detect_alerts` groups threshold crossings
into contiguous route alerts.
"""

from __future__ import annotations

from bisect import bisect_right
import math
from geo_utils import bearing_deg, haversine_km

G = 9.81
Z0, Z_REF, Z_CAR = 0.03, 10.0, 1.0
VEH = {"m": 250.0, "CdA": 0.10, "Crr": 0.006}
V_MAX = 130.0 / 3.6
A_PANEL, EFF_NOM, P_BATT = 4.0, 0.24, 2200.0


# ------------------------------ geometria ---------------------------------- #
def resample(points, step_km=1.0):
    """Sample a polyline at global, evenly spaced route distances.

    Cumulative distances are route distances, not straight-line distances
    between output samples. This keeps spacing correct when a sample interval
    crosses a bend and avoids the duplicate origin produced by the old carry
    algorithm.
    """
    if step_km <= 0:
        raise ValueError("step_km must be greater than zero")
    if not points:
        return [], [], [], []
    if len(points) == 1:
        return [points[0][0]], [points[0][1]], [0.0], [0.0]

    route_cum = [0.0]
    for i in range(len(points) - 1):
        route_cum.append(route_cum[-1] + haversine_km(*points[i], *points[i + 1]))
    total = route_cum[-1]
    if total <= 1e-9:
        return [points[0][0]], [points[0][1]], [0.0], [0.0]

    targets = [i * step_km for i in range(int(total / step_km) + 1)]
    if total - targets[-1] > 1e-9:
        targets.append(total)
    else:
        targets[-1] = total

    lats, lons = [], []
    seg_i = 0
    for target in targets:
        while seg_i + 1 < len(route_cum) - 1 and route_cum[seg_i + 1] < target:
            seg_i += 1
        while seg_i + 1 < len(route_cum) and route_cum[seg_i + 1] <= route_cum[seg_i]:
            seg_i += 1
        if seg_i + 1 >= len(points):
            la, lo = points[-1]
        else:
            span = route_cum[seg_i + 1] - route_cum[seg_i]
            f = 0.0 if span <= 0 else (target - route_cum[seg_i]) / span
            la1, lo1 = points[seg_i]; la2, lo2 = points[seg_i + 1]
            la = la1 + (la2 - la1) * f
            lo = lo1 + (lo2 - lo1) * f
        lats.append(la); lons.append(lo)
    cum = targets
    head = [bearing_deg(lats[i], lons[i], lats[i + 1], lons[i + 1])
            for i in range(len(lats) - 1)]
    head.append(head[-1] if head else 0.0)
    return lats, lons, cum, head


# --------------------------- fisica di supporto ---------------------------- #
def air_density(temp_c, press_hpa):
    return (press_hpa * 100.0) / (287.05 * (temp_c + 273.15))


def uv_to_speed_dir(u, v):
    return math.hypot(u, v), (math.degrees(math.atan2(-u, -v)) + 360.0) % 360.0


def dir_speed_to_uv(speed, direction_deg):
    r = math.radians(direction_deg)
    return -speed * math.sin(r), -speed * math.cos(r)


def scale_to_car(u, v):
    f = math.log(Z_CAR / Z0) / math.log(Z_REF / Z0)
    return u * f, v * f


def solve_speed(P, along, cross, grade, rho, v_max=V_MAX):
    CdA, m, Crr = VEH["CdA"], VEH["m"], VEH["Crr"]

    def Ptot(v):
        airspeed = math.hypot(v - along, cross)
        return (0.5 * rho * CdA * airspeed * (v - along) * v
                + Crr * m * G * v + m * G * grade * v)

    # At v=0 the residual is -P. For the physical parameter range used by the
    # model, available power crosses required power once before the speed cap.
    # A direct bracket removes the previous 200-point scan from every solve.
    if Ptot(v_max) <= P:
        return v_max
    a, b = 0.0, v_max
    for _ in range(45):
        mid = (a + b) / 2.0
        if Ptot(mid) > P:
            b = mid
        else:
            a = mid
    return (a + b) / 2.0


def realize(cum, v_target_ms, v_max=V_MAX, a_acc=0.5, a_dec=1.2, dt=1.0):
    n = len(cum)
    if n == 0:
        return [], []
    if len(v_target_ms) != n:
        raise ValueError("cum and v_target_ms must have the same length")
    total_m = cum[-1] * 1000.0
    vr = [0.0] * n; eta = [0.0] * n
    t = x = v = 0.0; k = 0
    while k < n - 1 and x < total_m:
        idx = min(bisect_right(cum, x / 1000.0) - 1, n - 1)
        vt = v_target_ms[idx]
        if v < vt: v = min(vt, v + a_acc * dt, v_max)
        else:      v = max(vt, v - a_dec * dt)
        v = max(v, 0.3)
        x += v * dt; t += dt
        while k + 1 < n and x >= cum[k + 1] * 1000.0:
            k += 1; vr[k] = v; eta[k] = t
    for j in range(1, n):
        if eta[j] == 0.0:
            vr[j] = v_target_ms[j]
            eta[j] = eta[j - 1] + (cum[j] - cum[j - 1]) * 1000.0 / max(vr[j], 1.0)
    return vr, eta


# --------------------- campi sintetici (solo test offline) ----------------- #
def _synth_fields(lats, lons, cum):
    n = len(cum)
    F = {k: [0.0] * n for k in ("wind_u", "wind_v", "temp", "press", "cloud",
                                "precip", "ghi", "gust", "elev", "dust")}
    for i in range(n):
        la, lo, c = lats[i], lons[i], cum[i]
        elev = max(0.0, 350 + 180 * math.sin(math.radians(la * 3)) +
                   120 * math.cos(math.radians(lo * 2.4)))
        t = 30 - 0.5 * (abs(la) - 12) - 0.0065 * elev
        F["elev"][i] = elev; F["temp"][i] = t
        F["press"][i] = 1013.25 * (1 - 2.25577e-5 * elev) ** 5.25588
        F["wind_u"][i] = 5 * math.sin(math.radians(la * 2.2))
        F["wind_v"][i] = -5 * math.cos(math.radians(la * 1.8))
        F["gust"][i] = math.hypot(F["wind_u"][i], F["wind_v"][i]) * 1.4
        cl = max(0, min(100, 40 + 40 * math.sin(c / 40.0)))
        F["cloud"][i] = cl
        F["precip"][i] = max(0.0, (cl - 70) * 0.3)
        F["ghi"][i] = max(0.0, (1000 - 4 * (abs(la) - 12)) * (1 - 0.8 * cl / 100))
        F["dust"][i] = 0.0
    return F


# ------------------------------- build ------------------------------------- #
def build_profile(route_latlon, step_km=1.0, provider=None):
    lats, lons, cum, head = resample(route_latlon, step_km)
    n = len(cum)
    if n < 2:
        raise ValueError("route must contain at least two distinct points")
    F = provider(lats, lons, cum) if provider else _synth_fields(lats, lons, cum)
    required = ("wind_u", "wind_v", "temp", "press", "cloud", "precip",
                "ghi", "gust", "elev", "dust")
    missing = [key for key in required if key not in F]
    if missing:
        raise ValueError(f"provider fields missing: {', '.join(missing)}")
    wrong_length = [key for key in required if len(F[key]) != n]
    if wrong_length:
        raise ValueError(f"provider fields have invalid length: {', '.join(wrong_length)}")
    elev = F["elev"]
    grade = []
    for i in range(n - 1):
        dm = (cum[i + 1] - cum[i]) * 1000.0
        grade.append(max(-0.06, min(0.06, (elev[i + 1] - elev[i]) / dm if dm > 0 else 0.0)))
    grade.append(grade[-1] if grade else 0.0)

    cols = {k: [] for k in ("dist", "lat", "lon", "elev", "grade", "vPred",
                            "vNoWind", "dSpeed", "along", "cross", "windSpeed",
                            "windDir", "etaH", "cloud", "precip", "dust", "ghi",
                            "pv", "temp", "gust")}
    vss = [0.0] * n
    for i in range(n):
        u, v = F["wind_u"][i], F["wind_v"][i]
        temp, press = F["temp"][i], F["press"][i]
        rho = air_density(temp, press)
        ghi = max(0.0, F["ghi"][i])
        t_cell = temp + 0.028 * ghi
        eff_t = max(0.6, 1 - 0.004 * (t_cell - 25))
        pv = A_PANEL * EFF_NOM * ghi * eff_t
        pav = pv + P_BATT
        u1, v1 = scale_to_car(u, v)
        h = math.radians(head[i])
        along = u1 * math.sin(h) + v1 * math.cos(h)
        cross = -u1 * math.cos(h) + v1 * math.sin(h)
        wspd, wdir = uv_to_speed_dir(u, v)
        vss[i] = solve_speed(pav, along, cross, grade[i], rho)
        vnw = solve_speed(pav, 0.0, 0.0, grade[i], rho)
        cols["along"].append(round(along, 2)); cols["cross"].append(round(cross, 2))
        cols["windSpeed"].append(round(wspd, 1)); cols["windDir"].append(int(round(wdir)))
        cols["gust"].append(round(F["gust"][i], 1))
        cols["cloud"].append(round(F["cloud"][i])); cols["precip"].append(round(F["precip"][i], 2))
        cols["dust"].append(round(F["dust"][i])); cols["ghi"].append(round(ghi))
        cols["pv"].append(round(pv)); cols["vNoWind"].append(round(vnw * 3.6, 1))
        cols["elev"].append(round(elev[i], 1)); cols["grade"].append(round(grade[i] * 100, 2))
        cols["lat"].append(round(lats[i], 5)); cols["lon"].append(round(lons[i], 5))
        cols["dist"].append(round(cum[i], 1)); cols["temp"].append(round(temp, 1))

    vr, eta = realize(cum, vss)
    for i in range(n):
        cols["vPred"].append(round(vr[i] * 3.6, 1))
        cols["etaH"].append(round(eta[i] / 3600.0, 2))
        cols["dSpeed"].append(round((vss[i] - cols["vNoWind"][i] / 3.6) * 3.6, 1))
    return cols, (cum[-1] if cum else 0.0)


# ------------------------------ allerte ------------------------------------ #
ALERT_LABELS = {
    "rain": "Rain / thunderstorm", "wind": "Strong wind (gusts)",
    "cross": "Crosswind", "dust": "Dust / haze",
    "pv": "Low solar charging", "heat": "Extreme heat",
}
ALERT_COLORS = {
    "rain": "#4aa3e0", "wind": "#f2c14e", "cross": "#f2c14e",
    "dust": "#c9a06a", "pv": "#8b98a6", "heat": "#ef6a5a",
}


def detect_alerts(cols):
    """Group threshold-based weather conditions into contiguous route alerts."""
    n = len(cols["dist"])
    if n == 0:
        return []

    def rain(i):
        p = cols["precip"][i]
        if p >= 4 and cols["gust"][i] >= 14: return 3   # temporale
        if p >= 4: return 2
        return 1 if p >= 1.0 else 0

    def wind(i):
        g = cols["gust"][i]
        return 3 if g >= 24 else (2 if g >= 17 else 0)     # ~61 / ~86 km/h

    def cross(i):
        c = abs(cols["cross"][i])
        return 2 if c >= 9 else (1 if c >= 6 else 0)

    def dust(i):
        d = cols["dust"][i]
        return 2 if d >= 350 else (1 if d >= 150 else 0)

    def pv(i):
        return 1 if cols["pv"][i] < 250 else 0

    def heat(i):
        t = cols["temp"][i]
        return 2 if t >= 45 else (1 if t >= 40 else 0)

    alerts = []
    for typ, fn in [("rain", rain), ("wind", wind), ("cross", cross),
                    ("dust", dust), ("pv", pv), ("heat", heat)]:
        i = 0
        while i < n:
            if fn(i) > 0:
                j = i; peak = 0
                while j < n and fn(j) > 0:
                    peak = max(peak, fn(j)); j += 1
                km0, km1 = cols["dist"][i], cols["dist"][j - 1]
                if (km1 - km0) >= 2 or peak >= 2:
                    mid = (i + j) // 2
                    alerts.append({"type": typ, "label": ALERT_LABELS[typ], "sev": peak,
                                   "km0": round(km0), "km1": round(km1),
                                   "lat": cols["lat"][mid], "lon": cols["lon"][mid],
                                   "color": ALERT_COLORS[typ]})
                i = j
            else:
                i += 1
    alerts.sort(key=lambda a: (a["km0"], -a["sev"]))
    return alerts


if __name__ == "__main__":
    route = [[-12.4634, 130.8456], [-14.4650, 132.2635]]
    cols, dist = build_profile(route)  # synth
    print("offline synthetic:", len(cols["dist"]), "points,", round(dist), "km, alerts:",
          len(detect_alerts(cols)))
