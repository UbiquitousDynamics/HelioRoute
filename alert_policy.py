"""Pure alert proximity and priority policy."""

from __future__ import annotations

POINT_CFG = {"W": 7.0, "base_km": 1.0, "km_sev": 0.5, "lead_min": 0.6, "min_sev": 0.3}
WX_CFG = {
    "wind": {"W": 9.0, "base_km": 30, "km_sev": 10, "lead_min": 18, "min_sev": 6},
    "cross": {"W": 9.0, "base_km": 30, "km_sev": 10, "lead_min": 18, "min_sev": 6},
    "rain": {"W": 8.0, "base_km": 25, "km_sev": 8, "lead_min": 15, "min_sev": 5},
    "dust": {"W": 8.0, "base_km": 30, "km_sev": 10, "lead_min": 18, "min_sev": 6},
    "pv": {"W": 6.0, "base_km": 20, "km_sev": 6, "lead_min": 12, "min_sev": 4},
    "heat": {"W": 6.0, "base_km": 20, "km_sev": 6, "lead_min": 12, "min_sev": 4},
}
WX_DEFAULT = {"W": 7.0, "base_km": 22, "km_sev": 7, "lead_min": 14, "min_sev": 5}


def notify_config(alert):
    if alert.get("kind") == "point":
        return POINT_CFG
    return WX_CFG.get(alert["type"], WX_DEFAULT)


def notify_window_km(config, severity, speed_kmh):
    by_distance = config["base_km"] + config["km_sev"] * (severity - 1)
    by_time = speed_kmh * (config["lead_min"] + config["min_sev"] * (severity - 1)) / 60.0
    return max(by_distance, by_time)


def nearby_alerts(alerts, vehicle_km, speed_kmh):
    speed = max(float(speed_kmh or 0.0), 8.0)
    nearby = []
    for alert in alerts or []:
        severity = alert["sev"]
        config = notify_config(alert)
        if vehicle_km > alert["km1"] + 0.2:
            continue
        inside = alert["km0"] - 0.2 <= vehicle_km <= alert["km1"]
        distance = 0.0 if inside else max(0.0, alert["km0"] - vehicle_km)
        window = notify_window_km(config, severity, speed)
        if distance <= window:
            urgency = max(0.15, min(1.0, 1 - distance / max(window, 0.1)))
            score = round(config["W"] * (1 + 0.6 * (severity - 1)) * urgency, 1)
            nearby.append({
                "type": alert["type"],
                "label": alert["label"],
                "sev": severity,
                "km0": alert["km0"],
                "km1": alert["km1"],
                "dist": distance,
                "score": score,
            })
    nearby.sort(key=lambda item: item["score"], reverse=True)
    return nearby

