"""Shared geospatial primitives for routes and vehicle motion."""

from __future__ import annotations

import math

R_EARTH_KM = 6371.0088


def haversine_km(a_lat, a_lon, b_lat, b_lon):
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dp = math.radians(b_lat - a_lat)
    dl = math.radians(b_lon - a_lon)
    value = (math.sin(dp / 2) ** 2
             + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * R_EARTH_KM * math.asin(math.sqrt(value))


def bearing_deg(a_lat, a_lon, b_lat, b_lon):
    p1, p2 = math.radians(a_lat), math.radians(b_lat)
    dl = math.radians(b_lon - a_lon)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def segment_distance_m(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return math.hypot(px - ax, py - ay)
    fraction = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    fraction = max(0.0, min(1.0, fraction))
    return math.hypot(px - (ax + fraction * dx), py - (ay + fraction * dy))


def _rdp(points, eps_m):
    if len(points) < 3:
        return points[:]
    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        if end <= start + 1:
            continue
        lat0 = points[start][0]
        meters_lat = 111320.0
        meters_lon = 111320.0 * math.cos(math.radians(lat0))
        ax = points[start][1] * meters_lon
        ay = points[start][0] * meters_lat
        bx = points[end][1] * meters_lon
        by = points[end][0] * meters_lat
        max_distance, max_index = -1.0, -1
        for index in range(start + 1, end):
            px = points[index][1] * meters_lon
            py = points[index][0] * meters_lat
            distance = segment_distance_m(px, py, ax, ay, bx, by)
            if distance > max_distance:
                max_distance, max_index = distance, index
        if max_distance > eps_m and max_index != -1:
            keep[max_index] = True
            stack.append((start, max_index))
            stack.append((max_index, end))
    return [point for index, point in enumerate(points) if keep[index]]


def simplify_route(geometry, eps_m=10.0, max_points=6000):
    output = _rdp(geometry, eps_m)
    while len(output) > max_points and eps_m < 200:
        eps_m *= 2.0
        output = _rdp(geometry, eps_m)
    return output


def route_point_distance_m(lat, lon, geometry):
    if not geometry or len(geometry) < 2:
        return None
    meters_lat = 111320.0
    meters_lon = 111320.0 * math.cos(math.radians(lat))
    ax = (geometry[0][1] - lon) * meters_lon
    ay = (geometry[0][0] - lat) * meters_lat
    best = float("inf")
    for point in geometry[1:]:
        bx = (point[1] - lon) * meters_lon
        by = (point[0] - lat) * meters_lat
        best = min(best, segment_distance_m(0.0, 0.0, ax, ay, bx, by))
        ax, ay = bx, by
    return best

