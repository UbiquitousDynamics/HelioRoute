"""Construction of fresh map-service state.

Keeping the schema in one place prevents production initialization and test
reset helpers from drifting apart. The dictionary shape remains compatible
with the current public module API while state ownership is migrated.
"""

from __future__ import annotations


def new_service_state():
    return {
        "a": None,
        "b": None,
        "route_geom": None,
        "route": [],
        "cols": None,
        "map": [],
        "dist": 0.0,
        "progress_km": 0.0,
        "ready": False,
        "version": 0,
        "route_fallback": False,
        "building": False,
        "building_revision": None,
        "route_revision": 0,
        "route_request_revision": 0,
        "offroute_count": 0,
        "last_reroute": 0.0,
        "last_fallback_try": 0.0,
        "weather_ok": False,
        "weather_err": None,
        "updated": 0.0,
        "weather_backoff_until": 0.0,
        "alerts": [],
        "events": [],
        "event_seq": 0,
        "hazards": [],
        "notified": [],
        "last_notify": 0.0,
        "veh": None,
        "speed": 0.0,
        "heading": 0.0,
        "t": 0.0,
        "count": 0,
        "trail": [],
        "link_cell": None,
        "link_wifi": None,
        "link_rtt": None,
    }
