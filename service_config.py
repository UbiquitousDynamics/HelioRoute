"""Command-line configuration for the live HelioRoute service."""

from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class ServiceConfig:
    http_host: str = "127.0.0.1"
    http_port: int = 8000
    udp_host: str = "127.0.0.1"
    udp_port: int = 9999
    weather_period: float = 900.0
    hazard_period: float = 900.0
    hazard_radius: int = 300
    tomtom_key: str | None = None
    nominal_kmh: float = 90.0
    adaptive: bool = False
    adaptive_period: float = 20.0
    debug_scheduler: bool = False
    reroute_m: float = 120.0
    reroute_cooldown: float = 30.0
    reroute: bool = True


def build_parser():
    parser = argparse.ArgumentParser(
        description="Live map service with blended weather and alerts.")
    parser.add_argument("--http-host", default="127.0.0.1",
                        help="HTTP bind address; use 0.0.0.0 only on a trusted network")
    parser.add_argument("--http-port", type=int, default=8000)
    parser.add_argument("--udp-host", default="127.0.0.1",
                        help="UDP bind address; use 0.0.0.0 only on a trusted network")
    parser.add_argument("--udp-port", type=int, default=9999)
    parser.add_argument("--weather-period", type=float, default=900.0,
                        help="seconds between weather updates (default: 15 minutes)")
    parser.add_argument("--hazard-period", type=float, default=900.0,
                        help="seconds between roadwork/hazard updates")
    parser.add_argument("--hazard-radius", type=int, default=300,
                        help="OSM hazard search radius around the route, in metres")
    parser.add_argument("--tomtom-key", default=None,
                        help="optional TomTom key for live traffic")
    parser.add_argument("--nominal-kmh", type=float, default=90.0,
                        help="speed used to estimate waypoint arrival times")
    parser.add_argument("--adaptive", action="store_true",
                        help="enable the adaptive network scheduler")
    parser.add_argument("--adaptive-period", type=float, default=20.0)
    parser.add_argument("--debug-scheduler", action="store_true")
    parser.add_argument("--reroute-m", type=float, default=120.0)
    parser.add_argument("--reroute-cooldown", type=float, default=30.0)
    parser.add_argument("--no-reroute", action="store_true")
    return parser


def parse_config(argv=None):
    args = build_parser().parse_args(argv)
    if args.http_port <= 0 or args.udp_port <= 0:
        raise ValueError("ports must be greater than zero")
    if args.weather_period <= 0 or args.hazard_period <= 0 or args.adaptive_period <= 0:
        raise ValueError("update periods must be greater than zero")
    if args.hazard_radius <= 0 or args.nominal_kmh <= 0:
        raise ValueError("hazard radius and nominal speed must be greater than zero")
    return ServiceConfig(
        http_host=args.http_host,
        http_port=args.http_port,
        udp_host=args.udp_host,
        udp_port=args.udp_port,
        weather_period=args.weather_period,
        hazard_period=args.hazard_period,
        hazard_radius=args.hazard_radius,
        tomtom_key=args.tomtom_key,
        nominal_kmh=args.nominal_kmh,
        adaptive=args.adaptive,
        adaptive_period=args.adaptive_period,
        debug_scheduler=args.debug_scheduler,
        reroute_m=args.reroute_m,
        reroute_cooldown=args.reroute_cooldown,
        reroute=not args.no_reroute,
    )
