"""Typed boundary models shared by application and infrastructure code."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FetchResult:
    bytes_downloaded: int
    seconds: float
    ok: bool
    retry_after: int | None = None
    error: str | None = None

    @classmethod
    def coerce(cls, value):
        """Accept the legacy executor tuple while callers migrate incrementally."""
        if isinstance(value, cls):
            return value
        downloaded, seconds, ok, retry_after = value
        return cls(int(downloaded), float(seconds), bool(ok), retry_after)

