"""Console output helpers that remain usable on legacy Windows encodings."""

from __future__ import annotations

import sys


def safe_print(*values, sep=" ", end="\n", file=None, flush=False):
    """Print values, replacing only characters unsupported by the stream.

    Python installations launched from some Windows hosts still expose a
    CP1252 stdout stream. Operational logging must not crash application logic
    merely because a status symbol cannot be encoded.
    """
    stream = file or sys.stdout
    try:
        print(*values, sep=sep, end=end, file=stream, flush=flush)
    except UnicodeEncodeError:
        text = sep.join(str(value) for value in values) + end
        encoding = getattr(stream, "encoding", None) or "ascii"
        stream.write(text.encode(encoding, errors="replace").decode(encoding))
        if flush:
            stream.flush()
