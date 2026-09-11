"""Command-line entry point for the complete mission."""
import sys
from .mission import run

try:
    raise SystemExit(run())
except KeyboardInterrupt:
    raise SystemExit(130)
except Exception as error:
    print(f'[ERROR] {type(error).__name__}: {error}', file=sys.stderr, flush=True)
    raise SystemExit(3)
