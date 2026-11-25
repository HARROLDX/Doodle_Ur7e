"""Keep ROS alive while our own goal is canceled on SIGINT/SIGTERM."""
from contextlib import contextmanager
import signal

_requested = False
_cleaning = False


def check_stop():
    if _requested and not _cleaning:
        raise KeyboardInterrupt


@contextmanager
def stop_signals():
    global _requested
    _requested = False
    def request_stop(signum, frame):
        global _requested
        _requested = True
    previous = {s: signal.signal(s, request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


@contextmanager
def cancellation_cleanup():
    global _cleaning
    _cleaning = True
    try:
        yield
    finally:
        _cleaning = False
