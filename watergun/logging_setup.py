"""Rotating file + console logging. Also exposes an in-memory ring buffer for the web UI."""
import logging
from logging.handlers import RotatingFileHandler
from collections import deque
import os

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOG_DIR = os.path.join(BASE_DIR, "logs")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, "watergun.log")

_ring = deque(maxlen=500)  # recent lines for web UI


class RingHandler(logging.Handler):
    def emit(self, record):
        _ring.append(self.format(record))


def setup(max_bytes=1_048_576, backup_count=5):
    root = logging.getLogger()
    root.handlers.clear()
    root.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s", "%Y-%m-%d %H:%M:%S")

    fh = RotatingFileHandler(LOG_FILE, maxBytes=max_bytes, backupCount=backup_count)
    fh.setFormatter(fmt)
    root.addHandler(fh)

    sh = logging.StreamHandler()
    sh.setFormatter(fmt)
    root.addHandler(sh)

    rh = RingHandler()
    rh.setFormatter(fmt)
    root.addHandler(rh)
    return root


def recent(n=200):
    return list(_ring)[-n:]
