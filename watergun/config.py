"""Config loader/saver — single source of truth for runtime-tunable settings."""
import json
import os
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(os.path.dirname(BASE_DIR), "config.json")

_lock = threading.Lock()
_cache = None
_dirty_flag = False  # set when web UI saves, consumed by controller


def load():
    global _cache
    with _lock:
        with open(CONFIG_FILE, "r") as f:
            _cache = json.load(f)
        return _cache


def get():
    return _cache if _cache is not None else load()


def save(new_cfg):
    global _cache, _dirty_flag
    with _lock:
        with open(CONFIG_FILE, "w") as f:
            json.dump(new_cfg, f, indent=2)
        _cache = new_cfg
        _dirty_flag = True


def consume_dirty():
    global _dirty_flag
    with _lock:
        was = _dirty_flag
        _dirty_flag = False
        return was
