"""Config loader/saver — single source of truth for runtime-tunable settings."""
import json
import os
import threading

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(os.path.dirname(BASE_DIR), "config.json")

_lock = threading.Lock()
_cache = None
_dirty_flag = False  # set when web UI saves, consumed by controller


class ConfigShapeError(ValueError):
    pass


def load():
    global _cache
    with _lock:
        with open(CONFIG_FILE, "r") as f:
            _cache = json.load(f)
        return _cache


def get():
    return _cache if _cache is not None else load()


def validate_against(new_cfg, reference):
    """Reject saves that would corrupt structure. Recursively requires the same dict
    key sets as `reference`; leaves must be JSON primitives (str/int/float/bool/None).
    Leaf type changes are allowed (e.g. bird_class_id: null → int)."""
    def _check(new, ref, path):
        if isinstance(ref, dict):
            if not isinstance(new, dict):
                raise ConfigShapeError(f"{path or '<root>'}: expected object, got {type(new).__name__}")
            extra = set(new) - set(ref)
            missing = set(ref) - set(new)
            if extra:
                raise ConfigShapeError(f"{path or '<root>'}: unknown key(s) {sorted(extra)}")
            if missing:
                raise ConfigShapeError(f"{path or '<root>'}: missing key(s) {sorted(missing)}")
            for k in ref:
                _check(new[k], ref[k], f"{path}.{k}" if path else k)
        else:
            if not isinstance(new, (str, int, float, bool, type(None))):
                raise ConfigShapeError(f"{path}: leaf must be string/number/bool/null, got {type(new).__name__}")
    _check(new_cfg, reference, "")


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
