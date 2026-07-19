"""Shared calibration operations.

Used by BOTH the LAN Flask UI (web.py) and the cloud command handler (cloud.py)
so the aim formula, clamps, and safety rules stay in exactly one place. This is
the single source of truth the worklog repeatedly warned about keeping in sync.

Every operation that moves a servo or fires water requires calibration mode
(state.calibrating). aim(fire=True) and fire() bypass the armed AND-gate on
purpose — but only while calibrating — so nothing fires accidentally from
outside a deliberate calibration session.
"""
import logging

from . import config, hardware, calibration
from .state import state

log = logging.getLogger(__name__)


class CalibrationError(Exception):
    """Raised for invalid calibration requests (surfaced to web 4xx / cmd resp)."""


def _require_cal():
    if not state.calibrating:
        raise CalibrationError("not in calibration mode")


def enter():
    with state.lock:
        state.calibrating = True
    log.info("Calibration mode ENTERED")
    return {"calibrating": True}


def exit_():
    with state.lock:
        state.calibrating = False
    # Safety: relay off after leaving calibration, regardless of prior state.
    try:
        hardware.relay_off()
    except Exception:
        pass
    log.info("Calibration mode EXITED")
    return {"calibrating": False}


def jog(x=None, y=None):
    _require_cal()
    if x is not None:
        hardware.set_servo(hardware.SERVO_X_CHANNEL, float(x))
    if y is not None:
        hardware.set_servo(hardware.SERVO_Y_CHANNEL, float(y))
    return {"x": x, "y": y}


def set_cal(changes):
    """Persist current angles as named calibration points (e.g. SERVO_X_CENTER)."""
    _require_cal()
    vals = calibration.load()
    allowed = set(calibration.DEFAULTS.keys())
    changed = {}
    for k, v in (changes or {}).items():
        if k not in allowed:
            raise CalibrationError(f"unknown key {k}")
        try:
            vals[k] = float(v)
        except (TypeError, ValueError):
            raise CalibrationError(f"bad value for {k}")
        changed[k] = vals[k]
    if not changed:
        raise CalibrationError("no valid keys")
    calibration.save(vals)
    log.info("Calibration saved: %s", changed)
    return {"changed": changed}


def aim(nx, ny, fire=False, duration_ms=300):
    """Map normalized (nx,ny in [0,1]) -> servo angles using the same inversion the
    controller uses, move there, optionally fire a short burst. Returns the angles."""
    _require_cal()
    try:
        nx = max(0.0, min(1.0, float(nx)))
        ny = max(0.0, min(1.0, float(ny)))
    except (TypeError, ValueError):
        raise CalibrationError("nx/ny required")
    cal = calibration.load()
    cfg = config.get()
    a = cfg.get("aim", {})
    inx = (1.0 - nx) if a.get("invert_x", True) else nx
    iny = (1.0 - ny) if a.get("invert_y", True) else ny
    ax = cal["SERVO_X_MIN_ANGLE"] + inx * (cal["SERVO_X_MAX_ANGLE"] - cal["SERVO_X_MIN_ANGLE"])
    ay = cal["SERVO_Y_MIN_ANGLE"] + iny * (cal["SERVO_Y_MAX_ANGLE"] - cal["SERVO_Y_MIN_ANGLE"])
    ay += cfg["spray"]["water_jet_angle_offset"]
    ax = max(0.0, min(180.0, ax))
    ay = max(0.0, min(180.0, ay))
    hardware.set_servo(hardware.SERVO_X_CHANNEL, ax)
    hardware.set_servo(hardware.SERVO_Y_CHANNEL, ay)
    fired = False
    if fire:
        _require_cal()  # race guard: user could have exited between move and fire
        hardware.short_burst(int(duration_ms) / 1000.0)
        fired = True
    log.info("Calibration aim: nx=%.3f ny=%.3f -> (%.1f,%.1f) fire=%s", nx, ny, ax, ay, fired)
    return {"x": ax, "y": ay, "fired": fired}


def fire(duration_ms=300):
    _require_cal()
    hardware.short_burst(int(duration_ms) / 1000.0)
    return {"duration_ms": int(duration_ms)}


def dispatch(action, params):
    """Route a cloud command action to the right operation. Returns a result dict.
    Raises CalibrationError for bad requests."""
    params = params or {}
    if action == "enter":
        return enter()
    if action == "exit":
        return exit_()
    if action == "jog":
        return jog(params.get("x"), params.get("y"))
    if action == "set_cal":
        return set_cal(params.get("changes") or {k: v for k, v in params.items()})
    if action == "aim":
        return aim(params.get("nx"), params.get("ny"),
                   fire=bool(params.get("fire", False)),
                   duration_ms=int(params.get("duration_ms", 300)))
    if action == "fire":
        return fire(int(params.get("duration_ms", 300)))
    raise CalibrationError(f"unknown action {action!r}")
