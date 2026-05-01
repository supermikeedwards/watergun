"""Servo calibration — kept close to original interactive flow but headless-friendly file I/O."""
import logging
import os

log = logging.getLogger(__name__)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIBRATION_FILE = os.path.join(BASE_DIR, "servo_calibration.txt")

DEFAULTS = {
    # Last-known-working values from the Pi deployment (2026-04-24 backup).
    # Kept here so a missing servo_calibration.txt falls back to a usable
    # calibration rather than generic 90/90 factory defaults.
    "SERVO_X_CENTER": 92.0,
    "SERVO_Y_CENTER": 48.0,
    "SERVO_X_MIN_ANGLE": 64.0,
    "SERVO_X_MAX_ANGLE": 126.0,
    "SERVO_Y_MIN_ANGLE": 36.0,
    "SERVO_Y_MAX_ANGLE": 68.0,
}


def load():
    vals = dict(DEFAULTS)
    if os.path.exists(CALIBRATION_FILE):
        try:
            with open(CALIBRATION_FILE) as f:
                for line in f:
                    if "=" in line:
                        k, v = line.strip().split("=", 1)
                        if k in vals:
                            vals[k] = float(v)
            log.info("Loaded servo calibration from %s", CALIBRATION_FILE)
        except Exception as e:
            log.error("Failed to read calibration (%s); using defaults", e)
    else:
        save(vals)
    return vals


def save(vals):
    with open(CALIBRATION_FILE, "w") as f:
        for k, v in vals.items():
            f.write(f"{k}={v}\n")
    log.info("Saved servo calibration to %s", CALIBRATION_FILE)
