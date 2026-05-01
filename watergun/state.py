"""Shared state between controller + web UI. All reads/writes are thread-safe via the GIL for simple bools/strings.

Water control is an AND-gate of two independent flags:
  - water_enabled : toggled by the web UI only.
  - switch_enabled: reflects the physical switch's current (debounced) level.
Spray only if both True (see .armed).

Calibration mode:
  - calibrating=True : controller pauses detection/spray but keeps the camera running
    and publishes the latest frame as JPEG into latest_jpeg for the web UI's MJPEG stream.
    The /api/calibrate/fire endpoint bypasses the AND-gate (armed check) but only while
    calibrating=True, so nothing can fire accidentally from outside calibration.
"""
import threading

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.water_enabled = True    # web UI
        self.switch_enabled = True   # physical switch
        self.mode = "active"         # "active" | "off_hours"
        self.exit_flag = False
        self.last_detection = None
        self.last_spray = None

        # Calibration mode
        self.calibrating = False
        self.latest_jpeg = None      # latest encoded frame for MJPEG stream (bytes)
        self.latest_jpeg_ts = 0.0    # monotonic timestamp of last update

    @property
    def armed(self):
        return self.water_enabled and self.switch_enabled

state = State()
