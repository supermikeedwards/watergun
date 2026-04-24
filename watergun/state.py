"""Shared state between controller + web UI. All reads/writes are thread-safe via the GIL for simple bools/strings.

Water control is an AND-gate of two independent flags:
  - water_enabled : toggled by the web UI only.
  - switch_enabled: reflects the physical switch's current (debounced) level.
Spray only if both True (see .armed).
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

    @property
    def armed(self):
        return self.water_enabled and self.switch_enabled

state = State()
