"""Shared state between controller + web UI. All reads/writes are thread-safe via the GIL for simple bools/strings."""
import threading

class State:
    def __init__(self):
        self.lock = threading.Lock()
        # Water control: True = can spray, False = detection only (same as physical switch press)
        self.water_enabled = True
        # Operation mode: "active", "off_hours", "stopped"
        self.mode = "active"
        self.exit_flag = False
        # Last event timestamps for UI
        self.last_detection = None
        self.last_spray = None

state = State()
