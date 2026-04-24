#!/usr/bin/env python3
"""Entry point: starts web UI in a background thread, then runs the controller on the main thread."""
import threading
from watergun import config, logging_setup, controller, web

def main():
    cfg = config.load()
    logging_setup.setup(
        max_bytes=cfg["logging"]["max_bytes"],
        backup_count=cfg["logging"]["backup_count"],
    )
    w = cfg["web"]
    threading.Thread(target=web.run, args=(w["host"], w["port"]), daemon=True).start()
    controller.run()

if __name__ == "__main__":
    main()
