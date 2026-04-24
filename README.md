# Pigeon Watergun 💦

Raspberry Pi motion-detecting water gun. Detects pigeons (or other movement) in the camera frame, tracks them with OpenCV KCF, waits for the target to settle, and sprays with a sweep pattern. Operates only during configured opening hours; has a physical toggle switch and a mobile-friendly web UI.

## Structure
```
watergun/            # Python package
  config.py          # JSON-backed config, hot-reloaded on save
  logging_setup.py   # Rotating file log + in-memory ring for web UI
  state.py           # Shared state (water_enabled, mode, etc.)
  hardware.py        # GPIO / servos / camera wrappers
  calibration.py     # Servo calibration file I/O
  detector.py        # Motion detection + KCF tracker + stationary check
  controller.py      # Main loop, opening-hours state machine, switch watcher
  web.py             # Flask UI: Logs / Control / Config tabs
config.json          # All runtime-tunable values
run.py               # Entry point
systemd/watergun.service
```

## Running locally (on the Pi)
```bash
python3 -m pip install -r requirements.txt
python3 run.py
```
Web UI: `http://<pi-ip>:8080`

## Config highlights
All in `config.json` (editable via the web UI's Config tab):
- `opening_hours` — `start`/`end` (24h, Europe/Paris), plus `off_hours_cycle_seconds` (sleep interval outside opening hours)
- `detection` — thresholds, min area, stationary criteria
- `spray` — duration, sweep iterations, post-cycle wait
- `images` — JPEG quality, retention days (auto-delete)
- `logging` — rotation size/backup count

Detection params apply on the next loop iteration. Camera / hardware changes require a service restart.

## Raspberry Pi deployment

See [`docs/PI_SETUP.md`](docs/PI_SETUP.md) for full instructions: static IP, pull, install, enable autostart.
