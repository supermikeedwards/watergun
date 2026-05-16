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
  detector.py        # MOG2 motion gate + TFLite bird detector + KCF tracker (§10)
  controller.py      # Main loop, opening-hours state machine, switch watcher
  web.py             # Flask UI: Logs / Control / Config / Calibrate tabs
config.json          # All runtime-tunable values
models/              # TFLite model + COCO labels (committed)
tools/
  validate_detector.py  # Offline validation gate (laptop) — see worklog §10.6
test_images/         # Curated pigeon set for offline validation (gitignored)
run.py               # Entry point
systemd/watergun.service
```

## Detection (post §10 redesign)

Two-stage pipeline that replaces the original "largest moving contour" heuristic:
1. **MOG2 background subtraction** absorbs swaying branches into the background after
   ~1–2 minutes; only genuinely new foreground survives as ROI candidates.
2. **TFLite SSDLite MobileDet (INT8, 320×320)** runs on each ROI (and a periodic full-frame
   sweep) and accepts only detections matching the COCO `bird` class above
   `bird_detection.confidence_threshold`.

Falls back to the legacy motion-only path if `bird_detection.legacy_motion_only=true`.

## Running locally (on the Pi)
```bash
python3 -m pip install -r requirements.txt
python3 run.py
```
Web UI: `http://<pi-ip>:8080`

## Offline validation (before any Pi deployment)

```bash
# laptop — set up venv with deps including tflite-runtime or tensorflow
python3 -m venv .venv-validate && source .venv-validate/bin/activate
pip install -r requirements.txt
# drop ~20 curated images into test_images/ — see test_images/README.md
python3 tools/validate_detector.py
```
Pass criteria: ≥ 80% recall, ≤ 1 FP / image at confidence 0.4. See worklog §10.6.

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
