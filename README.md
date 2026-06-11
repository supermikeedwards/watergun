# Pigeon Watergun 💦

Bird-detecting water gun. An **OAK-D-POE** camera runs YOLOv8n on its Myriad X VPU,
detects and tracks birds on-device, and streams detections to a **Raspberry Pi 3**
controller. The Pi aims two servos, waits for the target to settle, and sprays.
Operates only during configured opening hours; has a physical toggle switch and a
mobile-friendly web UI. A **kids mode** retargets the gun at people for garden fun.

## Architecture

```
OAK-D-POE  (Myriad X VPU)              Raspberry Pi 3  (controller only)
  ColorCamera (IMX378)                   detector.py  -> consumes tracklets over LAN
    -> YOLOv8n (COCO .blob)              controller.py-> aim + spray state machine
    -> ObjectTracker (on-device)         hardware.py  -> PCA9685 servos, GPIO relay, switch
    -> tracklets + preview ──(PoE/Eth)── web.py       -> Flask UI (Logs/Control/Config/Calibrate)
```

The Pi connects to the OAK by IP over a dedicated Ethernet/PoE link
(`config.json` -> `oak.device_ip`). No PiCamera, no on-Pi inference.

## Structure
```
watergun/            # Python package
  config.py          # JSON-backed config, hot-reloaded on save
  logging_setup.py   # Rotating file log + in-memory ring for web UI
  state.py           # Shared state (water_enabled, switch_enabled, kids_mode, ...)
  hardware.py        # GPIO / servos / relay / switch (no camera)
  calibration.py     # Servo calibration file I/O
  detector.py        # OakDetector: depthai pipeline + on-device tracking + stationarity
  controller.py      # Main loop, opening-hours state machine, switch watcher, aim/spray
  web.py             # Flask UI: Logs / Control / Config / Calibrate tabs
config.json          # All runtime-tunable values
models/              # YOLOv8n MyriadX .blob (fetch via tools/fetch_yolov8n_blob.py)
tools/
  fetch_yolov8n_blob.py # Download/compile the YOLOv8n .blob (no OAK needed)
  pi_diagnose.sh        # Pi-side diagnostics
run.py               # Entry point
systemd/watergun.service
```

## Detection

Everything runs on the OAK:
1. **YOLOv8n (COCO 80-class)** full-frame inference at 30+ FPS on the Myriad X VPU.
2. **On-device ObjectTracker** assigns stable track IDs (no Pi-side cv2 tracker).
3. The Pi selects the highest-confidence tracklet of the target class — `bird` (bird mode)
   or `person` (kids mode) — and fires once it has been **stationary** for the mode's dwell
   time (`detection.min_acquire_time` for birds, `kids_mode.stationary_seconds` for kids).

Class indices use the YOLO 80-class scheme (`person`=0, `bird`=14); the detector resolves
target classes by name, so the index is never hand-maintained.

## Setup

```bash
# 1. Fetch the model blob (any machine with internet; no OAK required)
python3 tools/fetch_yolov8n_blob.py

# 2. On the Pi: install deps + run
python3 -m pip install --user -r requirements.txt
python3 run.py
```
Web UI: `http://<pi-ip>:8080`

### OAK network link
The Pi reaches the OAK over a dedicated Ethernet segment (Pi `eth0` static
`192.168.10.1/24`, OAK `192.168.10.2`). The Pi keeps Wi-Fi for the home network.
`depthai` connects directly by IP — see `oak.device_ip` in `config.json`.

## Config highlights
All in `config.json` (editable via the web UI's Config tab):
- `oak` — device IP, model blob, input size, confidence/IoU thresholds, tracker classes
- `detection` — stationary dwell + threshold, detection cooldown
- `aim` — `invert_x` / `invert_y` (OAK mounting orientation; calibrate on hardware)
- `spray` — duration, sweep iterations, post-cycle wait
- `kids_mode` — person threshold, dwell, spray duration/sweep
- `opening_hours`, `images`, `logging`, `telemetry`, `switch`, `calibration`

Most params apply on the next loop iteration. `oak.*`, `web.*`, `logging.*` and switch
config are read at init and require a service restart.

## Raspberry Pi deployment

See [`docs/PI_SETUP.md`](docs/PI_SETUP.md) for full instructions.
