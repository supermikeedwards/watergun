# Models

The detector runs **on the OAK-D-POE** (Intel Myriad X VPU), not on the Pi. The model
is a MyriadX `.blob`; the Pi never loads it — `depthai` ships it to the device.

## `yolov8n_coco_640x640.blob`

- **Architecture:** YOLOv8n (nano), anchor-free.
- **Input:** 640 × 640 × 3 (set in `config.json` -> `oak.model_input_w/h`).
- **Classes:** COCO 80-class (YOLO ordering — `person`=0, `bird`=14). The detector resolves
  target classes **by name** (`watergun/detector.py` -> `COCO_LABELS`), so the numeric index
  never needs hand-maintaining.
- **Compiled for:** 6 SHAVE cores (default for the OAK-D-POE Myriad X).
- **Source:** DepthAI model zoo via `blobconverter` — regenerate with:

  ```bash
  python3 tools/fetch_yolov8n_blob.py
  ```

- **Not committed by default** — run the fetch tool once on any internet-connected machine.
  The `.blob` is a few MB; commit it if you want the Pi to get it via `git pull` instead of
  running blobconverter on the Pi.

## depthai version note

The `.blob` must be compatible with the `depthai` firmware version. We pin `depthai` in
`requirements.txt` (`>=2.24,<3`). If you bump depthai across a major firmware change,
re-run `tools/fetch_yolov8n_blob.py` to recompile the blob against the matching OpenVINO version.

## Switching models

- **YOLOv8s** for higher accuracy (Q1 in worklog §12.6): fetch `yolov8s_coco_640x640` instead,
  update `oak.model_blob`. Check FPS on the VPU is still acceptable.
- **Custom-trained** (pigeon-specific): train YOLOv8n in Ultralytics, export ONNX, convert with
  `blobconverter.from_onnx(...)`. Keep the 80-class head or retrain the label resolution in
  `detector.COCO_LABELS`.
