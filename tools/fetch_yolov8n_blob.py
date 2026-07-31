#!/usr/bin/env python3
"""Build the YOLOv8n COCO MyriadX .blob for the OAK-D-POE spatial detector.

Background (worklog §19.10 / §20): the legacy DepthAI model zoo has NO 640x640
YOLOv8, so `blobconverter.from_zoo("yolov8n_coco_640x640")` 404s. The canonical
path is Luxonis's YOLO converter (head surgery) -> ONNX -> blobconverter -> .blob.
The committed blob was produced exactly as below; this script reproduces it.

The .blob is committed to the repo (models/yolov8n_coco_640x640.blob) so the Pi
gets it via `git pull` — you normally do NOT need to run this. Run it only to
rebuild (e.g. to swap to yolov8s, change input size, or bump OpenVINO).

------------------------------------------------------------------------------
STEP 1 — DepthAI-headed ONNX from an Ultralytics checkpoint (needs luxonis/tools)

  # isolated env (torch etc. are heavy; mmcv is GoldYOLO-only, exclude it):
  python3 -m venv .venv && . .venv/bin/activate   # (bootstrap pip if needed)
  git clone --recursive https://github.com/luxonis/tools.git
  grep -v -i '^mmcv' tools/requirements.txt > req.txt
  PIP_CONSTRAINT=tools/constraints.txt pip install -r req.txt blobconverter
  pip install --no-deps ./tools
  curl -L -o yolov8n.pt \
    https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt
  tools yolov8n.pt --imgsz "640 640"        # -> shared_with_container/outputs/.../yolov8n.onnx

STEP 2 — compile that ONNX to a MyriadX .blob (this script):

  python3 tools/fetch_yolov8n_blob.py path/to/yolov8n.onnx

------------------------------------------------------------------------------
Normalization / channel flags below MUST match the NN archive's declared
preprocessing (mean=0, scale=255, model wants RGB). The OAK ColorCamera is set to
BGR order in detector.py, so we compile with --reverse_input_channels; the blob
then converts BGR->RGB internally and saved JPEGs stay correct.
"""
import os
import shutil
import sys

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
OUT_PATH = os.path.join(OUT_DIR, "yolov8n_coco_640x640.blob")
SHAVES = 6
OPENVINO_VERSION = "2022.1"
OPTIMIZER_PARAMS = [
    "--mean_values=[0,0,0]",
    "--scale_values=[255,255,255]",
    "--reverse_input_channels",
]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print("usage: fetch_yolov8n_blob.py <path-to-yolov8n.onnx>", file=sys.stderr)
        return 2
    onnx = sys.argv[1]
    if not os.path.isfile(onnx):
        print(f"ONNX not found: {onnx} (run STEP 1 first)", file=sys.stderr)
        return 1
    try:
        import blobconverter
    except ImportError:
        print("blobconverter not installed. Run: pip install blobconverter", file=sys.stderr)
        return 1

    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Compiling {onnx} -> MyriadX blob (shaves={SHAVES}, OpenVINO {OPENVINO_VERSION})...")
    blob = blobconverter.from_onnx(
        model=onnx,
        data_type="FP16",
        shaves=SHAVES,
        optimizer_params=OPTIMIZER_PARAMS,
        version=OPENVINO_VERSION,
    )
    shutil.copy(blob, OUT_PATH)
    print(f"Wrote {OUT_PATH} ({os.path.getsize(OUT_PATH) / 1e6:.1f} MB)")
    print("Matches config.json -> oak.model_blob. Commit it so the Pi gets it via git pull.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
