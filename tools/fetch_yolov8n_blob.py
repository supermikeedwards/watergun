#!/usr/bin/env python3
"""Fetch / compile the YOLOv8n COCO MyriadX .blob for the OAK-D-POE.

No OAK hardware is required to run this — blobconverter compiles in the cloud (or
locally) and writes the .blob into models/. Run it once on any machine with internet:

    python3 tools/fetch_yolov8n_blob.py

It pulls the pre-converted YOLOv8n COCO model from the DepthAI model zoo, compiled
for 6 SHAVE cores (a safe default for the Myriad X on the OAK-D-POE). The output
path matches config.json -> oak.model_blob.

If the zoo name ever changes, see https://github.com/luxonis/depthai-model-zoo for
the current YOLOv8n entry, or convert your own .onnx via blobconverter.from_onnx().
"""
import os
import sys

MODEL_NAME = "yolov8n_coco_640x640"
ZOO_TYPE = "depthai"
SHAVES = 6
OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
OUT_PATH = os.path.join(OUT_DIR, f"{MODEL_NAME}.blob")


def main():
    try:
        import blobconverter
    except ImportError:
        print("blobconverter not installed. Run: pip install blobconverter", file=sys.stderr)
        return 1

    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"Fetching {MODEL_NAME} (zoo={ZOO_TYPE}, shaves={SHAVES})...")
    blob_path = blobconverter.from_zoo(
        name=MODEL_NAME,
        zoo_type=ZOO_TYPE,
        shaves=SHAVES,
    )
    # blobconverter caches into its own dir; copy to our models/ path.
    import shutil
    shutil.copy(blob_path, OUT_PATH)
    size_mb = os.path.getsize(OUT_PATH) / 1e6
    print(f"Wrote {OUT_PATH} ({size_mb:.1f} MB)")
    print("This path matches config.json -> oak.model_blob.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
