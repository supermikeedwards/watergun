# Models

The detector runs **on the OAK-D-POE** (Intel Myriad X VPU), not on the Pi. The Pi
never loads the model — `depthai` ships the `.blob` to the device. This model is the
**RGB detector** for the depth-aware **spatial** pipeline (worklog §20): its bounding
boxes are fused with the stereo depth map on-device by `YoloSpatialDetectionNetwork`,
so every detection carries XYZ (mm).

## `yolov8n_coco_640x640.blob`

- **Architecture:** YOLOv8n (nano), anchor-free. COCO 80-class (`person`=0, `bird`=14).
- **Input:** 640 × 640 × 3, full frame (no crop). `config.json` → `oak.model_input_w/h`.
- **Baked-in preprocessing:** mean `[0,0,0]`, scale `[255,255,255]`, `--reverse_input_channels`
  (blob converts the camera's BGR → the model's expected RGB internally).
- **Compiled for:** 6 SHAVE cores, OpenVINO 2022.1 (blobconverter default).
- **Size / SHA256:** ~6.4 MB — `sha256sum models/yolov8n_coco_640x640.blob` →
  `c97dd6cea9cf4a22fedf41a9ad4757803188af2e9c3dd21f4e58e30a7c5f4f1d`
- **Committed to the repo** so the Pi gets it via `git pull` (no build toolchain on the Pi).

### Provenance (how it was built, 2026-07-31)

1. `yolov8n.pt` — Ultralytics COCO checkpoint, release v8.2.0.
2. **Luxonis `tools` 0.3.6** (`tools yolov8n.pt --imgsz "640 640"`) → DepthAI-headed
   `yolov8n.onnx` + NN-archive `config.json` (3 output heads 80/40/20, YOLO parser,
   80 classes). This head surgery is what makes `YoloSpatialDetectionNetwork` parse it.
3. **blobconverter** `from_onnx(..., data_type="FP16", shaves=6, version="2022.1",`
   `optimizer_params=[mean 0, scale 255, --reverse_input_channels])` → this `.blob`.

Reproduce with `tools/fetch_yolov8n_blob.py` (see that file's header for the full
STEP 1 / STEP 2 commands).

## depthai version note

The `.blob` (OpenVINO 2022.1) is compatible with `depthai` 2.x (pinned `>=2.24,<3` in
`requirements.txt`). The Pi 3 is 32-bit armv7l, so depthai stays on v2. If you ever bump
depthai across a major OpenVINO change, rebuild the blob.

## Switching models

- **YOLOv8s** (more accuracy for the occluded/camouflaged scene, fewer FPS on the VPU —
  still fine for a 2 s dwell): repeat provenance STEP 1 with `yolov8s.pt`, STEP 2 unchanged,
  rename the output + update `config.json` → `oak.model_blob`. Same parser config
  (anchor-free, 80-class) so no other change.
- The on-device decode params (`num_classes=80`, `coord_size=4`, `anchors=[]`,
  `anchor_masks={}`) live in `config.json` → `oak` and match this model.
