# Models

TFLite models used by the bird detector (worklog §10).

## `ssdlite_mobiledet_coco_qat_postprocess.tflite`

- **Architecture:** SSDLite + MobileDet (Cortex-A backbone), with built-in NMS post-processing op.
- **Quantization:** INT8 (QAT — quantization-aware-training).
- **Input:** 320 × 320 × 3, uint8 RGB.
- **Output:** standard SSD post-process tuple — boxes (1, 10, 4), classes (1, 10), scores (1, 10), num_detections (1,).
- **Trained on:** COCO 2017 (90-class label map).
- **Source:** https://github.com/google-coral/test_data/raw/master/ssdlite_mobiledet_coco_qat_postprocess.tflite
  (downloaded 2026-05-16; SHA256 `32c486140391eb4dc43fca7113ad392be632dc5366687f2731f73d740678693f`)
- **Licence:** Apache 2.0 (TensorFlow Object Detection API). Same model is published as the
  CPU companion to Coral's Edge TPU MobileDet release.
- **Pi 3 expected latency:** ~589 ms full-frame (per arxiv:2409.16808). On a small ROI crop
  (e.g. 96×96..192×192 padded by `bird_detection.roi_padding_px`) it's ~150 ms — that's the
  primary inference path; full-frame sweeps run only every `full_frame_sweep_interval_seconds`.

## `coco_labels.txt`

- 90 lines, one class per line, 0-indexed by line number.
- Class indices include `n/a` placeholders (lines 12, 26, 29, 30, 45, 66, 68, 69, 71, 83) — these
  are unused class IDs in the COCO 90-class spec, kept as gaps so the index lines up with the
  model's output IDs.
- Bird is at **index 15** (line 16). The detector resolves this dynamically by class name from
  the labels file — `bird_detection.bird_class_id` in `config.json` is `null` by default and
  only needed if a future model uses a different label scheme.
- **Source:** https://github.com/google-coral/test_data/raw/master/coco_labels.txt

## Replacing the model

If validation against the pigeon test set fails (worklog §10.6 pass criteria), options:

1. **Try EfficientDet Lite0 INT8** — slightly slower (~611 ms full-frame on Pi 3) but
   higher mAP. Drop the file in here, point `bird_detection.model_path` at it, re-run validation.
2. **Train a custom pigeon classifier** — half-day's work; fine-tune SSDLite on Mike's own
   cherry-tree photos. Out of scope for the first iteration.
3. **Switch to Path B** (Coral USB Accelerator) — requires the `_edgetpu.tflite` variant
   compiled with `edgetpu_compiler`, plus the `pycoral` runtime. See worklog §10.4.
