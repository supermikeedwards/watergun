# Test images for offline validation

This directory holds the curated test set used by `tools/validate_detector.py` to gate
deployment to the Pi (worklog §10.6).

**The image files themselves are gitignored** — only this README is tracked. Curate locally
or share via the OneDrive folder.

## What to put here

~20 representative pigeon-in-foliage images (per worklog §10.6 step 2):

| Count | Category | Notes |
|------:|----------|-------|
| 5 | Clear view | Pigeon mostly unobscured. Baseline — model should detect ~all. |
| 8 | Partial occlusion | Branches / leaves covering 30–60% of body. The hard case. |
| 4 | Pose variation | Sitting, head-down feeding, wings spread mid-landing, turned away. |
| 3 | Lighting variation | Bright sun, deep shade, golden hour / low light. |

Mix sources:
- **iNaturalist** (https://inaturalist.org) — search "Columba palumbus" or "feral pigeon",
  filter for CC-licensed observations.
- **Flickr Creative Commons** — CC BY / CC BY-SA images of `pigeon` or `wood pigeon`.
- **Mike's own cherry tree photos** — most representative of the actual deployment scene.
  These are the ones that matter most. If the model misses *these*, fine-tuning or Path B is needed.

## Naming

No naming convention required, but optional prefix helps with reading the report:
- `clear_NN.jpg` — unobscured baseline
- `occluded_NN.jpg` — partial occlusion
- `pose_NN.jpg` — pose variation
- `light_NN.jpg` — lighting variation
- `cherrytree_NN.jpg` — Mike's own deployment-site photos

## Image format

Any common format (`.jpg`, `.jpeg`, `.png`, `.bmp`, `.webp`). The validation script auto-resizes
to the model's 320×320 input — but feeding it images closer to 640×480 (the camera's native
resolution) makes the test more representative of what the Pi will actually see.

Keep file sizes reasonable (< 2 MB each) — these are quick smoke tests, not benchmarks.

## Running validation

```bash
cd ~/projects/pigeon_watergun
# one-off venv setup (laptop)
python3 -m venv .venv-validate
source .venv-validate/bin/activate
pip install -r requirements.txt   # tflite-runtime may need 'tensorflow' fallback on x86_64
# run
python3 tools/validate_detector.py
# results in validation_output/
```

Pass criteria (worklog §10.6 step 5):
- ≥ 80% recall on the test set
- ≤ 1 false positive per image
- at confidence threshold 0.4

If validation **fails** → discuss with Mike before any Pi deployment. Options:
fine-tune custom model, drop in EfficientDet Lite0, or escalate to Path B (Coral USB).
