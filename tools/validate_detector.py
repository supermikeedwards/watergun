"""Offline validation gate for the bird detector (worklog §10.6).

Runs the configured TFLite bird detector against a directory of static images, prints
recall + false-positive metrics, and saves annotated copies for manual review.

USAGE:
    python3 tools/validate_detector.py
    python3 tools/validate_detector.py --images test_images --out validation_output
    python3 tools/validate_detector.py --threshold 0.3 --show-all-classes

Pass criteria (per worklog §10.6 step 5):
    >= 80% recall on the test set, <= 1 false positive per image, at confidence >= 0.4.

The script is intentionally laptop-friendly: MOG2 motion gating is bypassed (a single
static image has no temporal context). We measure pure BirdDetector capability.
That's the critical question — if the model can't see the pigeon at all, no amount
of ROI cropping on the Pi will help.

Output:
    - Per-image annotated JPEG written to --out/{stem}.jpg with bbox + score.
    - validation_report.json with per-image results + summary.
    - validation_report.txt (human-readable) with pass/fail verdict.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

# Make the watergun package importable when running this script directly.
REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

from watergun.detector import BirdDetector, _TFLITE_BACKEND  # noqa: E402

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def load_config(path: Path) -> dict:
    with open(path) as f:
        return json.load(f)


def list_images(images_dir: Path) -> list[Path]:
    if not images_dir.is_dir():
        raise FileNotFoundError(f"Test images directory not found: {images_dir}")
    out = []
    for p in sorted(images_dir.iterdir()):
        if p.is_file() and p.suffix.lower() in IMG_EXTS:
            out.append(p)
    return out


def annotate(frame_bgr: np.ndarray, detections: list[dict], labels: list[str], bird_idx: int) -> np.ndarray:
    """Draws all detections; bird boxes in green, other classes in yellow."""
    out = frame_bgr.copy()
    for d in detections:
        x, y, w, h = d["bbox"]
        is_bird = d["class_idx"] == bird_idx
        color = (0, 200, 0) if is_bird else (0, 200, 200)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 2)
        label = labels[d["class_idx"]] if 0 <= d["class_idx"] < len(labels) else f"id{d['class_idx']}"
        text = f"{label} {d['score']:.2f}"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (x, y - th - 4), (x + tw + 4, y), color, -1)
        cv2.putText(out, text, (x + 2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    return out


def run_inference_all_classes(birds: BirdDetector, frame_bgr: np.ndarray) -> tuple[list[dict], float]:
    """Runs full inference and returns ALL detections (any class), with timing."""
    h, w = frame_bgr.shape[:2]
    t0 = time.perf_counter()
    raw = birds.infer(frame_bgr)
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    out = []
    for r in raw:
        ymin, xmin, ymax, xmax = r["bbox_norm"]
        x = max(0, int(xmin * w))
        y = max(0, int(ymin * h))
        bw = min(w - x, int((xmax - xmin) * w))
        bh = min(h - y, int((ymax - ymin) * h))
        if bw <= 0 or bh <= 0:
            continue
        out.append({"bbox": (x, y, bw, bh), "score": r["score"], "class_idx": r["class_idx"]})
    return out, elapsed_ms


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(REPO_ROOT / "config.json"),
                    help="Config file with bird_detection section (default: ./config.json)")
    ap.add_argument("--images", default=str(REPO_ROOT / "test_images"),
                    help="Directory of test images (default: ./test_images)")
    ap.add_argument("--out", default=str(REPO_ROOT / "validation_output"),
                    help="Directory for annotated outputs + report (default: ./validation_output)")
    ap.add_argument("--threshold", type=float, default=None,
                    help="Override bird_detection.confidence_threshold for this run")
    ap.add_argument("--show-all-classes", action="store_true",
                    help="Annotate ALL detections (not just birds). Useful for diagnosing "
                         "'pigeon detected as cat' style failures.")
    ap.add_argument("--pass-recall", type=float, default=0.80,
                    help="Pass criterion: minimum recall (default 0.80 per §10.6)")
    ap.add_argument("--pass-max-fp-per-image", type=float, default=1.0,
                    help="Pass criterion: max false positives per image (default 1.0)")
    args = ap.parse_args()

    cfg = load_config(Path(args.config))
    if args.threshold is not None:
        cfg["bird_detection"]["confidence_threshold"] = args.threshold

    images_dir = Path(args.images)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    images = list_images(images_dir)
    if not images:
        print(f"No images found in {images_dir}.")
        print("See test_images/README.md for what to put there.")
        sys.exit(2)

    print(f"Loading BirdDetector (backend={_TFLITE_BACKEND}) ...")
    birds = BirdDetector(cfg)
    print(f"Bird class: '{birds.labels[birds.bird_idx]}' (idx={birds.bird_idx})")
    print(f"Confidence threshold: {birds.conf_threshold:.2f}")
    print(f"Validating {len(images)} images from {images_dir}\n")

    per_image = []
    latencies_ms = []
    bird_class_counter = Counter()
    detected_count = 0
    total_fp = 0  # total non-bird detections across all images

    for img_path in images:
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"  [WARN] could not read {img_path.name}, skipping")
            continue
        all_dets, elapsed_ms = run_inference_all_classes(birds, frame)
        latencies_ms.append(elapsed_ms)

        bird_dets = [d for d in all_dets if d["class_idx"] == birds.bird_idx]
        non_bird_dets = [d for d in all_dets if d["class_idx"] != birds.bird_idx]
        for d in non_bird_dets:
            bird_class_counter[birds.labels[d["class_idx"]] if 0 <= d["class_idx"] < len(birds.labels) else f"id{d['class_idx']}"] += 1
        max_bird_score = max((d["score"] for d in bird_dets), default=0.0)

        detected = bool(bird_dets)
        if detected:
            detected_count += 1
        # "False positive" = any detection that's not a bird (since this is a curated bird test set)
        total_fp += len(non_bird_dets)

        annotated = annotate(
            frame,
            all_dets if args.show_all_classes else bird_dets,
            birds.labels,
            birds.bird_idx,
        )
        out_path = out_dir / f"{img_path.stem}_annotated.jpg"
        cv2.imwrite(str(out_path), annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])

        per_image.append({
            "image": img_path.name,
            "detected": detected,
            "max_bird_score": round(max_bird_score, 3),
            "n_bird_detections": len(bird_dets),
            "n_other_detections": len(non_bird_dets),
            "other_classes": [
                {
                    "class": birds.labels[d["class_idx"]] if 0 <= d["class_idx"] < len(birds.labels) else f"id{d['class_idx']}",
                    "score": round(d["score"], 3),
                }
                for d in non_bird_dets
            ],
            "inference_ms": round(elapsed_ms, 1),
            "annotated_output": str(out_path.relative_to(REPO_ROOT)),
        })
        status = "BIRD" if detected else "MISS"
        print(f"  {status:<4}  {img_path.name:<40}  best_bird={max_bird_score:.2f}  "
              f"others={len(non_bird_dets)}  {elapsed_ms:.0f} ms")

    # Summary
    n = len(per_image)
    recall = detected_count / n if n else 0.0
    fp_per_image = total_fp / n if n else 0.0
    median_ms = float(np.median(latencies_ms)) if latencies_ms else 0.0
    p95_ms = float(np.percentile(latencies_ms, 95)) if latencies_ms else 0.0

    summary = {
        "total_images": n,
        "detected": detected_count,
        "recall": round(recall, 3),
        "total_false_positives": total_fp,
        "fp_per_image": round(fp_per_image, 3),
        "false_positive_classes": dict(bird_class_counter.most_common()),
        "inference_latency_ms": {"median": round(median_ms, 1), "p95": round(p95_ms, 1)},
        "config_used": {
            "model_path": cfg["bird_detection"]["model_path"],
            "confidence_threshold": cfg["bird_detection"]["confidence_threshold"],
            "bird_class_idx": birds.bird_idx,
            "tflite_backend": _TFLITE_BACKEND,
        },
        "pass_criteria": {
            "min_recall": args.pass_recall,
            "max_fp_per_image": args.pass_max_fp_per_image,
        },
    }
    summary["passed"] = recall >= args.pass_recall and fp_per_image <= args.pass_max_fp_per_image

    # Write JSON report
    json_path = out_dir / "validation_report.json"
    with open(json_path, "w") as f:
        json.dump({"summary": summary, "per_image": per_image}, f, indent=2)

    # Write human-readable report
    txt_path = out_dir / "validation_report.txt"
    with open(txt_path, "w") as f:
        verdict = "PASS" if summary["passed"] else "FAIL"
        f.write(f"VALIDATION REPORT — {verdict}\n")
        f.write("=" * 50 + "\n")
        f.write(f"Images:               {n}\n")
        f.write(f"Birds detected:       {detected_count} ({recall*100:.1f}% recall)\n")
        f.write(f"False positives:      {total_fp} total, {fp_per_image:.2f} per image\n")
        f.write(f"Latency (median/p95): {median_ms:.0f} / {p95_ms:.0f} ms (laptop, NOT Pi 3)\n")
        f.write(f"Backend:              {_TFLITE_BACKEND}\n")
        f.write(f"Confidence threshold: {birds.conf_threshold:.2f}\n")
        f.write("\n")
        f.write(f"Pass criteria: recall >= {args.pass_recall:.2f} AND fp/image <= {args.pass_max_fp_per_image:.2f}\n")
        f.write(f"Verdict:       {verdict}\n")
        if bird_class_counter:
            f.write("\nFalse-positive classes (frequency):\n")
            for cls, count in bird_class_counter.most_common():
                f.write(f"  {cls:<25} {count}\n")
        f.write("\nMissed images (no bird detected at threshold):\n")
        misses = [p for p in per_image if not p["detected"]]
        if not misses:
            f.write("  (none)\n")
        else:
            for p in sorted(misses, key=lambda x: x["max_bird_score"], reverse=True):
                f.write(f"  {p['image']:<40}  best_bird_score={p['max_bird_score']:.2f}\n")

    print()
    print("=" * 60)
    print(f"  Recall:      {detected_count}/{n} = {recall*100:.1f}%  (need >= {args.pass_recall*100:.0f}%)")
    print(f"  FP/image:    {fp_per_image:.2f}             (need <= {args.pass_max_fp_per_image:.2f})")
    print(f"  Median lat:  {median_ms:.0f} ms (laptop x86_64, NOT Pi 3)")
    print(f"  Verdict:     {'PASS ✓' if summary['passed'] else 'FAIL ✗'}")
    print("=" * 60)
    print(f"  Annotated images:  {out_dir}")
    print(f"  JSON report:       {json_path}")
    print(f"  Text report:       {txt_path}")
    sys.exit(0 if summary["passed"] else 1)


if __name__ == "__main__":
    main()
