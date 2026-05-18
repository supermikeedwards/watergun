"""Two-stage detection pipeline (worklog §10.3): MOG2 motion gate + TFLite SSD bird detector.

Replaces the old "largest moving contour" heuristic, which couldn't tell a swaying branch
from a partially-occluded camouflaged pigeon (worklog §10.1). Pipeline per call to detect():

  1. MotionStage runs MOG2 background subtraction. Wind-swayed branches are absorbed into
     the background after ~1-2 minutes; only genuinely new foreground pixels survive.
  2. From the foreground mask, propose 1..N ROI candidate boxes (size band + morph cleanup).
  3. BirdDetector runs TFLite SSDLite MobileDet on each ROI (or full frame on the periodic
     sweep), thresholds on COCO 'bird' class confidence.
  4. Best-scoring bird detection is returned to the controller, which then falls back to
     KCF tracking + the existing is_stationary check before authorising a spray.

Public API matches the previous Detector so controller.py only needs minor changes:
  - process_frame(image) -> resized BGR frame
  - detect(frame_bgr)    -> (cx, cy, bbox, score) | None
  - start_tracking, update_tracking, is_stationary, reset (unchanged)
  - reset_motion_background() (new) -> called by controller after AE re-lock

Legacy mode: setting bird_detection.legacy_motion_only=true switches to LegacyMotionDetector
(the pre-2026-05-16 motion-only path) for emergency rollback. No model required in that mode.
"""
import cv2
import logging
import math
import os
import time

import imutils
import numpy as np

log = logging.getLogger(__name__)


def _create_kcf_tracker():
    """Create a KCF tracker, robust to opencv-python vs opencv-contrib-python builds.

    OpenCV's tracker module has been reorganised several times across 4.x. Depending on the
    installed wheel, the KCF tracker can live in any of these places:
      * cv2.TrackerKCF.create()        — modern main-namespace class (some 4.5+ builds)
      * cv2.TrackerKCF_create()        — flat function form (some builds)
      * cv2.legacy.TrackerKCF_create() — opencv-contrib-python's legacy module

    We probe in that order. If none is available the build is missing the tracking module
    entirely and the user needs opencv-contrib-python(-headless).
    """
    if hasattr(cv2, "TrackerKCF"):
        return cv2.TrackerKCF.create()
    if hasattr(cv2, "TrackerKCF_create"):
        return cv2.TrackerKCF_create()
    legacy = getattr(cv2, "legacy", None)
    if legacy is not None and hasattr(legacy, "TrackerKCF_create"):
        return legacy.TrackerKCF_create()
    raise RuntimeError(
        "KCF tracker not found in cv2 build. Install opencv-contrib-python-headless "
        "(or opencv-contrib-python) — the non-contrib build dropped KCF in 4.5.x."
    )


# Pi 3 uses tflite_runtime; laptop dev/validation falls back to ai_edge_litert
# (Google's modern rebrand with Py3.12 wheels) or full tensorflow as last resort.
_TFLITE_BACKEND = None
try:
    from tflite_runtime.interpreter import Interpreter as _Interpreter  # type: ignore
    _TFLITE_BACKEND = "tflite_runtime"
except ImportError:
    try:
        from ai_edge_litert.interpreter import Interpreter as _Interpreter  # type: ignore
        _TFLITE_BACKEND = "ai_edge_litert"
    except ImportError:
        try:
            from tensorflow.lite.python.interpreter import Interpreter as _Interpreter  # type: ignore
            _TFLITE_BACKEND = "tensorflow"
        except ImportError:
            _Interpreter = None  # type: ignore


# ----------------------------- Stage 1: MOG2 motion gate -----------------------------

class MotionStage:
    """MOG2 background subtractor wrapper. Learns background over `history` frames; foreground
    = pixels that don't fit the per-pixel Gaussian mixture. Wind-swayed leaves get absorbed
    into the background after ~1-2 minutes of training, unlike single-frame diff which fires
    on every sway."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._init_bg()
        self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def _params(self):
        m = self.cfg["bird_detection"]["motion_stage"]
        return {
            "history": m.get("history", 500),
            "var_threshold": m.get("var_threshold", 16),
            "detect_shadows": m.get("detect_shadows", False),
            "min_area": m.get("min_area_px", 600),
            "max_area": m.get("max_area_px", 60000),
        }

    def _init_bg(self):
        p = self._params()
        self.bg = cv2.createBackgroundSubtractorMOG2(
            history=p["history"],
            varThreshold=p["var_threshold"],
            detectShadows=p["detect_shadows"],
        )

    def reset(self):
        """Re-create the MOG2 instance to wipe the learned background. Called by controller
        after camera AE re-lock, so the exposure step isn't seen as a giant moving region."""
        self._init_bg()
        log.info("MOG2 background reset")

    def propose_rois(self, frame_bgr):
        """Returns list of (x, y, w, h) candidate boxes from the foreground mask, sorted by
        area descending and capped to bird_detection.max_rois_per_frame."""
        fg = self.bg.apply(frame_bgr)
        # If detect_shadows=True, MOG2 marks shadow pixels with value 127. Drop them.
        _, fg = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
        # Morph open then close = remove speckle, fill holes inside blobs.
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self.morph_kernel)
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, self.morph_kernel)
        contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        p = self._params()
        rois = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < p["min_area"] or area > p["max_area"]:
                continue
            x, y, w, h = cv2.boundingRect(c)
            rois.append((x, y, w, h))
        rois.sort(key=lambda r: r[2] * r[3], reverse=True)
        cap = self.cfg["bird_detection"].get("max_rois_per_frame", 5)
        return rois[:cap]


# ------------------------- Stage 2: TFLite bird classifier -------------------------

class BirdDetector:
    """TFLite SSDLite MobileDet (or compatible SSD post-processed model) wrapper.
    Resolves the 'bird' class index by name from the labels file so we don't have to
    debate which of the COCO 80/90-class label conventions a given model uses."""

    def __init__(self, cfg):
        if _Interpreter is None:
            raise RuntimeError(
                "Neither tflite_runtime nor tensorflow.lite is importable. "
                "Install tflite-runtime on the Pi, or tensorflow on the laptop for validation."
            )
        bd = cfg["bird_detection"]
        model_path = self._resolve_path(bd["model_path"])
        labels_path = self._resolve_path(bd["labels_path"])
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"TFLite model not found: {model_path}")
        if not os.path.exists(labels_path):
            raise FileNotFoundError(f"COCO labels file not found: {labels_path}")

        self.interp = _Interpreter(model_path=model_path)
        self.interp.allocate_tensors()
        self.input_details = self.interp.get_input_details()
        self.output_details = self.interp.get_output_details()
        self.input_h = self.input_details[0]["shape"][1]
        self.input_w = self.input_details[0]["shape"][2]
        self.input_dtype = self.input_details[0]["dtype"]

        with open(labels_path) as f:
            self.labels = [line.strip() for line in f.readlines()]

        # Resolve bird class index by name. bird_class_id (if set in config) overrides.
        bird_name = bd.get("bird_class_name", "bird")
        explicit_id = bd.get("bird_class_id")
        if explicit_id is not None:
            self.bird_idx = int(explicit_id)
        else:
            try:
                self.bird_idx = self.labels.index(bird_name)
            except ValueError:
                raise ValueError(f"Class '{bird_name}' not found in {labels_path}")

        self.conf_threshold = bd.get("confidence_threshold", 0.4)
        log.info(
            "BirdDetector ready: backend=%s input=%dx%d dtype=%s bird='%s'(idx=%d) threshold=%.2f model=%s",
            _TFLITE_BACKEND, self.input_w, self.input_h, self.input_dtype,
            bird_name, self.bird_idx, self.conf_threshold, os.path.basename(model_path),
        )

    @staticmethod
    def _resolve_path(p):
        if os.path.isabs(p):
            return p
        base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(base, p)

    def _preprocess(self, crop_bgr):
        rgb = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(rgb, (self.input_w, self.input_h))
        if self.input_dtype == np.uint8:
            return np.expand_dims(resized, axis=0).astype(np.uint8)
        # Float input model: scale to [0,1]
        return np.expand_dims(resized.astype(np.float32) / 255.0, axis=0)

    def _identify_outputs(self):
        """Identify the four SSD post-process output tensors by shape + value range.
        Returns dict keys: boxes (1,N,4), classes (1,N), scores (1,N), num (1,) | None.
        Cached after the first inference."""
        # Heuristic: boxes are the only 3D tensor with last-dim 4. Scores are 2D floats <=1.
        # Classes are 2D ints/floats >1 (or, for some exporters, float-encoded int <90).
        # num_detections is 1D.
        boxes = classes = scores = num = None
        twods = []
        for d in self.output_details:
            arr = self.interp.get_tensor(d["index"])
            if arr.ndim == 3 and arr.shape[-1] == 4:
                boxes = ("boxes", d["index"])
            elif arr.ndim == 2:
                twods.append((d["index"], arr))
            elif arr.ndim == 1 and arr.size <= 4:
                num = ("num", d["index"])
        # Disambiguate the two 2D tensors by typical max value.
        if len(twods) == 2:
            (i0, a0), (i1, a1) = twods
            m0 = float(np.abs(a0).max()) if a0.size else 0.0
            m1 = float(np.abs(a1).max()) if a1.size else 0.0
            if m0 <= 1.0001 and m1 > 1.0001:
                scores = ("scores", i0); classes = ("classes", i1)
            elif m1 <= 1.0001 and m0 > 1.0001:
                scores = ("scores", i1); classes = ("classes", i0)
            else:
                # Both look like scores or both like classes (unlikely). Fall back to
                # canonical SSDLite ordering: lower index = classes, higher = scores.
                lo, hi = sorted([i0, i1])
                classes = ("classes", lo); scores = ("scores", hi)
        return {"boxes": boxes, "classes": classes, "scores": scores, "num": num}

    def infer(self, crop_bgr):
        """Run inference on a BGR crop. Returns list of {bbox_norm, score, class_idx}.
        bbox_norm = (ymin, xmin, ymax, xmax) in [0,1] relative to the crop (NOT the full frame)."""
        tensor = self._preprocess(crop_bgr)
        self.interp.set_tensor(self.input_details[0]["index"], tensor)
        self.interp.invoke()
        idents = self._identify_outputs()
        if idents["boxes"] is None or idents["classes"] is None or idents["scores"] is None:
            log.error("Unable to identify SSD output tensors: %s",
                      [(d["name"], list(d["shape"]), str(d["dtype"])) for d in self.output_details])
            return []
        boxes = self.interp.get_tensor(idents["boxes"][1])
        classes = self.interp.get_tensor(idents["classes"][1])
        scores = self.interp.get_tensor(idents["scores"][1])
        if idents["num"] is not None:
            num_t = self.interp.get_tensor(idents["num"][1])
            n = int(num_t.flat[0]) if num_t.size > 0 else boxes.shape[1]
        else:
            n = boxes.shape[1]
        n = min(n, boxes.shape[1])
        results = []
        for i in range(n):
            score = float(scores[0, i])
            if score < self.conf_threshold:
                continue
            cls = int(classes[0, i])
            ymin, xmin, ymax, xmax = boxes[0, i]
            results.append({
                "bbox_norm": (float(ymin), float(xmin), float(ymax), float(xmax)),
                "score": score,
                "class_idx": cls,
            })
        return results

    def detect_birds(self, frame_bgr):
        """Run inference on the given BGR frame (full frame OR a crop), return only detections
        whose class matches the configured bird index. Bbox returned in pixel coords of the
        input frame (caller is responsible for translating crop -> full-frame coords)."""
        h, w = frame_bgr.shape[:2]
        out = []
        for r in self.infer(frame_bgr):
            if r["class_idx"] != self.bird_idx:
                continue
            ymin, xmin, ymax, xmax = r["bbox_norm"]
            x = max(0, int(xmin * w))
            y = max(0, int(ymin * h))
            bw = min(w - x, int((xmax - xmin) * w))
            bh = min(h - y, int((ymax - ymin) * h))
            if bw <= 0 or bh <= 0:
                continue
            out.append({"bbox": (x, y, bw, bh), "score": r["score"]})
        return out


# ------------------------- Legacy motion-only path (fallback) -------------------------

class LegacyMotionDetector:
    """Original motion-only detector. Active when bird_detection.legacy_motion_only=true.
    Behaviour identical to the pre-2026-05-16 detector — kept verbatim for emergency rollback."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.prev_frame = None
        self.last_ref_update = time.time()

    def detect_motion(self, gray):
        d = self.cfg["detection"]
        if self.prev_frame is None:
            self.prev_frame = gray
            return None
        if (time.time() - self.last_ref_update) > d["ref_frame_time_limit"]:
            log.info("Refreshing motion reference frame")
            self.prev_frame = gray
            self.last_ref_update = time.time()
            return None
        delta = cv2.absdiff(self.prev_frame, gray)
        thresh = cv2.threshold(delta, d["motion_threshold"], 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return self._largest_motion(contours)

    def _largest_motion(self, contours):
        d = self.cfg["detection"]
        if not contours:
            return None
        for c in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(c)
            if area < d["min_area"]:
                return None
            x, y, w, h = cv2.boundingRect(c)
            if d["use_shape_filtering"]:
                ar = w / h if h > 0 else 0
                if ar < d["min_aspect_ratio"] or ar > d["max_aspect_ratio"]:
                    continue
                hull_area = cv2.contourArea(cv2.convexHull(c))
                if hull_area > 0 and (area / hull_area) < d["min_solidity"]:
                    continue
            return x + w // 2, y + h // 2, (x, y, w, h), area
        return None


# ---------------------------------- Orchestrator ----------------------------------

class Detector:
    """Public detector used by controller.py. Delegates Stage 1+2 to MotionStage+BirdDetector
    in the default CNN path, or falls through to LegacyMotionDetector when configured.
    KCF tracking and is_stationary kept identical to the legacy implementation — they still
    do useful work after CNN gives us a confirmed bird bbox."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.legacy = bool(cfg.get("bird_detection", {}).get("legacy_motion_only", False))
        # Tracking state (shared by both paths)
        self.tracker = None
        self.tracking = False
        self.track_start = None
        self.track_failures = 0
        self.positions = []
        self.first_acquired = None
        self.initial_position = None
        # Compat shim — controller used to clear det.prev_frame after AE re-lock.
        # Kept so that pattern still works (legacy path uses it; CNN path ignores it).
        self.prev_frame = None

        if self.legacy:
            log.warning("Detector running in LEGACY motion-only mode (bird_detection.legacy_motion_only=true)")
            self._legacy = LegacyMotionDetector(cfg)
            self._motion = None
            self._birds = None
        else:
            self._legacy = None
            self._motion = MotionStage(cfg)
            self._birds = BirdDetector(cfg)
        self._last_full_sweep = 0.0

    # --- frame prep ---

    def process_frame(self, image):
        """Resize incoming camera frame to the configured detection width."""
        w = self.cfg["camera"]["resolution_w"]
        return imutils.resize(image, width=w)

    # --- detection (returns (cx, cy, bbox, score) | None) ---

    def detect(self, frame_bgr):
        if self.legacy:
            return self._detect_legacy(frame_bgr)
        return self._detect_cnn(frame_bgr)

    def _detect_legacy(self, frame_bgr):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        # Honour the controller's prev_frame=None reset by mirroring it onto the legacy detector.
        if self.prev_frame is None:
            self._legacy.prev_frame = None
            self.prev_frame = gray
        r = self._legacy.detect_motion(gray)
        if r is None:
            return None
        cx, cy, bbox, area = r
        return cx, cy, bbox, float(area)

    def _detect_cnn(self, frame_bgr):
        bd = self.cfg["bird_detection"]
        h_full, w_full = frame_bgr.shape[:2]
        candidates = []

        rois = self._motion.propose_rois(frame_bgr)
        pad = bd.get("roi_padding_px", 32)
        for (x, y, w, h) in rois:
            x0 = max(0, x - pad)
            y0 = max(0, y - pad)
            x1 = min(w_full, x + w + pad)
            y1 = min(h_full, y + h + pad)
            crop = frame_bgr[y0:y1, x0:x1]
            if crop.size == 0:
                continue
            for det in self._birds.detect_birds(crop):
                bx, by, bw, bh = det["bbox"]
                # Translate crop-local bbox to full-frame coordinates
                candidates.append({
                    "bbox": (x0 + bx, y0 + by, bw, bh),
                    "score": det["score"],
                    "src": "roi",
                })

        # Periodic full-frame sweep to catch stationary birds that MOG2 missed.
        sweep_interval = bd.get("full_frame_sweep_interval_seconds", 3)
        if (time.time() - self._last_full_sweep) >= sweep_interval:
            self._last_full_sweep = time.time()
            for det in self._birds.detect_birds(frame_bgr):
                candidates.append({"bbox": det["bbox"], "score": det["score"], "src": "sweep"})

        if not candidates:
            return None
        # Highest confidence wins; ties broken by area.
        best = max(candidates, key=lambda c: (c["score"], c["bbox"][2] * c["bbox"][3]))
        x, y, w, h = best["bbox"]
        cx = x + w // 2
        cy = y + h // 2
        log.info("Bird detected: src=%s score=%.2f bbox=(%d,%d,%d,%d)",
                 best["src"], best["score"], x, y, w, h)
        return cx, cy, (x, y, w, h), best["score"]

    # --- background-model reset (called by controller after AE re-lock) ---

    def reset_motion_background(self):
        if self._motion is not None:
            self._motion.reset()
        if self._legacy is not None:
            self._legacy.prev_frame = None
        self.prev_frame = None

    # --- tracking + stationary check (UNCHANGED from legacy implementation) ---

    def reset(self):
        self.tracker = None
        self.tracking = False
        self.track_start = None
        self.track_failures = 0
        self.positions = []
        self.first_acquired = None
        self.initial_position = None
        log.info("Tracking reset")

    def start_tracking(self, frame, bbox):
        self.tracker = _create_kcf_tracker()
        if self.tracker.init(frame, bbox):
            self.tracking = True
            self.track_start = time.time()
            self.first_acquired = time.time()
            self.track_failures = 0
            self.positions = []
            self.initial_position = None
            log.info("Tracker initialized at %s", bbox)
            return True
        log.warning("Tracker init failed")
        return False

    def update_tracking(self, frame):
        """Returns (center_x, center_y) on success, None on transient failure,
        or 'lost'/'timeout' string sentinels."""
        d = self.cfg["detection"]
        ok, box = self.tracker.update(frame)
        if not ok:
            self.track_failures += 1
            log.info("Track failure %d/%d", self.track_failures, d["max_track_failures"])
            if self.track_failures >= d["max_track_failures"]:
                return "lost"
            return None
        self.track_failures = 0
        if self.track_start and (time.time() - self.track_start) > d["tracking_timeout"]:
            return "timeout"
        x, y, w, h = [int(v) for v in box]
        return x + w // 2, y + h // 2

    def is_stationary(self, cx, cy):
        d = self.cfg["detection"]
        now = time.time()
        self.positions.append((cx, cy, now))
        if len(self.positions) > 10:
            self.positions.pop(0)
        if len(self.positions) < d["min_positions_for_stationary"]:
            return False
        if (now - self.first_acquired) < d["min_acquire_time"]:
            return False
        if self.initial_position is None:
            self.initial_position = (cx, cy)
        ix, iy = self.initial_position
        max_mv = 0
        for px, py, _ in self.positions:
            mv = math.sqrt((px - ix) ** 2 + (py - iy) ** 2)
            max_mv = max(max_mv, mv)
            if mv > d["stationary_threshold"]:
                log.info("Target moved %.1fpx, resetting stationary check", mv)
                self.first_acquired = now
                self.initial_position = (cx, cy)
                return False
        log.info("Target stationary (max movement %.1fpx)", max_mv)
        return True
