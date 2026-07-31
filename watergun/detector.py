"""OAK-D-POE depth-aware detector (greenfield re-architecture, worklog §20).

Everything — RGB capture, stereo depth, YOLO inference, spatial fusion and
multi-object tracking — runs ON THE OAK DEVICE (Intel Myriad X VPU). The
Raspberry Pi just consumes spatial tracklets over the dedicated Ethernet/PoE link.

Pipeline on the OAK:

    ColorCamera (IMX378, FULL frame squashed to NN input, no crop)
        │ preview = NN input (e.g. 640x640)
        │
    MonoCamera L ─┐
                  ├─► StereoDepth (HIGH_DENSITY, aligned to RGB / CAM_A)
    MonoCamera R ─┘        │ depth map
        │                  ▼
        └────► YoloSpatialDetectionNetwork ◄── inputDepth
                  │  SpatialImgDetections: bbox + label + confidence + XYZ (mm)
                  ▼
               ObjectTracker (on-device persistent IDs; spatial coords on tracklets)
                  │  Tracklets ──► XLinkOut ──► Pi
                  ▼

Why spatial (vs the old RGB-only YoloDetectionNetwork):
  - Every track carries a real 3D position (X/Y/Z in millimetres), so the
    controller can gate on a distance BAND (reject the fence/cat behind the tree
    and the grass in front) and, once calibrated, range-compensate the water jet.
  - Full frame (no crop) so the SAME pipeline serves bird mode (class 'bird') and
    kids mode (class 'person', who can be anywhere in view).

COCO class indices are the YOLO 80-class scheme (person=0, bird=14). We resolve
target classes BY NAME against COCO_LABELS, so the numeric index is never
hand-maintained.

Spatial coordinate system (OAK, left-handed Cartesian, millimetres):
  X = right(+)/left(-) of camera centre, Y = up(+)/down(-), Z = forward distance.
"""
import logging
import math
import time
from collections import defaultdict, deque

log = logging.getLogger(__name__)

# depthai + cv2 are imported lazily in OakDetector.start() so this module can be
# imported (and py_compiled) on a dev machine without the libraries installed.
dai = None
cv2 = None

# COCO 80-class labels in the order YOLOv8 uses. Index = class id.
COCO_LABELS = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
    "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

_MONO_RES = {
    "400": "THE_400_P",
    "720": "THE_720_P",
    "800": "THE_800_P",
}


def label_id(name):
    """Resolve a COCO class name to its YOLO 80-class index, or None if unknown."""
    try:
        return COCO_LABELS.index(name)
    except ValueError:
        return None


class OakDetector:
    """Owns the depthai spatial pipeline + device connection and the per-track
    stationarity bookkeeping. Public API used by the controller:

        start()                         -> connect to OAK, open output queues
        poll()                          -> (tracklets, frame_bgr_or_None)
        select_target(tracklets, ...)   -> best matching tracklet dict or None
        record(track_id, cx, cy)        -> append a centroid sample
        is_stationary(track_id, ...)    -> bool
        reset()                         -> clear stationarity history
        get_latest_jpeg(quality)        -> bytes or None (web MJPEG stream)
        close()

    Each tracklet dict:
        {id, label_id, label, score, cx, cy, bbox,      # cx/cy/bbox normalized 0..1
         x_mm, y_mm, z_mm, dist_mm}                      # spatial, millimetres
    z_mm is forward distance; dist_mm is euclidean range. Both are 0 when the OAK
    could not resolve depth for that ROI (out of stereo range / no disparity).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self._device = None
        self._q_track = None
        self._q_frame = None
        self._latest_frame = None          # most recent BGR np.ndarray from the OAK
        # track_id -> deque[(t, cx, cy)] of recent normalized centroids
        self._history = defaultdict(lambda: deque(maxlen=60))

    # ----- lifecycle -------------------------------------------------------

    def start(self):
        global dai, cv2
        import depthai as _dai
        import cv2 as _cv2
        dai, cv2 = _dai, _cv2

        oak = self.cfg["oak"]
        pipeline = self._build_pipeline(oak)

        device_ip = oak.get("device_ip")
        if device_ip:
            info = dai.DeviceInfo(device_ip)
            log.info("Connecting to OAK at %s", device_ip)
            self._device = dai.Device(pipeline, info)
        else:
            log.info("Connecting to OAK via auto-discovery (no device_ip set)")
            self._device = dai.Device(pipeline)

        self._q_track = self._device.getOutputQueue("tracklets", maxSize=4, blocking=False)
        self._q_frame = self._device.getOutputQueue("preview", maxSize=4, blocking=False)
        log.info("OakDetector ready (spatial): model=%s input=%dx%d conf=%.2f "
                 "depth_band=[%s,%s]mm bbox_scale=%.2f tracker_labels=%s",
                 oak["model_blob"], oak["model_input_w"], oak["model_input_h"],
                 oak["confidence_threshold"], oak.get("depth_lower_threshold_mm"),
                 oak.get("depth_upper_threshold_mm"), oak.get("bounding_box_scale_factor", 0.5),
                 oak.get("tracker_labels"))

    def _build_pipeline(self, oak):
        pipeline = dai.Pipeline()

        # --- RGB camera: full frame squashed to NN input (no crop) ---
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(oak["model_input_w"], oak["model_input_h"])
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(oak.get("fps", 30))
        cam.setPreviewKeepAspectRatio(False)  # use the whole FOV, don't crop

        # --- Stereo pair -> depth, aligned to the RGB camera ---
        mono_res = _MONO_RES.get(str(oak.get("mono_resolution", "400")), "THE_400_P")
        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_l.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, mono_res))
        mono_l.setCamera("left")
        mono_r.setResolution(getattr(dai.MonoCameraProperties.SensorResolution, mono_res))
        mono_r.setCamera("right")

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # Align depth to the RGB (CAM_A) frame the detector runs on.
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setLeftRightCheck(oak.get("stereo_lr_check", True))
        stereo.setSubpixel(oak.get("stereo_subpixel", False))
        stereo.setOutputSize(mono_l.getResolutionWidth(), mono_l.getResolutionHeight())

        # --- Spatial YOLO detection network ---
        nn = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        nn.setBlobPath(oak["model_blob"])
        nn.setConfidenceThreshold(oak["confidence_threshold"])
        nn.input.setBlocking(False)
        # Spatial params: scale bbox down so depth ROI stays on the object, and
        # clamp the depth range the calculator considers valid.
        nn.setBoundingBoxScaleFactor(oak.get("bounding_box_scale_factor", 0.5))
        nn.setDepthLowerThreshold(int(oak.get("depth_lower_threshold_mm", 100)))
        nn.setDepthUpperThreshold(int(oak.get("depth_upper_threshold_mm", 20000)))
        # Yolo params (YOLOv8 is anchor-free: empty anchors + masks).
        nn.setNumClasses(oak.get("num_classes", 80))
        nn.setCoordinateSize(oak.get("coord_size", 4))
        nn.setAnchors(oak.get("anchors", []))
        nn.setAnchorMasks(oak.get("anchor_masks", {}))
        nn.setIouThreshold(oak.get("iou_threshold", 0.5))
        nn.setNumInferenceThreads(2)

        # --- Object tracker (on-device persistent IDs) ---
        tracker = pipeline.create(dai.node.ObjectTracker)
        track_ids = []
        for nm in oak.get("tracker_labels", ["bird", "person"]):
            lid = label_id(nm)
            if lid is not None:
                track_ids.append(lid)
        if track_ids:
            tracker.setDetectionLabelsToTrack(track_ids)
        tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        # --- Linking (mirrors the canonical spatial_object_tracker example) ---
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)
        cam.preview.link(nn.input)
        stereo.depth.link(nn.inputDepth)
        nn.passthrough.link(tracker.inputTrackerFrame)
        nn.passthrough.link(tracker.inputDetectionFrame)
        nn.out.link(tracker.inputDetections)

        xout_track = pipeline.create(dai.node.XLinkOut)
        xout_track.setStreamName("tracklets")
        tracker.out.link(xout_track.input)

        xout_frame = pipeline.create(dai.node.XLinkOut)
        xout_frame.setStreamName("preview")
        tracker.passthroughTrackerFrame.link(xout_frame.input)

        return pipeline

    def close(self):
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None

    # ----- per-frame consumption ------------------------------------------

    def poll(self):
        """Non-blocking. Returns (tracklets, frame_bgr_or_None).

        tracklets: list of dicts for ACTIVE (NEW/TRACKED) tracks, with normalized
        centroid/bbox AND spatial coordinates (mm)."""
        frame = None
        in_frame = self._q_frame.tryGet() if self._q_frame is not None else None
        if in_frame is not None:
            frame = in_frame.getCvFrame()
            self._latest_frame = frame

        tracklets = []
        in_track = self._q_track.tryGet() if self._q_track is not None else None
        if in_track is not None:
            for t in in_track.tracklets:
                status = str(t.status.name) if hasattr(t.status, "name") else str(t.status)
                if status not in ("NEW", "TRACKED"):
                    continue
                roi = t.roi  # normalized Rect 0..1
                cx = roi.x + roi.width / 2.0
                cy = roi.y + roi.height / 2.0
                lid = int(t.label)
                sc = t.spatialCoordinates
                x_mm, y_mm, z_mm = float(sc.x), float(sc.y), float(sc.z)
                dist_mm = math.sqrt(x_mm * x_mm + y_mm * y_mm + z_mm * z_mm) if z_mm > 0 else 0.0
                try:
                    score = float(t.srcImgDetection.confidence)
                except Exception:
                    score = 0.0
                tracklets.append({
                    "id": int(t.id),
                    "label_id": lid,
                    "label": COCO_LABELS[lid] if 0 <= lid < len(COCO_LABELS) else str(lid),
                    "score": score,
                    "cx": cx,
                    "cy": cy,
                    "bbox": (roi.x, roi.y, roi.x + roi.width, roi.y + roi.height),
                    "x_mm": x_mm,
                    "y_mm": y_mm,
                    "z_mm": z_mm,
                    "dist_mm": dist_mm,
                })
        return tracklets, frame

    # ----- target selection + gating + stationarity -----------------------

    @staticmethod
    def in_depth_band(tracklet, min_mm, max_mm):
        """True if the track's forward distance (z) is within [min_mm, max_mm].

        Unknown depth (z_mm <= 0, i.e. the OAK couldn't resolve disparity for the
        ROI) is treated as PASS — we don't drop a real target just because stereo
        dropped out. Tighten by requiring depth in the controller if needed."""
        z = tracklet.get("z_mm", 0.0)
        if z <= 0:
            return True
        if min_mm is not None and z < min_mm:
            return False
        if max_mm is not None and z > max_mm:
            return False
        return True

    def select_target(self, tracklets, target_label, conf_threshold,
                       min_dist_mm=None, max_dist_mm=None):
        """Pick the highest-confidence tracklet matching target_label above
        threshold AND within the depth band. Returns the tracklet dict or None."""
        candidates = [t for t in tracklets
                      if t["label"] == target_label
                      and t["score"] >= conf_threshold
                      and self.in_depth_band(t, min_dist_mm, max_dist_mm)]
        if not candidates:
            return None
        return max(candidates, key=lambda t: t["score"])

    def record(self, track_id, cx, cy):
        """Append a normalized centroid sample for a track id."""
        self._history[track_id].append((time.time(), cx, cy))

    def is_stationary(self, track_id, threshold_norm, dwell_s, min_positions):
        """True if this track's recent centroids stayed within threshold_norm
        (fraction of frame) for at least dwell_s seconds across >= min_positions samples."""
        hist = self._history.get(track_id)
        if not hist or len(hist) < min_positions:
            return False
        now = time.time()
        window = [(t, x, y) for (t, x, y) in hist if now - t <= dwell_s + 1.0]
        if len(window) < min_positions:
            return False
        if window[-1][0] - window[0][0] < dwell_s:
            return False
        xs = [x for (_, x, _) in window]
        ys = [y for (_, _, y) in window]
        if (max(xs) - min(xs)) > threshold_norm:
            return False
        if (max(ys) - min(ys)) > threshold_norm:
            return False
        return True

    def reset(self):
        """Clear all stationarity history (e.g. after a spray cycle or calibration)."""
        self._history.clear()

    def forget(self, track_id):
        self._history.pop(track_id, None)

    # ----- web stream ------------------------------------------------------

    def get_latest_jpeg(self, quality=60):
        """Encode the most recent OAK preview frame as JPEG bytes, or None."""
        if self._latest_frame is None or cv2 is None:
            return None
        try:
            ok, buf = cv2.imencode(".jpg", self._latest_frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
            return buf.tobytes() if ok else None
        except Exception as e:
            log.warning("JPEG encode failed: %s", e)
            return None
