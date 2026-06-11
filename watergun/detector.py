"""OAK-D-POE detector.

All camera capture + inference + multi-object tracking happens ON THE OAK DEVICE
(Intel Myriad X VPU). The Raspberry Pi just consumes the results over the dedicated
Ethernet/PoE link.

Pipeline on the OAK:

    ColorCamera (RGB, IMX378)
        │  preview = NN input size (e.g. 640x640)
        ▼
    YoloDetectionNetwork (YOLOv8n, COCO 80-class .blob)
        │  ImgDetections (normalized bboxes + label + confidence)
        ▼
    ObjectTracker (on-device, persistent track IDs)
        │  Tracklets
        ▼
    XLinkOut ──► Pi  (tracklets queue + passthrough preview frame queue)

Design notes / why this is so much simpler than the old Pi-3 TFLite path:
  - No MOG2 motion gating: the VPU runs the full-frame detector at 30+ FPS, so we
    don't need to pre-filter regions on the CPU.
  - No cv2 KCF/CSRT tracker: the OAK's ObjectTracker assigns stable IDs on-device.
    The "is it stationary?" check just watches one track ID's centroid over time.
  - No AE-lock / AWB juggling: the OAK ISP manages its own exposure.
  - Detections are normalized 0..1 relative to the NN input, so aim mapping is
    resolution-independent (no pixel math tied to a specific capture size).

COCO class indices are the YOLO 80-class scheme (person=0, bird=14) — NOT the old
90-class TFLite labels file. We resolve target classes BY NAME against COCO_LABELS
below, so the numeric index never has to be hand-maintained.
"""
import logging
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


def label_id(name):
    """Resolve a COCO class name to its YOLO 80-class index, or None if unknown."""
    try:
        return COCO_LABELS.index(name)
    except ValueError:
        return None


class OakDetector:
    """Owns the depthai pipeline + device connection and the per-track stationarity
    bookkeeping. Public API used by the controller:

        start()                         -> connect to OAK, open output queues
        poll()                          -> (tracklets, frame_bgr_or_None)
        select_target(tracklets, ...)   -> best matching tracklet dict or None
        is_stationary(track_id, cx, cy, dwell_s) -> bool
        reset()                         -> clear stationarity history
        get_latest_jpeg(quality)        -> bytes or None  (for web MJPEG stream)
        close()
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
        log.info("OakDetector ready: model=%s input=%dx%d conf=%.2f tracker_labels=%s",
                 oak["model_blob"], oak["model_input_w"], oak["model_input_h"],
                 oak["confidence_threshold"], oak.get("tracker_labels"))

    def _build_pipeline(self, oak):
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setPreviewSize(oak["model_input_w"], oak["model_input_h"])
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setFps(oak.get("fps", 30))
        cam.setPreviewKeepAspectRatio(False)

        nn = pipeline.create(dai.node.YoloDetectionNetwork)
        nn.setBlobPath(oak["model_blob"])
        nn.setConfidenceThreshold(oak["confidence_threshold"])
        nn.setNumClasses(oak.get("num_classes", 80))
        nn.setCoordinateSize(oak.get("coord_size", 4))
        # YOLOv8 is anchor-free: empty anchors + masks.
        nn.setAnchors(oak.get("anchors", []))
        nn.setAnchorMasks(oak.get("anchor_masks", {}))
        nn.setIouThreshold(oak.get("iou_threshold", 0.5))
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)
        cam.preview.link(nn.input)

        tracker = pipeline.create(dai.node.ObjectTracker)
        # Only track the classes we care about (resolved from names -> ids).
        track_ids = []
        for nm in oak.get("tracker_labels", ["bird", "person"]):
            lid = label_id(nm)
            if lid is not None:
                track_ids.append(lid)
        if track_ids:
            tracker.setDetectionLabelsToTrack(track_ids)
        tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        nn.passthrough.link(tracker.inputTrackerFrame)
        nn.passthrough.link(tracker.inputDetectionFrame)
        nn.out.link(tracker.inputDetections)

        xout_track = pipeline.create(dai.node.XLinkOut)
        xout_track.setStreamName("tracklets")
        tracker.out.link(xout_track.input)

        xout_frame = pipeline.create(dai.node.XLinkOut)
        xout_frame.setStreamName("preview")
        # Passthrough frame is aligned with the detections shown on it.
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

        tracklets: list of dicts for ACTIVE (NEW/TRACKED) tracks:
            {id, label_id, label, score, cx, cy, bbox}
            cx/cy and bbox are normalized 0..1 (center + (xmin,ymin,xmax,ymax)).
        """
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
                lid = int(t.srcImgDetection.label)
                tracklets.append({
                    "id": int(t.id),
                    "label_id": lid,
                    "label": COCO_LABELS[lid] if 0 <= lid < len(COCO_LABELS) else str(lid),
                    "score": float(t.srcImgDetection.confidence),
                    "cx": cx,
                    "cy": cy,
                    "bbox": (roi.x, roi.y, roi.x + roi.width, roi.y + roi.height),
                })
        return tracklets, frame

    # ----- target selection + stationarity --------------------------------

    def select_target(self, tracklets, target_label, conf_threshold):
        """Pick the highest-confidence tracklet matching target_label above threshold.
        Returns the tracklet dict or None."""
        candidates = [t for t in tracklets
                      if t["label"] == target_label and t["score"] >= conf_threshold]
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
        # Must span at least dwell_s of wall-clock time.
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
