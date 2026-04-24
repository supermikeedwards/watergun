"""Motion detection + KCF tracking + stationary check. No cv2 UI calls — headless."""
import cv2
import math
import time
import logging
import imutils

log = logging.getLogger(__name__)


class Detector:
    def __init__(self, cfg):
        self.cfg = cfg
        self.prev_frame = None
        self.last_ref_update = time.time()
        self.tracker = None
        self.tracking = False
        self.track_start = None
        self.track_failures = 0
        self.positions = []
        self.first_acquired = None
        self.initial_position = None

    def reset(self):
        self.tracker = None
        self.tracking = False
        self.track_start = None
        self.track_failures = 0
        self.positions = []
        self.first_acquired = None
        self.initial_position = None
        log.info("Tracking reset")

    def process_frame(self, image):
        w = self.cfg["camera"]["resolution_w"]
        frame = imutils.resize(image, width=w)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)
        return frame, gray

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

    def detect(self, gray):
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

    def start_tracking(self, frame, bbox):
        self.tracker = cv2.legacy.TrackerKCF_create()
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
        """Returns (center_x, center_y) on success, None on failure, or 'timeout'/'lost' string sentinels."""
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
