"""Main controller: orchestrates hardware + detector + state machine.

State machine:
  - "active"     : within opening hours -> fast detection cycle
  - "off_hours"  : outside opening hours -> sleep long intervals
  - "stopped"    : water disabled (via switch or web UI)  [note: this is just water_enabled=False]
"""
import cv2
import logging
import os
import threading
import time
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo

from . import config, hardware, calibration
from .detector import Detector
from .state import state

log = logging.getLogger(__name__)

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMAGE_DIR = os.path.join(BASE_DIR, "motion_images")
os.makedirs(IMAGE_DIR, exist_ok=True)


def _in_opening_hours(cfg):
    oh = cfg["opening_hours"]
    tz = ZoneInfo(oh["timezone"])
    now = datetime.now(tz).time()
    sh, sm = [int(x) for x in oh["start"].split(":")]
    eh, em = [int(x) for x in oh["end"].split(":")]
    from datetime import time as dtime
    return dtime(sh, sm) <= now < dtime(eh, em)


def _cleanup_old_images(retention_days):
    cutoff = time.time() - retention_days * 86400
    removed = 0
    try:
        for f in os.listdir(IMAGE_DIR):
            p = os.path.join(IMAGE_DIR, f)
            if os.path.isfile(p) and os.path.getmtime(p) < cutoff:
                os.remove(p)
                removed += 1
    except Exception as e:
        log.warning("Image cleanup error: %s", e)
    if removed:
        log.info("Deleted %d images older than %dd", removed, retention_days)


def _switch_watcher():
    """Daemon: toggles state.water_enabled on physical switch press (HIGH -> LOW)."""
    prev = 1
    while not state.exit_flag:
        try:
            cur = hardware.read_switch()
            if prev == 1 and cur == 0:
                with state.lock:
                    state.water_enabled = not state.water_enabled
                log.info("Physical switch: water %s", "ENABLED" if state.water_enabled else "DISABLED")
                # wait for release
                while hardware.read_switch() == 0 and not state.exit_flag:
                    time.sleep(0.05)
                time.sleep(0.2)
            prev = cur
        except Exception as e:
            log.error("Switch watcher error: %s", e)
        time.sleep(0.05)


def _aim_and_spray(cx, cy, cal, cfg):
    """Map target -> servo angles, spray with sweep. Re-checks water_enabled each step so
    switch/web Stop during a cycle cuts the spray immediately."""
    sp = cfg["spray"]
    cam = cfg["camera"]
    # Keep original inversion (per Mike: current aim works)
    nx = 1.0 - (cx / cam["resolution_w"])
    ny = 1.0 - (cy / cam["resolution_h"])
    ax = cal["SERVO_X_MIN_ANGLE"] + nx * (cal["SERVO_X_MAX_ANGLE"] - cal["SERVO_X_MIN_ANGLE"])
    ay = cal["SERVO_Y_MIN_ANGLE"] + ny * (cal["SERVO_Y_MAX_ANGLE"] - cal["SERVO_Y_MIN_ANGLE"])
    ay += sp["water_jet_angle_offset"]
    ax = max(0, min(180, ax))
    ay = max(0, min(180, ay))
    log.info("Aiming: target=(%d,%d) -> angles=(%.1f,%.1f)", cx, cy, ax, ay)
    hardware.set_servo(hardware.SERVO_X_CHANNEL, ax)
    hardware.set_servo(hardware.SERVO_Y_CHANNEL, ay)

    if not state.water_enabled:
        log.info("Water disabled — aim only, no spray")
        time.sleep(sp["spray_duration"])
        return

    log.info("Spray START")
    hardware.relay_on()
    state.last_spray = datetime.now().isoformat(timespec="seconds")
    left = max(0, ax - sp["servo_trigger_sweep"])
    right = min(180, ax + sp["servo_trigger_sweep"])
    step_sleep = sp["spray_duration"] / (sp["sweep_iterations"] * 2)
    try:
        for i in range(sp["sweep_iterations"]):
            if not state.water_enabled or state.exit_flag:
                log.info("Spray interrupted at iteration %d", i)
                break
            hardware.set_servo(hardware.SERVO_X_CHANNEL, left)
            if not state.water_enabled or state.exit_flag:
                break
            time.sleep(step_sleep)
            hardware.set_servo(hardware.SERVO_X_CHANNEL, right)
            if not state.water_enabled or state.exit_flag:
                break
            time.sleep(step_sleep)
    finally:
        hardware.relay_off()
        log.info("Spray END")


def run():
    cfg = config.load()
    cal = calibration.load()
    cam, raw = hardware.init(cfg["camera"])
    hardware.set_servo(hardware.SERVO_X_CHANNEL, cal["SERVO_X_CENTER"])
    hardware.set_servo(hardware.SERVO_Y_CHANNEL, cal["SERVO_Y_CENTER"])

    threading.Thread(target=_switch_watcher, daemon=True).start()

    det = Detector(cfg)
    last_detection = 0
    last_image_cleanup = 0
    last_opening_hours_log = None

    try:
        log.info("Main loop starting")
        for frame in cam.capture_continuous(raw, format="bgr", use_video_port=True):
            if state.exit_flag:
                break
            # Hot-reload config if web UI saved it
            if config.consume_dirty():
                cfg = config.load()
                det.cfg = cfg
                log.info("Config reloaded")

            # Opening hours state machine
            active = _in_opening_hours(cfg)
            new_mode = "active" if active else "off_hours"
            if new_mode != state.mode:
                state.mode = new_mode
                log.info("Mode -> %s", new_mode)
            if not active:
                raw.truncate(0); raw.seek(0)
                log.info("Off-hours: sleeping %ds", cfg["opening_hours"]["off_hours_cycle_seconds"])
                _sleep_interruptible(cfg["opening_hours"]["off_hours_cycle_seconds"])
                continue

            # Daily image cleanup
            if time.time() - last_image_cleanup > 86400:
                _cleanup_old_images(cfg["images"]["retention_days"])
                last_image_cleanup = time.time()

            image = frame.array
            resized, gray = det.process_frame(image)
            raw.truncate(0); raw.seek(0)

            if det.tracking:
                r = det.update_tracking(resized)
                if r in ("lost", "timeout"):
                    log.info("Tracking ended: %s", r)
                    det.reset()
                    continue
                if r is None:
                    continue
                cx, cy = r
                if det.is_stationary(cx, cy):
                    log.info("Target stable at (%d,%d) — firing", cx, cy)
                    state.last_detection = datetime.now().isoformat(timespec="seconds")
                    if cfg["images"]["save_detections"]:
                        _save_image(resized, cx, cy, cfg["images"]["jpeg_quality"])
                    _aim_and_spray(cx, cy, cal, cfg)
                    last_detection = time.time()
                    log.info("Post-cycle wait: %ss", cfg["spray"]["post_cycle_wait_seconds"])
                    _sleep_interruptible(cfg["spray"]["post_cycle_wait_seconds"])
                    det.reset()
            else:
                # Cooldown between detections
                if (time.time() - last_detection) < cfg["detection"]["min_detection_interval"]:
                    continue
                motion = det.detect(gray)
                if motion:
                    cx, cy, bbox, area = motion
                    log.info("Motion: (%d,%d) area=%d", cx, cy, area)
                    det.start_tracking(resized, bbox)
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt")
    except Exception as e:
        log.exception("Fatal error in main loop: %s", e)
    finally:
        log.info("Cleaning up")
        state.exit_flag = True
        hardware.set_servo(hardware.SERVO_X_CHANNEL, cal["SERVO_X_CENTER"])
        hardware.set_servo(hardware.SERVO_Y_CHANNEL, cal["SERVO_Y_CENTER"])
        try:
            cam.close()
        except Exception:
            pass
        hardware.cleanup()
        log.info("Controller stopped")


def _sleep_interruptible(seconds):
    end = time.time() + seconds
    while time.time() < end and not state.exit_flag:
        time.sleep(min(0.5, end - time.time()))


def _save_image(frame, cx, cy, quality):
    try:
        import cv2
        p = os.path.join(IMAGE_DIR, f"motion_{int(time.time())}.jpg")
        img = frame.copy()
        cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
        cv2.imwrite(p, img, [cv2.IMWRITE_JPEG_QUALITY, quality])
    except Exception as e:
        log.warning("Failed to save image: %s", e)
