"""Main controller: orchestrates Pi hardware + OAK detector + state machine.

The Pi 3 is controller-only. The OAK-D-POE does camera + YOLOv8n inference +
object tracking on its Myriad X VPU and streams results here over the dedicated
Ethernet/PoE link. This loop consumes tracklets, decides when to fire, and drives
the servos + solenoid.

Operating modes (mutually exclusive, toggled from the web UI via state.kids_mode):
  - bird mode (default): target COCO class "bird", 2 s stationary dwell.
  - kids mode          : target COCO class "person", 0.5 s dwell, more generous spray.

State machine (opening hours):
  - "active"    : within opening hours -> detect + spray.
  - "off_hours" : outside opening hours -> sleep in long intervals to save power.
Water arming is an AND-gate of state.water_enabled (web UI) and state.switch_enabled
(physical switch); see state.armed.
"""
import logging
import os
import threading
import time
from datetime import datetime
from zoneinfo import ZoneInfo

from . import config, hardware, calibration
from .detector import OakDetector
from .cloud import CloudClient
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


def _switch_watcher(debounce_ms):
    """Daemon: mirrors the physical switch's current (debounced) level into
    state.switch_enabled. AND-gated with state.water_enabled (web UI)."""
    debounce_s = debounce_ms / 1000.0
    try:
        state.switch_enabled = hardware.read_switch()
    except Exception:
        pass
    candidate = state.switch_enabled
    candidate_since = time.time()
    log.info("Switch watcher started: initial switch_enabled=%s", state.switch_enabled)
    while not state.exit_flag:
        try:
            cur = hardware.read_switch()
            if cur != candidate:
                candidate = cur
                candidate_since = time.time()
            elif cur != state.switch_enabled and (time.time() - candidate_since) >= debounce_s:
                with state.lock:
                    state.switch_enabled = cur
                log.info("Physical switch -> %s (armed=%s)",
                         "ENABLED" if cur else "DISABLED", state.armed)
        except Exception as e:
            log.error("Switch watcher error: %s", e)
        time.sleep(0.02)


def _aim_and_spray(nx, ny, cal, cfg, spray_duration, sweep_enabled):
    """Map a normalized target (0..1) -> servo angles, then spray.
    Re-checks state.armed each step so switch/web Stop mid-cycle cuts the spray.

    nx/ny are normalized centroid coords from the OAK (0..1). Aim inversion is
    config-driven (oak.aim_invert_x / aim_invert_y) since the OAK has no vflip/hflip
    and its mounting orientation must be calibrated on real hardware."""
    sp = cfg["spray"]
    aim = cfg.get("aim", {})
    if aim.get("invert_x", True):
        nx = 1.0 - nx
    if aim.get("invert_y", True):
        ny = 1.0 - ny
    ax = cal["SERVO_X_MIN_ANGLE"] + nx * (cal["SERVO_X_MAX_ANGLE"] - cal["SERVO_X_MIN_ANGLE"])
    ay = cal["SERVO_Y_MIN_ANGLE"] + ny * (cal["SERVO_Y_MAX_ANGLE"] - cal["SERVO_Y_MIN_ANGLE"])
    ay += sp["water_jet_angle_offset"]
    ax = max(0, min(180, ax))
    ay = max(0, min(180, ay))
    log.info("Aiming: target=(%.3f,%.3f) -> angles=(%.1f,%.1f)", nx, ny, ax, ay)
    hardware.set_servo(hardware.SERVO_X_CHANNEL, ax)
    hardware.set_servo(hardware.SERVO_Y_CHANNEL, ay)

    if not state.armed:
        log.info("Not armed (water_enabled=%s switch_enabled=%s) — aim only, no spray",
                 state.water_enabled, state.switch_enabled)
        time.sleep(spray_duration)
        return

    log.info("Spray START (duration=%.2fs sweep=%s)", spray_duration, sweep_enabled)
    hardware.relay_on()
    state.last_spray = datetime.now().isoformat(timespec="seconds")
    try:
        if not sweep_enabled:
            # Single-point spray (kids mode default): just hold the jet on target.
            end = time.time() + spray_duration
            while time.time() < end:
                if not state.armed or state.exit_flag:
                    log.info("Spray interrupted")
                    break
                time.sleep(min(0.05, end - time.time()))
        else:
            left = max(0, ax - sp["servo_trigger_sweep"])
            right = min(180, ax + sp["servo_trigger_sweep"])
            step_sleep = spray_duration / (sp["sweep_iterations"] * 2)
            for i in range(sp["sweep_iterations"]):
                if not state.armed or state.exit_flag:
                    log.info("Spray interrupted at iteration %d", i)
                    break
                hardware.set_servo(hardware.SERVO_X_CHANNEL, left)
                if not state.armed or state.exit_flag:
                    break
                time.sleep(step_sleep)
                hardware.set_servo(hardware.SERVO_X_CHANNEL, right)
                if not state.armed or state.exit_flag:
                    break
                time.sleep(step_sleep)
    finally:
        hardware.relay_off()
        log.info("Spray END")


def run():
    cfg = config.load()
    cal = calibration.load()
    hardware.init(cfg.get("switch"))
    hardware.set_servo(hardware.SERVO_X_CHANNEL, cal["SERVO_X_CENTER"])
    hardware.set_servo(hardware.SERVO_Y_CHANNEL, cal["SERVO_Y_CENTER"])

    threading.Thread(target=_switch_watcher,
                     args=(cfg["switch"]["debounce_ms"],),
                     daemon=True).start()

    det = OakDetector(cfg)
    det.start()

    cloud = CloudClient(cfg)
    cloud.start()

    last_detection = 0
    last_image_cleanup = 0
    last_telemetry = 0
    last_stream_publish = 0.0
    was_calibrating = False

    stream_fps = cfg.get("calibration", {}).get("stream_fps", 5)
    stream_quality = cfg.get("calibration", {}).get("stream_jpeg_quality", 60)
    stream_interval = 1.0 / max(1, stream_fps)

    try:
        log.info("Main loop starting")
        while not state.exit_flag:
            # Hot-reload config if the web UI saved it.
            if config.consume_dirty():
                cfg = config.load()
                det.cfg = cfg
                cloud.report_config(cfg)
                log.info("Config reloaded")

            # Opening-hours state machine.
            active = _in_opening_hours(cfg)
            new_mode = "active" if active else "off_hours"
            if new_mode != state.mode:
                state.mode = new_mode
                log.info("Mode -> %s", new_mode)
                cloud.report_status()
            if not active:
                log.info("Off-hours: sleeping %ds", cfg["opening_hours"]["off_hours_cycle_seconds"])
                _sleep_interruptible(cfg["opening_hours"]["off_hours_cycle_seconds"])
                continue

            # Daily image cleanup.
            if time.time() - last_image_cleanup > 86400:
                _cleanup_old_images(cfg["images"]["retention_days"])
                last_image_cleanup = time.time()

            # Periodic telemetry log (Pi-side health: temp / clock / throttled).
            if (time.time() - last_telemetry) > cfg["telemetry"]["log_interval_seconds"]:
                telemetry = hardware.read_telemetry()
                log.info("Telemetry: %s", telemetry)
                cloud.report_telemetry(telemetry)
                last_telemetry = time.time()

            # Pull the latest detections + frame from the OAK.
            tracklets, frame = det.poll()

            # Publish the latest frame for the web UI MJPEG stream (rate-limited).
            now = time.time()
            if (now - last_stream_publish) >= stream_interval:
                jpeg = det.get_latest_jpeg(stream_quality)
                if jpeg is not None:
                    with state.lock:
                        state.latest_jpeg = jpeg
                        state.latest_jpeg_ts = now
                    # Push to the SPA over IoT only while someone is watching.
                    if cloud.viewer_active():
                        cloud.publish_stream_frame(jpeg)
                last_stream_publish = now

            # Calibration mode: pause detection/spray, keep stream alive.
            if state.calibrating:
                if not was_calibrating:
                    det.reset()
                    was_calibrating = True
                time.sleep(0.02)
                continue
            if was_calibrating:
                det.reset()
                was_calibrating = False

            # Cooldown between spray cycles.
            if (time.time() - last_detection) < cfg["detection"]["min_detection_interval"]:
                time.sleep(0.01)
                continue

            # Mode-dependent target selection.
            kids = state.kids_mode
            det_cfg = cfg["detection"]
            if kids:
                km = cfg["kids_mode"]
                target_label = "person"
                conf = km["person_confidence_threshold"]
                dwell = km["stationary_seconds"]
                spray_duration = km["spray_duration"]
                sweep_enabled = km["sweep_enabled"]
                post_cycle = km["post_cycle_wait_seconds"]
            else:
                target_label = "bird"
                conf = cfg["oak"]["confidence_threshold"]
                dwell = det_cfg["min_acquire_time"]
                spray_duration = cfg["spray"]["spray_duration"]
                sweep_enabled = True
                post_cycle = cfg["spray"]["post_cycle_wait_seconds"]

            target = det.select_target(tracklets, target_label, conf)
            if not target:
                time.sleep(0.01)
                continue

            det.record(target["id"], target["cx"], target["cy"])
            if det.is_stationary(target["id"],
                                 det_cfg["stationary_threshold_norm"],
                                 dwell,
                                 det_cfg["min_positions_for_stationary"]):
                log.info("%s target id=%d stable at (%.3f,%.3f) score=%.2f — firing",
                         target_label, target["id"], target["cx"], target["cy"], target["score"])
                state.last_detection = datetime.now().isoformat(timespec="seconds")
                saved_path = None
                if cfg["images"]["save_detections"] and frame is not None:
                    saved_path = _save_image(frame, target["cx"], target["cy"], cfg["images"]["jpeg_quality"])
                if saved_path:
                    cloud.upload_image(saved_path,
                                       {"label": target_label, "score": round(target["score"], 3),
                                        "cx": round(target["cx"], 3), "cy": round(target["cy"], 3)})
                _aim_and_spray(target["cx"], target["cy"], cal, cfg, spray_duration, sweep_enabled)
                last_detection = time.time()
                cloud.report_status()
                log.info("Post-cycle wait: %ss", post_cycle)
                _sleep_interruptible(post_cycle)
                det.reset()
    except KeyboardInterrupt:
        log.info("KeyboardInterrupt")
    except Exception as e:
        log.exception("Fatal error in main loop: %s", e)
    finally:
        log.info("Cleaning up")
        state.exit_flag = True
        try:
            hardware.set_servo(hardware.SERVO_X_CHANNEL, cal["SERVO_X_CENTER"])
            hardware.set_servo(hardware.SERVO_Y_CHANNEL, cal["SERVO_Y_CENTER"])
        except Exception:
            pass
        det.close()
        hardware.cleanup()
        try:
            cloud.close()
        except Exception:
            pass
        log.info("Controller stopped")


def _sleep_interruptible(seconds):
    end = time.time() + seconds
    while time.time() < end and not state.exit_flag:
        time.sleep(min(0.5, end - time.time()))


def _save_image(frame, nx, ny, quality):
    """Save a detection frame with a marker at the normalized target point.
    Returns the file path on success, else None."""
    try:
        import cv2
        h, w = frame.shape[:2]
        cx, cy = int(nx * w), int(ny * h)
        p = os.path.join(IMAGE_DIR, f"detection_{int(time.time())}.jpg")
        img = frame.copy()
        cv2.drawMarker(img, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.imwrite(p, img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return p
    except Exception as e:
        log.warning("Failed to save image: %s", e)
        return None
