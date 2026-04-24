"""GPIO + servo + camera wrappers. Isolates hardware so other modules stay testable."""
import logging
import time

log = logging.getLogger(__name__)

# These may be unavailable on dev machines; import lazily inside init.
GPIO = None
ServoKit = None
PiCamera = None
PiRGBArray = None

RELAY_PIN = 17
EXIT_SWITCH_PIN = 27  # default, overridden by config in init()
SWITCH_ACTIVE_LOW = True  # default, overridden by config in init()
SERVO_X_CHANNEL = 0
SERVO_Y_CHANNEL = 1
MIN_IMP = [500, 500]
MAX_IMP = [2500, 2500]
MIN_ANG = [0, 0]
MAX_ANG = [180, 180]

_pca = None


def init(camera_cfg, switch_cfg=None):
    """Set up GPIO, servos, and return a camera + raw_capture pair."""
    global GPIO, ServoKit, PiCamera, PiRGBArray, _pca, EXIT_SWITCH_PIN, SWITCH_ACTIVE_LOW
    import RPi.GPIO as _gpio
    from adafruit_servokit import ServoKit as _sk
    from picamera import PiCamera as _pc
    from picamera.array import PiRGBArray as _pra
    GPIO, ServoKit, PiCamera, PiRGBArray = _gpio, _sk, _pc, _pra

    if switch_cfg is not None:
        EXIT_SWITCH_PIN = switch_cfg["pin"]
        SWITCH_ACTIVE_LOW = switch_cfg["active_low"]

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    # Pull-up for active-low (switch closes to ground), pull-down for active-high.
    pud = GPIO.PUD_UP if SWITCH_ACTIVE_LOW else GPIO.PUD_DOWN
    GPIO.setup(EXIT_SWITCH_PIN, GPIO.IN, pull_up_down=pud)

    _pca = ServoKit(channels=16)
    _pca.servo[SERVO_X_CHANNEL].set_pulse_width_range(MIN_IMP[0], MAX_IMP[0])
    _pca.servo[SERVO_Y_CHANNEL].set_pulse_width_range(MIN_IMP[1], MAX_IMP[1])

    res = (camera_cfg["resolution_w"], camera_cfg["resolution_h"])
    cam = PiCamera()
    cam.resolution = res
    cam.framerate = camera_cfg["framerate"]
    cam.vflip = camera_cfg["vflip"]
    cam.hflip = camera_cfg["hflip"]
    raw = PiRGBArray(cam, size=res)
    time.sleep(2)  # warm-up
    log.info("Hardware initialized: camera=%s fps=%d", res, camera_cfg["framerate"])
    return cam, raw


def set_servo(channel, angle):
    angle = max(MIN_ANG[channel], min(MAX_ANG[channel], angle))
    _pca.servo[channel].angle = angle
    time.sleep(0.3)


def relay_on():
    GPIO.output(RELAY_PIN, GPIO.HIGH)


def relay_off():
    GPIO.output(RELAY_PIN, GPIO.LOW)


def read_switch():
    """Return True when the physical switch is in the 'armed' position.
    active_low: LOW reading (0) means armed. active_high: HIGH (1) means armed.
    """
    level = GPIO.input(EXIT_SWITCH_PIN)
    return (level == 0) if SWITCH_ACTIVE_LOW else (level == 1)


def cleanup():
    try:
        relay_off()
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass


def relock_camera_exposure(cam, settle_seconds=3):
    """Let camera auto-meter briefly, then lock exposure + AWB to current values.
    Re-callable: flips modes back to auto, settles, then re-locks. Returns dict of locked values."""
    try:
        cam.exposure_mode = "auto"
        cam.awb_mode = "auto"
        time.sleep(settle_seconds)
        ss = cam.exposure_speed  # read auto-metered value (us)
        gains = cam.awb_gains    # (red, blue) Fractions
        cam.shutter_speed = ss
        cam.awb_mode = "off"
        cam.awb_gains = gains
        cam.exposure_mode = "off"
        locked = {
            "shutter_us": ss,
            "awb_red": float(gains[0]),
            "awb_blue": float(gains[1]),
            "iso": cam.iso,
            "analog_gain": float(cam.analog_gain),
            "digital_gain": float(cam.digital_gain),
        }
        log.info("Camera AE re-locked: %s", locked)
        return locked
    except Exception as e:
        log.warning("AE re-lock failed: %s", e)
        return None


def read_telemetry():
    """Cheap vcgencmd reads: core temp, arm clock, throttled flags. ~1ms total."""
    import subprocess
    def _run(args):
        try:
            return subprocess.check_output(args, timeout=1).decode().strip()
        except Exception:
            return "?"
    return {
        "temp": _run(["vcgencmd", "measure_temp"]).replace("temp=", ""),
        "arm_hz": _run(["vcgencmd", "measure_clock", "arm"]).split("=")[-1],
        "throttled": _run(["vcgencmd", "get_throttled"]).replace("throttled=", ""),
    }
