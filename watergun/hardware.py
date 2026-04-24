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
EXIT_SWITCH_PIN = 27
SERVO_X_CHANNEL = 0
SERVO_Y_CHANNEL = 1
MIN_IMP = [500, 500]
MAX_IMP = [2500, 2500]
MIN_ANG = [0, 0]
MAX_ANG = [180, 180]

_pca = None


def init(camera_cfg):
    """Set up GPIO, servos, and return a camera + raw_capture pair."""
    global GPIO, ServoKit, PiCamera, PiRGBArray, _pca
    import RPi.GPIO as _gpio
    from adafruit_servokit import ServoKit as _sk
    from picamera import PiCamera as _pc
    from picamera.array import PiRGBArray as _pra
    GPIO, ServoKit, PiCamera, PiRGBArray = _gpio, _sk, _pc, _pra

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RELAY_PIN, GPIO.OUT)
    GPIO.output(RELAY_PIN, GPIO.LOW)
    GPIO.setup(EXIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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
    return GPIO.input(EXIT_SWITCH_PIN)


def cleanup():
    try:
        relay_off()
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass
