"""GPIO + servo wrappers for the Pi 3 controller.

The Pi 3 is now controller-only: it owns the servos (PCA9685), the water solenoid
relay (GPIO17), and the physical arming switch (GPIO27). All camera + inference work
lives on the OAK-D-POE (see watergun/detector.py), so there is no PiCamera, no
auto-exposure handling, and no image capture here anymore.
"""
import logging
import time

log = logging.getLogger(__name__)

# Imported lazily inside init() so this module loads on dev machines.
GPIO = None
ServoKit = None

RELAY_PIN = 17
EXIT_SWITCH_PIN = 27       # default, overridden by config in init()
SWITCH_ACTIVE_LOW = True   # default, overridden by config in init()
SERVO_X_CHANNEL = 0
SERVO_Y_CHANNEL = 1
MIN_IMP = [500, 500]
MAX_IMP = [2500, 2500]
MIN_ANG = [0, 0]
MAX_ANG = [180, 180]

_pca = None


def init(switch_cfg=None):
    """Set up GPIO + servos. No camera — the OAK handles imaging."""
    global GPIO, ServoKit, _pca, EXIT_SWITCH_PIN, SWITCH_ACTIVE_LOW
    import RPi.GPIO as _gpio
    from adafruit_servokit import ServoKit as _sk
    GPIO, ServoKit = _gpio, _sk

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
    log.info("Hardware initialized: servos + relay (pin %d) + switch (pin %d, active_low=%s)",
             RELAY_PIN, EXIT_SWITCH_PIN, SWITCH_ACTIVE_LOW)


def set_servo(channel, angle):
    angle = max(MIN_ANG[channel], min(MAX_ANG[channel], angle))
    _pca.servo[channel].angle = angle
    time.sleep(0.3)


def relay_on():
    GPIO.output(RELAY_PIN, GPIO.HIGH)


def relay_off():
    GPIO.output(RELAY_PIN, GPIO.LOW)


def short_burst(duration_s):
    """Open the relay for duration_s seconds, then close. Used by the calibration
    'Fire test shot' endpoint. Safety: caller is responsible for gating this (see
    web.py — only callable while state.calibrating=True)."""
    duration_s = max(0.05, min(2.0, float(duration_s)))  # clamp 50ms..2s
    log.info("Calibration short burst: %.2fs", duration_s)
    try:
        relay_on()
        time.sleep(duration_s)
    finally:
        relay_off()


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


def read_telemetry():
    """Cheap vcgencmd reads: core temp, arm clock, throttled flags. ~1ms total.
    Kept even though the OAK is PoE-powered — still useful for spotting Pi thermal
    or under-voltage issues on the controller itself."""
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
