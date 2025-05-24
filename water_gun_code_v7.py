#!/usr/bin/env python3

# Required packages
# pip install opencv-python-headless
# pip install picamera
# pip install RPi.GPIO
# pip install adafruit-circuitpython-servokit
# pip install imutils

"""
offset to fire higher
escape doesn't work well
targeting is inverted, aims left instead of right
"""

"""

 #   r: Reset tracking and look for new targets
 #   c: Enter servo calibration mode
 #   h: Return servos to home/center position
 #   ESC: Exit the program

Step 1: Center Position Calibration

    Use the A/D keys to adjust the X (horizontal) center position
    Use the W/S keys to adjust the Y (vertical) center position
    Press SPACE to save and continue to the next step
    Press ESC to cancel calibration

Step 2: X Range Calibration

    Use the A/D keys to adjust the minimum X angle
    Use the Q/E keys to adjust the maximum X angle
    Press SPACE to save and continue to the next step
    Press ESC to cancel calibration

Step 3: Y Range Calibration

    Use the W/S keys to adjust the minimum Y angle
    Use the R/F keys to adjust the maximum Y angle
    Press SPACE to save and complete the calibration
    Press ESC to cancel calibration

After completing all steps, the calibration will be saved to the file specified by CALIBRATION_FILE and the servos will return to their center positions.
Tips for Effective Calibration

    Start by setting the center position to where you want the water gun to aim when at rest.
    For the X range, move to the leftmost position you want the gun to be able to aim, then the rightmost.
    For the Y range, move to the lowest position you want the gun to aim, then the highest.
    Make small adjustments (the code moves in increments of 2 degrees) and test thoroughly.
    If you make a mistake, you can press ESC to cancel and start over.


"""


# Standard imports
import cv2
import numpy as np
import time
import datetime
import RPi.GPIO as GPIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import threading
from adafruit_servokit import ServoKit
import imutils
import os
import logging
import math

# Define paths and create directories
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IMAGE_FOLDER = os.path.join(BASE_DIR, "motion_images")
LOG_FOLDER = os.path.join(BASE_DIR, "logs")
CALIBRATION_FILE = os.path.join(BASE_DIR, "servo_calibration.txt")

# Create necessary directories
for folder in [IMAGE_FOLDER, LOG_FOLDER]:
    os.makedirs(folder, exist_ok=True)

# Configure logging
LOG_FILE = os.path.join(LOG_FOLDER, f"watergun_{time.strftime('%Y%m%d_%H%M%S')}.log")
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# System Constants
CAMERA_RESOLUTION = (640, 480)
CAMERA_FRAMERATE = 30
MOTION_THRESHOLD = 35 # adjust for sensitivity
MIN_AREA = 800  # Minimum area to start tracking - adjust for sensitivity
TRACKING_MIN_AREA = 600  # Minimum area to maintain tracking - adjust for sensitivity
RELAY_PIN = 17
EXIT_SWITCH_PIN = 27
SPRAY_DURATION = 0.5  # seconds
LOOP_DELAY = 2  # seconds delay between detection cycles
MIN_DETECTION_INTERVAL = 3  # seconds
INITIAL_DETECTION_PERIOD = 1  # seconds

# Motion detection constants
BLUR_SIZE = 21
REF_FRAME_TIME_LIMIT = 120  # seconds
MIN_ACQUIRE_TIME = 2  # seconds
TARGET_MOVEMENT_THRESHOLD = 60  # pixels - adjust for sensitivity 
THRESHOLD_SENSITIVITY = 25
MIN_CONTOUR_AREA = 75 # adjust for sensitivity

# Tracking constants
TRACKER_TYPE = "KCF"  # Options: BOOSTING, MIL, KCF, TLD, MEDIANFLOW, GOTURN, MOSSE, CSRT
MAX_TRACK_FAILURES = 3  # Maximum consecutive tracking failures before resetting
TRACKING_TIMEOUT = 15  # seconds to track before giving up if not stabilized

# Servo configuration
SERVO_X_CHANNEL = 0
SERVO_Y_CHANNEL = 1
SERVO_Y_IS_CONTINUOUS = True
MIN_IMP = [500, 500]
MAX_IMP = [2500, 2500]
MIN_ANG = [0, 0]
MAX_ANG = [180, 180]
SERVO_TRIGGER_SWEEP = 20
SWEEP_ITERATIONS = 5

# Servo calibration defaults - these can be overridden by calibration
SERVO_X_CENTER = 90  # Default center position for X servo (horizontal)
SERVO_Y_CENTER = 90  # Default center position for Y servo (vertical)

# Mapping ranges - these define how camera coordinates map to servo angles
SERVO_X_MIN_ANGLE = 30   # Minimum angle for X servo
SERVO_X_MAX_ANGLE = 150  # Maximum angle for X servo
SERVO_Y_MIN_ANGLE = 30   # Minimum angle for Y servo
SERVO_Y_MAX_ANGLE = 150  # Maximum angle for Y servo

# Global variables
exit_flag = False
prev_frame = None
last_detection_time = 0
target_first_acquired_time = None
last_target_x = 0
last_target_y = 0
last_reference_update = datetime.datetime.now()
target_positions = []
POSITION_HISTORY_LENGTH = 10  # Number of positions to keep track of
STATIONARY_THRESHOLD = 30  # Maximum allowed movement in pixels (changed from 30 to 180)
MIN_POSITIONS_FOR_STATIONARY = 5  # Minimum number of positions needed to confirm stationary

# Tracking state variables
tracker = None
tracking_box = None
tracking_start_time = None
track_failure_count = 0
currently_tracking = False
calibration_mode = False

# Water jet angle compensation
WATER_JET_ANGLE_OFFSET = 5  # degrees to fire higher to compensate for water arc

# Shape filtering constants (optional enhancement)
USE_SHAPE_FILTERING = True  # Set to False to disable
MIN_ASPECT_RATIO = 0.3      # Minimum width/height ratio
MAX_ASPECT_RATIO = 3.0      # Maximum width/height ratio
MIN_SOLIDITY = 0.3          # Minimum filled area ratio (filters out very irregular shapes)


def log_status(message, level="INFO"):
    """Log status message with appropriate level"""
    if level == "ERROR":
        logger.error(message)
    elif level == "WARNING":
        logger.warning(message)
    else:
        logger.info(message)


def log_error(message, exception=None):
    """Log error messages with exception details if available"""
    if exception:
        logger.error(f"{message}: {str(exception)}", exc_info=True)
    else:
        logger.error(message)


def initialize_system():
    """Initialize system components and log configuration"""
    log_status("=== System Initialization ===")
    log_status(f"Base directory: {BASE_DIR}")
    log_status(f"Image folder: {IMAGE_FOLDER}")
    log_status(f"Log folder: {LOG_FOLDER}")
    log_status(f"Log file: {LOG_FILE}")
    log_status(f"Camera resolution: {CAMERA_RESOLUTION}")
    log_status(f"Camera framerate: {CAMERA_FRAMERATE}")
    log_status(f"Motion threshold: {MOTION_THRESHOLD}")
    log_status(f"Minimum area: {MIN_AREA}")
    log_status(f"Spray duration: {SPRAY_DURATION}")
    log_status(f"Tracking method: {TRACKER_TYPE}")
    log_status(f"Servo X center: {SERVO_X_CENTER}")
    log_status(f"Servo Y center: {SERVO_Y_CENTER}")
    log_status(f"Servo X range: {SERVO_X_MIN_ANGLE} to {SERVO_X_MAX_ANGLE}")
    log_status(f"Servo Y range: {SERVO_Y_MIN_ANGLE} to {SERVO_Y_MAX_ANGLE}")
    log_status("GPIO initialization complete")
    log_status("Camera initialization complete")
    log_status("=== Initialization Complete ===")


def create_tracker():
    """Create and return an OpenCV tracker based on the selected tracker type"""
    if TRACKER_TYPE == 'BOOSTING':
        return cv2.legacy.TrackerBoosting_create()
    elif TRACKER_TYPE == 'MIL':
        return cv2.legacy.TrackerMIL_create()
    elif TRACKER_TYPE == 'KCF':
        return cv2.legacy.TrackerKCF_create()
    elif TRACKER_TYPE == 'TLD':
        return cv2.legacy.TrackerTLD_create()
    elif TRACKER_TYPE == 'MEDIANFLOW':
        return cv2.legacy.TrackerMedianFlow_create()
    elif TRACKER_TYPE == 'GOTURN':
        return cv2.legacy.TrackerGOTURN_create()
    elif TRACKER_TYPE == 'MOSSE':
        return cv2.legacy.TrackerMOSSE_create()
    elif TRACKER_TYPE == 'CSRT':
        return cv2.legacy.TrackerCSRT_create()
    else:
        log_status(f"Unknown tracker type {TRACKER_TYPE}, using CSRT", "WARNING")
        return cv2.legacy.TrackerCSRT_create()


def set_servo_angle(servo_channel, angle):
    """Set servo angle with bounds checking"""
    try:
        angle = max(MIN_ANG[servo_channel], min(MAX_ANG[servo_channel], angle))
        log_status(f"Moving servo {servo_channel} to angle {angle}")
        pca.servo[servo_channel].angle = angle
        time.sleep(0.3)
    except Exception as e:
        log_error("Servo error", e)


def center_servos():
    """Move servos to their defined center positions"""
    log_status(f"Centering servos: X={SERVO_X_CENTER}, Y={SERVO_Y_CENTER}")
    set_servo_angle(SERVO_X_CHANNEL, SERVO_X_CENTER)
    set_servo_angle(SERVO_Y_CHANNEL, SERVO_Y_CENTER)
    time.sleep(1)  # Give servos time to move


def spray_water():
    """Activate water spray"""
    try:
        log_status("Activating water relay")
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        time.sleep(SPRAY_DURATION)
        GPIO.output(RELAY_PIN, GPIO.LOW)
        log_status("Deactivating water relay")
    except Exception as e:
        log_error("Error during water spray", e)


def process_frame(frame):
    """Process frame for motion detection"""
    frame = imutils.resize(frame, width=CAMERA_RESOLUTION[0])
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (BLUR_SIZE, BLUR_SIZE), 0)
    return frame, gray


def detect_motion(current_frame, reference_frame):
    """Detect motion between frames"""
    frame_delta = cv2.absdiff(reference_frame, current_frame)
    thresh = cv2.threshold(frame_delta, MOTION_THRESHOLD, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.dilate(thresh, None, iterations=2)
    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours, thresh


def is_target_stationary(current_x, current_y):
    """
    Check if target has remained stationary by tracking position history
    Returns True if target has been relatively stationary for the required time
    """
    global target_first_acquired_time, target_positions
    
    current_time = datetime.datetime.now()
    
    # Add current position to history
    target_positions.append((current_x, current_y, current_time))
    
    # Keep only recent positions (limit to POSITION_HISTORY_LENGTH)
    while len(target_positions) > POSITION_HISTORY_LENGTH:
        target_positions.pop(0)
    
    # Need minimum number of positions to determine if stationary
    if len(target_positions) < MIN_POSITIONS_FOR_STATIONARY:
        log_status(f"Not enough position history: {len(target_positions)}/{MIN_POSITIONS_FOR_STATIONARY}")
        return False
    
    # Check if enough time has passed since first acquisition
    time_since_first_acquisition = (current_time - target_first_acquired_time).total_seconds()
    log_status(f"Time since first acquisition: {time_since_first_acquisition:.1f} seconds")
    
    if time_since_first_acquisition < MIN_ACQUIRE_TIME:
        log_status("Not enough time since first acquisition")
        return False
    
    # Get the position when the target was first acquired
    if hasattr(is_target_stationary, 'initial_position'):
        initial_pos_x, initial_pos_y = is_target_stationary.initial_position
    else:
        # First time this function runs for current tracking session
        initial_pos_x, initial_pos_y = current_x, current_y
        is_target_stationary.initial_position = (initial_pos_x, initial_pos_y)
    
    # Check if all recent positions are within threshold compared to initial position
    is_stationary = True
    max_movement = 0
    
    for pos_x, pos_y, _ in target_positions:
        dx = abs(pos_x - initial_pos_x)
        dy = abs(pos_y - initial_pos_y)
        movement = math.sqrt(dx*dx + dy*dy)
        max_movement = max(max_movement, movement)
        
        if movement > STATIONARY_THRESHOLD:
            is_stationary = False
            log_status(f"Target moved too much: {movement} pixels from initial position (threshold: {STATIONARY_THRESHOLD})")
            break
    
    if is_stationary:
        log_status(f"Target is stationary. Max movement: {max_movement} pixels")
        return True
    else:
        # Reset acquisition time and initial position if target moved too much
        target_first_acquired_time = current_time
        is_target_stationary.initial_position = (current_x, current_y)
        return False
        
def aim_and_spray(x, y):
    """Aim servos and activate spray with custom center and mapping"""
    log_status(f"Aiming at coordinates: x={x}, y={y}")
    try:
        # Convert pixel coordinates to normalized values (0-1)
        norm_x = x / CAMERA_RESOLUTION[0]
        norm_y = y / CAMERA_RESOLUTION[1]
        
        # INVERT X-axis to fix camera flip issue - when object is on left, aim left
        norm_x = 1.0 - norm_x
        
        # Map normalized coordinates to servo angles using custom ranges
        angle_x = SERVO_X_MIN_ANGLE + norm_x * (SERVO_X_MAX_ANGLE - SERVO_X_MIN_ANGLE)
        angle_y = SERVO_Y_MIN_ANGLE + norm_y * (SERVO_Y_MAX_ANGLE - SERVO_Y_MIN_ANGLE)
        
        # Apply water jet angle offset to fire higher
        angle_y = angle_y - WATER_JET_ANGLE_OFFSET
        
        # Cap angles to ensure they're within the allowed range
        angle_x = max(MIN_ANG[0], min(MAX_ANG[0], angle_x))
        angle_y = max(MIN_ANG[1], min(MAX_ANG[1], angle_y))
        
        log_status(f"Calculated angles (with offset): x={angle_x:.2f}, y={angle_y:.2f}")
        
        # Initial aim
        set_servo_angle(SERVO_X_CHANNEL, angle_x)
        set_servo_angle(SERVO_Y_CHANNEL, angle_y)
        
        # Activate spray
        log_status("Starting spray sequence")
        GPIO.output(RELAY_PIN, GPIO.HIGH)
        
        # Sweeping pattern
        left_sweep = max(MIN_ANG[0], angle_x - SERVO_TRIGGER_SWEEP)
        right_sweep = min(MAX_ANG[0], angle_x + SERVO_TRIGGER_SWEEP)
        
        for i in range(SWEEP_ITERATIONS):
            log_status(f"Sweep iteration {i+1}/{SWEEP_ITERATIONS}")
            set_servo_angle(SERVO_X_CHANNEL, left_sweep)
            
            # Check for exit during spray sequence
            if exit_flag:
                break
            time.sleep(SPRAY_DURATION / (SWEEP_ITERATIONS * 2))
            
            set_servo_angle(SERVO_X_CHANNEL, right_sweep)
            if exit_flag:
                break
            time.sleep(SPRAY_DURATION / (SWEEP_ITERATIONS * 2))
            
    except Exception as e:
        log_error("Error during aim_and_spray", e)
    finally:
        GPIO.output(RELAY_PIN, GPIO.LOW)
        set_servo_angle(SERVO_X_CHANNEL, angle_x)
        log_status("Spray sequence completed")
        
def get_largest_motion(contours, min_area=MIN_AREA):
    """Get center coordinates and bounding box of largest motion with optional shape filtering"""
    if not contours:
        return None
    
    # Sort contours by area, largest first
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
    
    for contour in sorted_contours:
        area = cv2.contourArea(contour)
        if area >= min_area:
            (x, y, w, h) = cv2.boundingRect(contour)
            
            # Optional shape filtering to reduce false detections from branches
            if USE_SHAPE_FILTERING:
                # Check aspect ratio
                aspect_ratio = w / h if h > 0 else 0
                if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
                    log_status(f"Rejected contour: bad aspect ratio {aspect_ratio:.2f}")
                    continue
                
                # Check solidity (filled area vs bounding box area)
                hull = cv2.convexHull(contour)
                hull_area = cv2.contourArea(hull)
                if hull_area > 0:
                    solidity = area / hull_area
                    if solidity < MIN_SOLIDITY:
                        log_status(f"Rejected contour: low solidity {solidity:.2f}")
                        continue
            
            center_x = x + w // 2
            center_y = y + h // 2
            return (center_x, center_y, (x, y, w, h), contour)
    
    return None


def check_exit_conditions():
    """Check for exit conditions"""
    global exit_flag, calibration_mode
    while not exit_flag:
        if GPIO.input(EXIT_SWITCH_PIN) == GPIO.LOW:
            log_status("Exit button pressed")
            exit_flag = True
            break
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            log_status("ESC key pressed")
            exit_flag = True
            break
        elif key == ord('c'):  # 'c' key for calibration
            log_status("Calibration mode requested")
            calibration_mode = True
        elif key == ord('h'):  # 'h' key to center servos
            log_status("Centering servos requested")
            center_servos()
        elif key == ord('r'):  # 'r' key to reset tracking
            log_status("Reset tracking requested")
            reset_tracking()
            
        time.sleep(0.1)


def reset_tracking():
    """Reset tracking state variables"""
    global tracker, tracking_box, tracking_start_time, track_failure_count, target_positions, currently_tracking, target_first_acquired_time
    
    tracker = None
    tracking_box = None
    tracking_start_time = None
    track_failure_count = 0
    target_positions = []
    currently_tracking = False
    target_first_acquired_time = None
    if hasattr(is_target_stationary, 'initial_position'):
        delattr(is_target_stationary, 'initial_position')
    log_status("Tracking reset, looking for new objects")


def load_calibration():
    """Load servo calibration from file if it exists, otherwise create with defaults"""
    global SERVO_X_CENTER, SERVO_Y_CENTER
    global SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE
    
    try:
        if os.path.exists(CALIBRATION_FILE):
            log_status(f"Loading calibration from {CALIBRATION_FILE}")
            with open(CALIBRATION_FILE, 'r') as f:
                for line in f:
                    if '=' in line:
                        key, value = line.strip().split('=')
                        if key == 'SERVO_X_CENTER':
                            SERVO_X_CENTER = float(value)
                        elif key == 'SERVO_Y_CENTER':
                            SERVO_Y_CENTER = float(value)
                        elif key == 'SERVO_X_MIN_ANGLE':
                            SERVO_X_MIN_ANGLE = float(value)
                        elif key == 'SERVO_X_MAX_ANGLE':
                            SERVO_X_MAX_ANGLE = float(value)
                        elif key == 'SERVO_Y_MIN_ANGLE':
                            SERVO_Y_MIN_ANGLE = float(value)
                        elif key == 'SERVO_Y_MAX_ANGLE':
                            SERVO_Y_MAX_ANGLE = float(value)
            log_status(f"Loaded calibration: X={SERVO_X_CENTER}, Y={SERVO_Y_CENTER}")
            return True
        else:
            log_status(f"Calibration file not found. Creating with default values.")
            save_calibration()
            return False
    except Exception as e:
        log_error("Failed to load calibration", e)
        log_status("Creating calibration file with default values")
        save_calibration()
        return False


def save_calibration():
    """Save servo calibration to file"""
    try:
        with open(CALIBRATION_FILE, 'w') as f:
            f.write(f"SERVO_X_CENTER={SERVO_X_CENTER}\n")
            f.write(f"SERVO_Y_CENTER={SERVO_Y_CENTER}\n")
            f.write(f"SERVO_X_MIN_ANGLE={SERVO_X_MIN_ANGLE}\n")
            f.write(f"SERVO_X_MAX_ANGLE={SERVO_X_MAX_ANGLE}\n")
            f.write(f"SERVO_Y_MIN_ANGLE={SERVO_Y_MIN_ANGLE}\n")
            f.write(f"SERVO_Y_MAX_ANGLE={SERVO_Y_MAX_ANGLE}\n")
        log_status(f"Calibration saved to {CALIBRATION_FILE}")
        return True
    except Exception as e:
        log_error("Failed to save calibration", e)
        return False
        
def calibrate_servos():
    """Interactive calibration routine for servos"""
    global calibration_mode, SERVO_X_CENTER, SERVO_Y_CENTER
    global SERVO_X_MIN_ANGLE, SERVO_X_MAX_ANGLE, SERVO_Y_MIN_ANGLE, SERVO_Y_MAX_ANGLE
    
    calibration_mode = True
    log_status("Starting servo calibration...")
    
    # Create a calibration window with instructions
    cv2.namedWindow("Servo Calibration", cv2.WINDOW_NORMAL)
    calibration_img = np.zeros((480, 640, 3), dtype=np.uint8)
    
    # Center position calibration
    temp_x_center = SERVO_X_CENTER
    temp_y_center = SERVO_Y_CENTER
    
    # Set to current center position
    set_servo_angle(SERVO_X_CHANNEL, temp_x_center)
    set_servo_angle(SERVO_Y_CHANNEL, temp_y_center)
    
    calibration_step = 0  # 0: X/Y center, 1: X min/max, 2: Y min/max
    temp_x_min = SERVO_X_MIN_ANGLE
    temp_x_max = SERVO_X_MAX_ANGLE
    temp_y_min = SERVO_Y_MIN_ANGLE
    temp_y_max = SERVO_Y_MAX_ANGLE
    
    while calibration_mode:
        # Clear the image
        calibration_img.fill(0)
        
        if calibration_step == 0:
            # Center calibration
            cv2.putText(calibration_img, "CENTER POSITION CALIBRATION", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(calibration_img, f"X Center: {temp_x_center:.1f}", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(calibration_img, f"Y Center: {temp_y_center:.1f}", (20, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(calibration_img, "Use A/D keys to adjust X", (20, 160), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Use W/S keys to adjust Y", (20, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Press SPACE to continue to range calibration", (20, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Press ESC to cancel calibration", (20, 280), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        elif calibration_step == 1:
            # X range calibration
            cv2.putText(calibration_img, "X RANGE CALIBRATION", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(calibration_img, f"X Min: {temp_x_min:.1f}", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(calibration_img, f"X Max: {temp_x_max:.1f}", (20, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(calibration_img, "Use A/D keys to adjust X Min", (20, 160), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Use Q/E keys to adjust X Max", (20, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Press SPACE to continue to Y range calibration", (20, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Press ESC to cancel calibration", (20, 280), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        elif calibration_step == 2:
            # Y range calibration
            cv2.putText(calibration_img, "Y RANGE CALIBRATION", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(calibration_img, f"Y Min: {temp_y_min:.1f}", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(calibration_img, f"Y Max: {temp_y_max:.1f}", (20, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(calibration_img, "Use W/S keys to adjust Y Min", (20, 160), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Use R/F keys to adjust Y Max", (20, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Press SPACE to save and exit calibration", (20, 240), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(calibration_img, "Press ESC to cancel calibration", (20, 280), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Display the calibration window
        cv2.imshow("Servo Calibration", calibration_img)
        key = cv2.waitKey(100) & 0xFF
        
        # Process key inputs
        if key == 27:  # ESC key - cancel
            log_status("Calibration cancelled")
            calibration_mode = False
            set_servo_angle(SERVO_X_CHANNEL, SERVO_X_CENTER)
            set_servo_angle(SERVO_Y_CHANNEL, SERVO_Y_CENTER)
            break
            
        elif key == 32:  # SPACE key - next step
            if calibration_step == 0:
                # Save center positions and move to X range
                SERVO_X_CENTER = temp_x_center
                SERVO_Y_CENTER = temp_y_center
                calibration_step = 1
                # Move to min X position
                set_servo_angle(SERVO_X_CHANNEL, temp_x_min)
                log_status(f"Center positions set: X={SERVO_X_CENTER}, Y={SERVO_Y_CENTER}")
                log_status("Now calibrating X range")
            
            elif calibration_step == 1:
                # Save X range and move to Y range
                SERVO_X_MIN_ANGLE = temp_x_min
                SERVO_X_MAX_ANGLE = temp_x_max
                calibration_step = 2
                # Move to min Y position
                set_servo_angle(SERVO_X_CHANNEL, SERVO_X_CENTER)  # Reset X to center
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_min)
                log_status(f"X range set: Min={SERVO_X_MIN_ANGLE}, Max={SERVO_X_MAX_ANGLE}")
                log_status("Now calibrating Y range")
            
            elif calibration_step == 2:
                # Save Y range and finish calibration
                SERVO_Y_MIN_ANGLE = temp_y_min
                SERVO_Y_MAX_ANGLE = temp_y_max
                log_status(f"Y range set: Min={SERVO_Y_MIN_ANGLE}, Max={SERVO_Y_MAX_ANGLE}")
                
                # Save all settings to file
                save_calibration()
                
                # Return to center position
                set_servo_angle(SERVO_X_CHANNEL, SERVO_X_CENTER)
                set_servo_angle(SERVO_Y_CHANNEL, SERVO_Y_CENTER)
                
                calibration_mode = False
                log_status("Calibration completed and saved")
        
        # Handle adjustments based on current calibration step
        if calibration_step == 0:
            # Center position adjustments
            if key == ord('a'):  # Left
                temp_x_center = max(MIN_ANG[0], temp_x_center - 2)
                set_servo_angle(SERVO_X_CHANNEL, temp_x_center)
            
            elif key == ord('d'):  # Right
                temp_x_center = min(MAX_ANG[0], temp_x_center + 2)
                set_servo_angle(SERVO_X_CHANNEL, temp_x_center)
            
            elif key == ord('w'):  # Up
                temp_y_center = max(MIN_ANG[1], temp_y_center - 2)
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_center)
            
            elif key == ord('s'):  # Down
                temp_y_center = min(MAX_ANG[1], temp_y_center + 2)
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_center)
        
        elif calibration_step == 1:
            # X range adjustments
            if key == ord('a'):  # Decrease X Min
                temp_x_min = max(MIN_ANG[0], temp_x_min - 2)
                set_servo_angle(SERVO_X_CHANNEL, temp_x_min)
            
            elif key == ord('d'):  # Increase X Min
                temp_x_min = min(temp_x_max - 10, temp_x_min + 2)
                set_servo_angle(SERVO_X_CHANNEL, temp_x_min)
            
            elif key == ord('q'):  # Decrease X Max
                temp_x_max = max(temp_x_min + 10, temp_x_max - 2)
                set_servo_angle(SERVO_X_CHANNEL, temp_x_max)
            
            elif key == ord('e'):  # Increase X Max
                temp_x_max = min(MAX_ANG[0], temp_x_max + 2)
                set_servo_angle(SERVO_X_CHANNEL, temp_x_max)
        
        elif calibration_step == 2:
            # Y range adjustments
            if key == ord('w'):  # Decrease Y Min
                temp_y_min = max(MIN_ANG[1], temp_y_min - 2)
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_min)
            
            elif key == ord('s'):  # Increase Y Min
                temp_y_min = min(temp_y_max - 10, temp_y_min + 2)
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_min)
            
            elif key == ord('r'):  # Decrease Y Max
                temp_y_max = max(temp_y_min + 10, temp_y_max - 2)
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_max)
            
            elif key == ord('f'):  # Increase Y Max
                temp_y_max = min(MAX_ANG[1], temp_y_max + 2)
                set_servo_angle(SERVO_Y_CHANNEL, temp_y_max)
    
    # Close calibration window
    cv2.destroyWindow("Servo Calibration")
    
def interruptible_sleep(duration):
    """Sleep that can be interrupted by exit conditions"""
    global exit_flag
    start_time = time.time()
    while time.time() - start_time < duration:
        if exit_flag:
            return
        if GPIO.input(EXIT_SWITCH_PIN) == GPIO.LOW:
            log_status("Exit button pressed during sleep")
            exit_flag = True
            return
        key = cv2.waitKey(100) & 0xFF
        if key == 27:  # ESC key
            log_status("ESC key pressed during sleep")
            exit_flag = True
            return
        time.sleep(0.1)
    
def main():
    """Main execution function"""
    global prev_frame, exit_flag, last_detection_time, last_reference_update
    global tracker, tracking_box, tracking_start_time, track_failure_count, target_positions, currently_tracking
    global target_first_acquired_time, pca, calibration_mode

    try:
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Ensure relay is off initially
        GPIO.setup(EXIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize PCA9685 and servos
        pca = ServoKit(channels=16)
        pca.servo[SERVO_X_CHANNEL].set_pulse_width_range(MIN_IMP[0], MAX_IMP[0])
        pca.servo[SERVO_Y_CHANNEL].set_pulse_width_range(MIN_IMP[1], MAX_IMP[1])

        # Load calibration if available, or create with defaults
        load_calibration()

        # Center servos at startup
        center_servos()

        # Initialize camera
        camera = PiCamera()
        camera.resolution = CAMERA_RESOLUTION
        camera.framerate = CAMERA_FRAMERATE
        camera.vflip = True  # Add this line to flip vertically
        camera.hflip = True  # Uncomment if horizontal flip is also needed
        raw_capture = PiRGBArray(camera, size=CAMERA_RESOLUTION)

        # Allow camera to warm up
        time.sleep(2)

        # Initialize system
        initialize_system()

        # Start exit conditions checking thread
        exit_thread = threading.Thread(target=check_exit_conditions)
        exit_thread.daemon = True
        exit_thread.start()

        log_status("Starting main detection loop")
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            if exit_flag:
                log_status("Exit flag detected, stopping main loop")
                break

            # Check if calibration mode is requested
            if calibration_mode:
                log_status("Entering calibration mode")
                calibrate_servos()
                log_status("Resuming normal operation")
                
            current_time = datetime.datetime.now()
            image = frame.array
            resized_image, processed_frame = process_frame(image)
            
            # Clear buffer for next frame
            raw_capture.truncate(0)
            raw_capture.seek(0)
            
            # Display frame (debug window)
            debug_frame = resized_image.copy()
            
            # Check if we're currently tracking an object
            if currently_tracking:
                # Update the tracker
                success, box = tracker.update(resized_image)
                
                if success:
                    # Reset failure counter
                    track_failure_count = 0
                    
                    # Extract position
                    x, y, w, h = [int(v) for v in box]
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Check if target is stationary
                    if is_target_stationary(center_x, center_y):
                        log_status("Target acquired and stationary")
                        
                        # Draw rectangle around tracked object
                        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(debug_frame, "TARGET STABLE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.drawMarker(debug_frame, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                        
                        # Save image
                        timestamp = int(time.time())
                        image_path = os.path.join(IMAGE_FOLDER, f"motion_{timestamp}.jpg")
                        cv2.imwrite(image_path, debug_frame)
                        log_status(f"Saved detection image: {image_path}")
                        
                        # Process target
                        aim_and_spray(center_x, center_y)
                        last_detection_time = current_time
                        
                        log_status(f"Waiting {LOOP_DELAY} seconds before next detection")
                        interruptible_sleep(LOOP_DELAY)
                        if exit_flag:
                            break
                        
                        # Reset tracking to look for new targets
                        reset_tracking()
                    else:
                        # Still tracking but not yet stationary
                        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(debug_frame, "Tracking", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.drawMarker(debug_frame, (center_x, center_y), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)
                    
                else:
                    # Tracking failure
                    track_failure_count += 1
                    log_status(f"Tracking failure #{track_failure_count}/{MAX_TRACK_FAILURES}")
                    cv2.putText(debug_frame, f"Tracking failure #{track_failure_count}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    
                    if track_failure_count >= MAX_TRACK_FAILURES:
                        log_status("Too many tracking failures, resetting")
                        reset_tracking()
                
                # Check for tracking timeout
                if tracking_start_time:
                    elapsed_time = (current_time - tracking_start_time).total_seconds()
                    cv2.putText(debug_frame, f"Tracking: {elapsed_time:.1f}s", (10, 60), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    if elapsed_time > TRACKING_TIMEOUT:
                        log_status(f"Tracking timeout after {elapsed_time:.1f} seconds")
                        reset_tracking()
            
            # If not tracking, look for motion
            else:
                # Initialize or update reference frame periodically
                if prev_frame is None or (current_time - last_reference_update).seconds > REF_FRAME_TIME_LIMIT:
                    prev_frame = processed_frame
                    last_reference_update = current_time
                    log_status("Reference frame updated")
                    continue
                
                # Detect motion
                contours, thresh = detect_motion(processed_frame, prev_frame)
                prev_frame = processed_frame
                
                # Look for suitable object to track
                motion_result = get_largest_motion(contours)
                
                if motion_result:
                    center_x, center_y, bbox, contour = motion_result
                    (x, y, w, h) = bbox
                    
                    # Start tracking this object
                    tracker = create_tracker()
                    success = tracker.init(resized_image, (x, y, w, h))
                    
                    if success:
                        log_status(f"Started tracking object at ({center_x}, {center_y})")
                        tracking_box = (x, y, w, h)
                        tracking_start_time = current_time
                        target_first_acquired_time = current_time
                        currently_tracking = True
                        track_failure_count = 0
                        target_positions = []
                        
                        # Draw rectangle around detected motion
                        cv2.rectangle(debug_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                        cv2.putText(debug_frame, "Motion Detected", (x, y - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    else:
                        log_status("Failed to initialize tracker")
                
                # Draw motion contours
                cv2.putText(debug_frame, "Monitoring", (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.drawContours(debug_frame, contours, -1, (0, 255, 255), 2)
            
            # Display status on frame
            cv2.putText(debug_frame, f"Time: {current_time.strftime('%H:%M:%S')}", (10, debug_frame.shape[0] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Show the frame
            cv2.imshow("Water Gun Monitor", debug_frame)
            
            # Check for key presses (moved to check_exit_conditions thread)
            
    except Exception as e:
        log_error("Error in main loop", e)
    
    finally:
        # Clean up
        log_status("Cleaning up resources")
        GPIO.output(RELAY_PIN, GPIO.LOW)  # Ensure relay is off
        center_servos()  # Return servos to center position
        GPIO.cleanup()
        cv2.destroyAllWindows()
        
        if 'camera' in locals():
            camera.close()
        
        log_status("System shutdown complete")


if __name__ == "__main__":
    main()
