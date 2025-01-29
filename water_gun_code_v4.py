#!/usr/bin/env python3

# Required packages
# pip install opencv-python-headless
# pip install picamera
# pip install RPi.GPIO
# pip install adafruit-circuitpython-servokit
# pip install imutils

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
import distutils.dir_util
import os
import logging

# Define paths and create directories
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
IMAGE_FOLDER = os.path.join(BASE_DIR, "motion_images")
LOG_FOLDER = os.path.join(BASE_DIR, "logs")

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
MOTION_THRESHOLD = 25
MIN_AREA = 500
RELAY_PIN = 17
EXIT_SWITCH_PIN = 18
SPRAY_DURATION = 0.5  # seconds
LOOP_DELAY = 2  # seconds delay between detection cycles
MIN_DETECTION_INTERVAL = 3  # seconds

# Motion detection constants
BLUR_SIZE = 21
REF_FRAME_TIME_LIMIT = 120  # seconds
MIN_ACQUIRE_TIME = 2  # seconds
TARGET_MOVEMENT_THRESHOLD = 50  # pixels
THRESHOLD_SENSITIVITY = 25
MIN_CONTOUR_AREA = 16

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

# Global variables
exit_flag = False
prev_frame = None
last_detection_time = 0
target_first_acquired_time = datetime.datetime.now()
last_target_x = 0
last_target_y = 0
last_reference_update = datetime.datetime.now()
target_positions = []
POSITION_HISTORY_LENGTH = 10  # Number of positions to keep track of
STATIONARY_THRESHOLD = 20  # Maximum allowed movement in pixels
MIN_POSITIONS_FOR_STATIONARY = 5  # Minimum number of positions needed to confirm stationary


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
    log_status("GPIO initialization complete")
    log_status("Camera initialization complete")
    log_status("=== Initialization Complete ===")

def set_servo_angle(servo_channel, angle):
    """Set servo angle with bounds checking"""
    try:
        angle = max(MIN_ANG[servo_channel], min(MAX_ANG[servo_channel], angle))
        log_status(f"Moving servo {servo_channel} to angle {angle}")
        pca.servo[servo_channel].angle = angle
        time.sleep(0.3)
    except Exception as e:
        log_error("Servo error", e)

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
    return gray

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
    global target_positions
    current_time = datetime.datetime.now()
    
    # Add current position to history
    target_positions.append((current_x, current_y, current_time))
    
    # Keep only recent positions
    while (len(target_positions) > 0 and 
           (current_time - target_positions[0][2]).seconds > MIN_ACQUIRE_TIME):
        target_positions.pop(0)
    
    # Check if enough time has passed since first acquisition
    time_since_first_acquisition = current_time - target_first_acquired_time
    if time_since_first_acquisition.seconds >= INITIAL_DETECTION_PERIOD:
        # Check if all recent positions are within threshold
        is_stationary = True
        for i in range(len(target_positions) - 1):
            dx = abs(target_positions[i][0] - target_positions[i+1][0])
            dy = abs(target_positions[i][1] - target_positions[i+1][1])
            if dx > STATIONARY_THRESHOLD or dy > STATIONARY_THRESHOLD:
                is_stationary = False
                break
        
        # Check if enough time has passed
        time_stationary = current_time - target_positions[0][2]
        
        if is_stationary and time_stationary.seconds >= MIN_ACQUIRE_TIME:
            log_status("Target has been detected and moved less thand thresholds")
            return True
    else:
        # If the target has been detected for the initial detection period, consider it stationary
        if len(target_positions) >= MIN_POSITIONS_FOR_STATIONARY:
            log_status("Target has been detected for the initial detection period, consider it stationary")
            return True
    
    return False

def aim_and_spray(x, y):
    """Aim servos and activate spray"""
    log_status(f"Aiming at coordinates: x={x}, y={y}")
    try:
        # Convert pixel coordinates to angles
        angle_x = (x / CAMERA_RESOLUTION[0]) * (MAX_ANG[0] - MIN_ANG[0]) + MIN_ANG[0]
        angle_y = (y / CAMERA_RESOLUTION[1]) * (MAX_ANG[1] - MIN_ANG[1]) + MIN_ANG[1]
        
        # Cap angles
        angle_x = max(MIN_ANG[0], min(MAX_ANG[0], angle_x))
        angle_y = max(MIN_ANG[1], min(MAX_ANG[1], angle_y))
        
        log_status(f"Calculated angles: x={angle_x:.2f}, y={angle_y:.2f}")
        
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
            time.sleep(SPRAY_DURATION / (SWEEP_ITERATIONS * 2))
            set_servo_angle(SERVO_X_CHANNEL, right_sweep)
            time.sleep(SPRAY_DURATION / (SWEEP_ITERATIONS * 2))
            
    except Exception as e:
        log_error("Error during aim_and_spray", e)
    finally:
        GPIO.output(RELAY_PIN, GPIO.LOW)
        set_servo_angle(SERVO_X_CHANNEL, angle_x)
        log_status("Spray sequence completed")

def validate_motion(contours):
    """Validate detected motion"""
    if not contours:
        return False
    
    largest_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest_contour)
    return MIN_AREA <= area <= CAMERA_RESOLUTION[0] * CAMERA_RESOLUTION[1] * 0.5

def get_largest_motion(contours):
    """Get center coordinates of largest motion"""
    if not contours:
        return None
    
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) < MIN_AREA:
        return None
    
    (x, y, w, h) = cv2.boundingRect(largest_contour)
    center_x = x + w // 2
    center_y = y + h // 2
    return (center_x, center_y, largest_contour)

def check_exit_conditions():
    """Check for exit conditions"""
    global exit_flag
    while not exit_flag:
        if GPIO.input(EXIT_SWITCH_PIN) == GPIO.LOW:
            exit_flag = True
            break
        
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            exit_flag = True
            break
        
        time.sleep(0.1)

def main():
    """Main execution function"""
    global prev_frame, exit_flag, last_detection_time, last_reference_update, target_positions, target_first_acquired_time

    try:
        # Initialize GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(RELAY_PIN, GPIO.OUT)
        GPIO.setup(EXIT_SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Initialize PCA9685 and servos
        pca = ServoKit(channels=16)
        pca.servo[SERVO_X_CHANNEL].set_pulse_width_range(MIN_IMP[0], MAX_IMP[0])
        pca.servo[SERVO_Y_CHANNEL].set_pulse_width_range(MIN_IMP[1], MAX_IMP[1])

        # Initialize camera
        camera = PiCamera()
        camera.resolution = CAMERA_RESOLUTION
        camera.framerate = CAMERA_FRAMERATE
        raw_capture = PiRGBArray(camera, size=CAMERA_RESOLUTION)

        # Allow camera to warm up
        time.sleep(2)

        # Initialize system
        initialize_system()

        # Start exit conditions checking thread
        exit_thread = threading.Thread(target=check_exit_conditions)
        exit_thread.start()

        log_status("Starting main detection loop")
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            if exit_flag:
                log_status("Exit flag detected, stopping main loop")
                break

            current_time = datetime.datetime.now()
            image = frame.array
            processed_frame = process_frame(image)
            
            # Initialize or update reference frame
            if prev_frame is None or (current_time - last_reference_update).seconds > REF_FRAME_TIME_LIMIT:
                prev_frame = processed_frame
                last_reference_update = current_time
                raw_capture.truncate(0)
                log_status("Reference frame updated")
                continue
            
            # Detect motion
            contours, thresh = detect_motion(processed_frame, prev_frame)
            
            if validate_motion(contours):
                target_info = get_largest_motion(contours)
                
                if target_info:
                    center_x, center_y, largest_contour = target_info
                    log_status(f"Motion detected at coordinates: x={center_x}, y={center_y}")
                    
                    # Set target_first_acquired_time if it's the first detection
                    if target_first_acquired_time is None:
                        target_first_acquired_time = current_time
                    
                    if is_target_stationary(center_x, center_y):
                        log_status("Target acquired and stationary")
                        
                        # Draw targeting
                        (x, y, w, h) = cv2.boundingRect(largest_contour)
                        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.drawMarker(image, (center_x, center_y), (0, 0, 255), 
                                     cv2.MARKER_CROSS, 10, 2)
                        
                        # Save image
                        timestamp = int(time.time())
                        image_path = os.path.join(IMAGE_FOLDER, f"motion_{timestamp}.jpg")
                        cv2.imwrite(image_path, image)
                        log_status(f"Saved detection image: {image_path}")
                        
                        # Process target
                        aim_and_spray(center_x, center_y)
                        last_detection_time = current_time
                        
                        log_status(f"Waiting {LOOP_DELAY} seconds before next detection")
                        time.sleep(LOOP_DELAY)
                        
                        # Clear position history after confirming stationary target
                        target_positions = []
                        
            # Display frame
            cv2.imshow("Frame", image)
            
            # Reset for next frame
            prev_frame = processed_frame
            raw_capture.truncate(0)
            
    except Exception as e:
        log_error("Critical error in main loop", e)
    finally:
        log_status("Starting system shutdown")
        try:
            pca.servo[SERVO_X_CHANNEL].angle = None
            pca.servo[SERVO_Y_CHANNEL].angle = None
            log_status("Servos disabled")
        except Exception as e:
            log_error("Error disabling servos", e)
        
        GPIO.cleanup()
        cv2.destroyAllWindows()
        camera.close()
        log_status("Hardware resources released")
        exit_flag = True
        exit_thread.join(timeout=1)
        log_status("System shutdown complete")

if __name__ == "__main__":
    main()
