#!/usr/bin/env python3
import time
import cv2
import RPi.GPIO as GPIO

# Import SunFounder libraries
from picarx import Picarx           # For driving the car (motors & steering)
from picamera2 import Picamera2     # For camera functionality

# -----------------------------
# Configuration & Constants
# -----------------------------
SAMPLETIME = 0.1        # Loop interval (seconds)

# Proportional gain for turning (tune this for your setup)
KP_TURN = 0.1

# Camera frame dimensions (adjust based on your camera configuration)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Steering limits (assumed range in degrees; adjust as needed)
MAX_STEER = 30
MIN_STEER = -30

# -----------------------------
# Initialize SunFounder Components
# -----------------------------
px = Picarx()
picam2 = Picamera2()
picam2.start_preview()  # Start camera preview if desired

# -----------------------------
# Dummy Object Detection Function
# -----------------------------
def detect_object(frame):
    """
    This function simulates object detection.
    Replace this with your actual CV algorithm.
    It returns a bounding box as (x_min, y_min, x_max, y_max).
    """
    # For demonstration: simulate a bounding box around the center.
    box_width = 100
    box_height = 100
    x_min = (FRAME_WIDTH // 2) - (box_width // 2)
    y_min = (FRAME_HEIGHT // 2) - (box_height // 2)
    x_max = x_min + box_width
    y_max = y_min + box_height
    return (x_min, y_min, x_max, y_max)

print("Starting turning control using CV bounding box...")

while True:
    # Capture a frame from the camera
    frame = picam2.capture_array()
    
    # Detect object to get the bounding box
    bbox = detect_object(frame)
    x_min, y_min, x_max, y_max = bbox
    
    # Calculate bounding box dimensions and center
    bbox_width = x_max - x_min
    bbox_height = y_max - y_min
    bbox_center_x = (x_min + x_max) / 2
    bbox_center_y = (y_min + y_max) / 2

    # Determine the horizontal error (difference from the frame center)
    frame_center_x = FRAME_WIDTH / 2
    error = frame_center_x - bbox_center_x  # positive: object is left of center

    # Compute the turn command using a proportional controller
    turn_command = KP_TURN * error

    # Clamp the turn command to the steering limits
    if turn_command > MAX_STEER:
        turn_command = MAX_STEER
    elif turn_command < MIN_STEER:
        turn_command = MIN_STEER

    # Apply the steering command
    # (Assumes px.set_dir() adjusts the steering angle; adjust if your API differs)
    px.set_dir(turn_command)

    # Debug output
    print(f"BBox: {bbox}, Center: ({bbox_center_x:.2f}, {bbox_center_y:.2f}), " +
          f"Error: {error:.2f}, Turn command: {turn_command:.2f}")

    # Optional: Draw the bounding box on the frame for visualization
    cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(SAMPLETIME)

# When finished, reset steering and clean up resources
px.set_dir(0)
picam2.stop_preview()
GPIO.cleanup()
cv2.destroyAllWindows()
