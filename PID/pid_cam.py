#!/usr/bin/env python3
import time
import cv2
import RPi.GPIO as GPIO
from ultralytics import YOLO
from picamera2 import Picamera2
from picarx import Picarx

# -----------------------------
# Configuration & Constants
# -----------------------------
SAMPLETIME = 0.1             # Loop interval (seconds)
KP_TURN = 0.2                # Proportional gain for turning (adjust as needed)
ERROR_DEADBAND = 20          # Pixels error below which we consider the cup centered
MAX_TURN_SPEED = 50          # Maximum motor speed magnitude for turning

FRAME_WIDTH = 640            # Camera frame width (pixels)
FRAME_HEIGHT = 480           # Camera frame height (pixels)

# -----------------------------
# Initialize Components
# -----------------------------
px = Picarx()
model = YOLO("yolov5su.pt")  # Using the improved 'u' model as per the tip

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
)
picam2.configure(config)
picam2.start_preview()  # Optional: if a display is available
picam2.start()

print("Starting cup detection and in-place turning control (headless mode). Press Ctrl+C to exit.")

try:
    while True:
        frame = picam2.capture_array()
        frame = frame[:, :, :3]  # Convert from 4 channels to 3 channels

        results = model(frame, stream=True)
        
        cup_detected = False
        best_bbox = None
        best_confidence = 0

        for result in results:
            for box in result.boxes:
                cls_index = int(box.cls[0]) if box.cls is not None else -1
                if cls_index == 41:  # Cup class in COCO
                    conf = box.conf[0]
                    if conf > best_confidence:
                        best_confidence = conf
                        best_bbox = box.xyxy[0]
                    cup_detected = True

        if cup_detected and best_bbox is not None:
            x_min, y_min, x_max, y_max = map(int, best_bbox)
            bbox_center_x = (x_min + x_max) / 2
            frame_center_x = FRAME_WIDTH / 2
            error = frame_center_x - bbox_center_x

            if abs(error) < ERROR_DEADBAND:
                turn_speed = 0
                print("Cup centered. Stopping rotation.")
            else:
                turn_speed = KP_TURN * error
                if turn_speed > MAX_TURN_SPEED:
                    turn_speed = MAX_TURN_SPEED
                elif turn_speed < -MAX_TURN_SPEED:
                    turn_speed = -MAX_TURN_SPEED
                print(f"Turning with speed: {turn_speed:.2f} (Error: {error:.2f})")

            # Turn in place: left motor negative, right motor positive for left turn (and vice versa)
            px.set_motor_speed(1, -turn_speed)
            px.set_motor_speed(2, turn_speed)
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")
        else:
            px.set_motor_speed(1, 0)
            px.set_motor_speed(2, 0)
            print("No cups detected.")

        # Instead of displaying, we just wait
        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and control stopped by user.")

finally:
    px.set_motor_speed(1, 0)
    px.set_motor_speed(2, 0)
    picam2.stop_preview()
    picam2.stop()
    GPIO.cleanup()
    # cv2.destroyAllWindows()  # Removed since no GUI windows are created
