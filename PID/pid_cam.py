#!/usr/bin/env python3
import time
import cv2
import RPi.GPIO as GPIO
from ultralytics import YOLO
from picamera2 import Picamera2
from picarx import Picarx

# Configuration & Constants
SAMPLETIME = 0.1
KP_TURN = 0.2
ERROR_DEADBAND = 20
MAX_TURN_SPEED = 50
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Initialize Components
px = Picarx()
model = YOLO("yolov5su.pt")  # Improved model

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
)
picam2.configure(config)
picam2.start_preview()  # Optional if display is available
picam2.start()

print("Starting cup detection and tracking (headless mode). Press Ctrl+C to exit.")

try:
    while True:
        frame = picam2.capture_array()
        frame = frame[:, :, :3]  # Convert to 3-channel RGB
        
        results = model(frame, stream=True)
        cup_detected = False
        best_bbox = None
        best_confidence = 0

        for result in results:
            for box in result.boxes:
                cls_index = int(box.cls[0]) if box.cls is not None else -1
                if cls_index == 41:
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
                steer_cmd = 0
                print("Cup centered. Steering straight.")
            else:
                steer_cmd = KP_TURN * error
                if steer_cmd > MAX_TURN_SPEED:
                    steer_cmd = MAX_TURN_SPEED
                elif steer_cmd < -MAX_TURN_SPEED:
                    steer_cmd = -MAX_TURN_SPEED
                print(f"Error: {error:.2f}, Steering command: {steer_cmd:.2f}°")

            # Apply steering via front wheels
            px.set_dir_servo_angle(steer_cmd)
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")
        else:
            px.set_dir_servo_angle(0)
            print("No cups detected.")

        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and control stopped by user.")

finally:
    px.set_dir_servo_angle(0)
    picam2.stop_preview()
    picam2.stop()
    GPIO.cleanup()
    # No cv2.destroyAllWindows() needed in headless mode