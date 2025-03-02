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
# Initialize robot drive & differential motor control
px = Picarx()

# Load YOLO model (using a YOLOv5 model file; change filename if needed)
model = YOLO("yolov5s.pt")

# Initialize the robot camera using Picamera2
picam2 = Picamera2()
# Configure preview: use 4-channel XRGB format; we'll drop the alpha channel later.
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
)
picam2.configure(config)
picam2.start_preview()  # Optional: if a local display is attached
picam2.start()

print("Starting cup detection and in-place turning control. Press 'q' to exit.")

try:
    while True:
        # Capture a frame (4 channels: XRGB)
        frame = picam2.capture_array()
        # Convert frame to 3 channels (drop alpha)
        frame = frame[:, :, :3]

        # Run YOLO inference on the frame (stream mode for efficiency)
        results = model(frame, stream=True)
        
        cup_detected = False
        best_bbox = None
        best_confidence = 0
        
        # Process detections: look for cups (COCO index 41)
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
            # Extract bounding box coordinates
            x_min, y_min, x_max, y_max = map(int, best_bbox)
            # Compute the center of the bounding box
            bbox_center_x = (x_min + x_max) / 2
            # Compute horizontal error (positive if cup is left of center)
            frame_center_x = FRAME_WIDTH / 2
            error = frame_center_x - bbox_center_x
            
            # Print detection info
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")
            print(f"Frame center: {frame_center_x:.2f}, Object center: {bbox_center_x:.2f}, Error: {error:.2f}")
            
            # If error is within deadband, consider the cup centered: stop turning.
            if abs(error) < ERROR_DEADBAND:
                turn_speed = 0
                print("Cup centered. Stopping rotation.\n")
            else:
                # Compute turn speed from error (adjust gain as needed)
                turn_speed = KP_TURN * error
                # Clamp the turn speed to maximum limits
                if turn_speed > MAX_TURN_SPEED:
                    turn_speed = MAX_TURN_SPEED
                elif turn_speed < -MAX_TURN_SPEED:
                    turn_speed = -MAX_TURN_SPEED
                print(f"Turning with computed speed: {turn_speed:.2f}\n")
            
            # To turn in place:
            # - If turn_speed is positive (error positive: cup is left), then
            #   turn left: left motor runs backward, right motor forward.
            # - If turn_speed is negative, turn right.
            px.set_motor_speed(1, -turn_speed)  # Left motor
            px.set_motor_speed(2, turn_speed)   # Right motor

            # Optionally, draw the bounding box for visualization (if display available)
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        else:
            # No cup detected: stop turning.
            px.set_motor_speed(1, 0)
            px.set_motor_speed(2, 0)
            print("No cups detected.\n")
        
        # Optionally, display the frame (if a display is attached)
        cv2.imshow("Cup Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and control stopped by user.")

finally:
    # Stop the motors, reset steering, and clean up resources
    px.set_motor_speed(1, 0)
    px.set_motor_speed(2, 0)
    picam2.stop_preview()
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()