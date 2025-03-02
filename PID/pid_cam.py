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
SAMPLETIME = 0.1        # Loop interval (seconds)
KP_TURN = 0.1           # Proportional gain for turning

FRAME_WIDTH = 640       # Camera frame width (pixels)
FRAME_HEIGHT = 480      # Camera frame height (pixels)

# Steering limits (in degrees; adjust as needed)
MAX_STEER = 30
MIN_STEER = -30

# -----------------------------
# Initialize Components
# -----------------------------
# Initialize robot drive & steering
px = Picarx()

# Load YOLO model (using a YOLOv5 model file)
model = YOLO("yolov5s.pt")
# (If using a YOLOv8 model, adjust the file accordingly.)

# Initialize the robot camera using Picamera2
picam2 = Picamera2()
# Create and configure a preview at the desired resolution and format.
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
)
picam2.configure(config)
picam2.start_preview()  # Optional: start preview if you have a display connected
picam2.start()

print("Starting combined cup detection and steering control. Press 'q' to exit.")

try:
    while True:
        # Capture a frame from the camera (4 channels: XRGB)
        frame = picam2.capture_array()
        # Drop the extra alpha channel (convert from 4 channels to 3 channels)
        frame = frame[:, :, :3]
        
        # Run YOLO inference (using stream mode for efficiency)
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
                    # Choose the detection with the highest confidence
                    if conf > best_confidence:
                        best_confidence = conf
                        best_bbox = box.xyxy[0]
                    cup_detected = True
        
        if cup_detected and best_bbox is not None:
            # Extract bounding box coordinates
            x_min, y_min, x_max, y_max = map(int, best_bbox)
            # Compute the center of the bounding box
            bbox_center_x = (x_min + x_max) / 2
            # Compute the error: difference between frame center and object center
            frame_center_x = FRAME_WIDTH / 2
            error = frame_center_x - bbox_center_x  # positive error means object is left of center
            
            # Compute turn command (proportional to error)
            turn_command = KP_TURN * error
            # Clamp the turn command to steering limits
            if turn_command > MAX_STEER:
                turn_command = MAX_STEER
            elif turn_command < MIN_STEER:
                turn_command = MIN_STEER
            
            # Apply the steering command
            px.set_dir(turn_command)
            
            # Print detection and steering info to the console
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")
            print(f"Frame center: {frame_center_x:.2f}, Object center: {bbox_center_x:.2f}, Error: {error:.2f}")
            print(f"Turn command applied: {turn_command:.2f}\n")
            
            # Optionally, draw the bounding box on the frame (for visualization)
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        else:
            # If no cup detected, reset steering (or keep previous command)
            px.set_dir(0)
            print("No cups detected.\n")
        
        # Optionally, display the frame (if a display is available)
        cv2.imshow("Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and control stopped by user.")

finally:
    # Reset steering and clean up resources
    px.set_dir(0)
    picam2.stop_preview()
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()
