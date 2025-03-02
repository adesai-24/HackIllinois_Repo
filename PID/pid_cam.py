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
SAMPLETIME = 0.1             # Loop interval in seconds
KP_TURN = 0.2                # Proportional gain for steering
ERROR_DEADBAND = 20          # Pixel threshold below which cup is considered centered
MAX_STEER = 30               # Maximum steering angle (degrees)
FORWARD_SPEED = 50           # Constant forward speed (adjust as needed)

FRAME_WIDTH = 640            # Camera frame width in pixels
FRAME_HEIGHT = 480           # Camera frame height in pixels

# -----------------------------
# Initialize Components
# -----------------------------
# Initialize robot drivetrain and steering (front wheels)
px = Picarx()

# Load the improved YOLO model (yolov5su.pt)
model = YOLO("yolov5su.pt")

# Initialize the robot camera using Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
)
picam2.configure(config)
picam2.start_preview()  # Optional: only if a display is attached locally
picam2.start()

print("Starting cup detection and drivetrain tracking. Press 'q' to exit.")

try:
    while True:
        # Drive forward at constant speed
        px.forward(FORWARD_SPEED)

        # Capture a frame (returns a 4-channel array, XRGB8888)
        frame = picam2.capture_array()
        # Convert to 3-channel RGB by dropping the alpha channel
        frame = frame[:, :, :3]

        # Run YOLO inference (using stream mode for efficiency)
        results = model(frame, stream=True)
        
        cup_detected = False
        best_bbox = None
        best_confidence = 0

        # Process detections: look for cups (COCO class index 41)
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
            # Compute the center of the bounding box and the frame
            bbox_center_x = (x_min + x_max) / 2
            frame_center_x = FRAME_WIDTH / 2
            error = frame_center_x - bbox_center_x  # positive if cup is left of center

            # Compute steering command from error (proportional control)
            if abs(error) < ERROR_DEADBAND:
                steer_cmd = 0
                print("Cup centered. Steering straight.")
            else:
                steer_cmd = KP_TURN * error
                # Clamp the steering command
                if steer_cmd > MAX_STEER:
                    steer_cmd = MAX_STEER
                elif steer_cmd < -MAX_STEER:
                    steer_cmd = -MAX_STEER
                print(f"Error: {error:.2f} pixels, Steering command: {steer_cmd:.2f}°")
            
            # Apply the steering command via the front wheel servo
            px.set_dir_servo_angle(steer_cmd)
            
            # Draw the bounding box and label on the frame for visualization
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(frame, f"Cup {best_confidence:.2f}", (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")
        else:
            # If no cup is detected, keep steering straight.
            px.set_dir_servo_angle(0)
            print("No cups detected.")

        # Display the camera view on your laptop
        cv2.imshow("Cup Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and tracking stopped by user.")

finally:
    # Stop the drivetrain and reset steering
    px.forward(0)
    px.set_dir_servo_angle(0)
    picam2.stop_preview()
    picam2.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()