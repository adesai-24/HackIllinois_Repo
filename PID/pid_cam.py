#!/usr/bin/env python3
import time
import cv2
import RPi.GPIO as GPIO
from ultralytics import YOLO
from picamera2 import Picamera2
from picarx import Picarx

# Configuration & Constants
SAMPLETIME = 0.1             # Loop interval (seconds)
KP_TURN = 0.2                # Proportional gain for steering
ERROR_DEADBAND = 20          # Pixel threshold below which cup is considered centered
MAX_TURN_SPEED = 50          # Maximum steering command (degrees)
FORWARD_SPEED = 50           # Forward motor speed when driving forward
FORWARD_DURATION = 0.1       # Duration (in seconds) to drive forward when cup detected

FRAME_WIDTH = 640            # Camera frame width (pixels)
FRAME_HEIGHT = 480           # Camera frame height (pixels)

# Initialize Components
px = Picarx()
model = YOLO("yolov5su.pt")  # Using the improved 'u' model

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (FRAME_WIDTH, FRAME_HEIGHT)}
)
picam2.configure(config)
# Optional: start preview if a display is attached
picam2.start_preview()
picam2.start()

print("Starting cup detection and tracking. Press 'q' to exit.\n")

try:
    while True:
        # Capture a frame and drop the alpha channel to convert from 4-channel XRGB to 3-channel RGB
        frame = picam2.capture_array()
        frame = frame[:, :, :3]

        # Run YOLO inference (stream mode for efficiency)
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
            # Extract bounding box and compute centers
            x_min, y_min, x_max, y_max = map(int, best_bbox)
            bbox_center_x = (x_min + x_max) / 2
            frame_center_x = FRAME_WIDTH / 2
            error = frame_center_x - bbox_center_x

            # Compute steering command (proportional to error)
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
            
            # Apply the steering command via the front wheel servo
            px.set_dir_servo_angle(steer_cmd)
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")

            # Drive forward for a short pulse in the direction of the cup
            px.forward(FORWARD_SPEED)
            time.sleep(FORWARD_DURATION)
            px.forward(0)

            # Optionally, draw the bounding box on the frame for visualization
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            cv2.putText(frame, f"Cup {best_confidence:.2f}", (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            # If no cup is detected, reset steering and stop forward motion.
            px.set_dir_servo_angle(0)
            px.forward(0)
            print("No cups detected.")

        # Optionally, display the frame (if a display is available)
        try:
            cv2.imshow("Cup Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        except cv2.error:
            # If display functions are unavailable (headless mode), ignore.
            pass

        print("-" * 60)
        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and control stopped by user.")

finally:
    # Reset steering and drive to 0, then clean up resources
    px.set_dir_servo_angle(0)
    px.forward(0)
    picam2.stop_preview()
    picam2.stop()
    GPIO.cleanup()
    try:
        cv2.destroyAllWindows()
    except cv2.error:
        pass
