#!/usr/bin/env python3
import time
import cv2
import RPi.GPIO as GPIO
from ultralytics import YOLO
from picamera2 import Picamera2
from picarx import Picarx
from robot_hat import Music

# Configuration & Constants
SAMPLETIME = 0.1             # Loop interval (seconds)
KP_TURN = 0.2                # Proportional gain for steering
ERROR_DEADBAND = 20          # Pixel threshold below which cup is considered centered
MAX_TURN_SPEED = 50          # Maximum steering command (degrees)
FORWARD_SPEED = 50           # Forward motor speed when driving forward
FORWARD_DURATION = 1        # Duration (seconds) to drive forward when cup detected

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
# Optionally start preview if you have a display; omit if headless
# picam2.start_preview()
picam2.start()

# Initialize Music for sound playback
music = Music()
music.music_set_volume(60)
print("Starting cup detection, tracking, and forward drive. Press Ctrl+C to exit.\n")

# Flag to ensure sound is played only once per detection cycle
sound_played = False

try:
    while True:
        # Capture a frame and convert from 4-channel (XRGB) to 3-channel (RGB)
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
            # Extract bounding box and compute the horizontal center
            x_min, y_min, x_max, y_max = map(int, best_bbox)
            bbox_center_x = (x_min + x_max) / 2
            frame_center_x = FRAME_WIDTH / 2
            error = frame_center_x - bbox_center_x

            # Compute steering command using proportional control
            if abs(error) < ERROR_DEADBAND:
                steer_cmd = 0
                print("Cup centered. Steering straight.")
            else:
                steer_cmd = KP_TURN * error
                if steer_cmd > MAX_TURN_SPEED:
                    steer_cmd = MAX_TURN_SPEED
                elif steer_cmd < -MAX_TURN_SPEED:
                    steer_cmd = -MAX_TURN_SPEED
                print(f"Error: {error:.2f} pixels, Steering command: {steer_cmd:.2f}°")
            
            # Apply steering via the front wheel servo
            px.set_dir_servo_angle(steer_cmd)
            print(f"Cup detected: BBox=({x_min}, {y_min}, {x_max}, {y_max}), Confidence={best_confidence:.2f}")

            # Play a sound if not already played this detection cycle
            if not sound_played:
                music.sound_play('/home/pi/HackIllinois_Repo/audio')
                sound_played = True

            # Drive forward for a short pulse in the current steering direction
            px.forward(FORWARD_SPEED)
            time.sleep(FORWARD_DURATION)
            px.forward(0)
        else:
            # No cup detected: reset steering, stop forward motion, and reset sound flag
            px.set_dir_servo_angle(0)
            px.forward(0)
            sound_played = False
            print("No cups detected.")

        # Print a separator for clarity in the console
        print("-" * 60)
        time.sleep(SAMPLETIME)

except KeyboardInterrupt:
    print("\nDetection and control stopped by user.")

finally:
    # Reset steering and forward motion, then clean up
    px.set_dir_servo_angle(0)
    px.forward(0)
    picam2.stop()
    GPIO.cleanup()
    # No cv2.destroyAllWindows() needed if no windows are created
