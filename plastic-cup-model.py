#!/usr/bin/env python3
import time
from ultralytics import YOLO
from picamera2 import Picamera2

# Load the YOLOv5s model (you can change this to your model file)
model = YOLO("yolov5s.pt")

# Initialize the robot camera using Picamera2
picam2 = Picamera2()
# Create a preview configuration at 640x480 with a 4-channel (XRGB) format
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Starting cup detection. Press Ctrl+C to exit.")

try:
    while True:
        # Capture a frame from the robot camera as a numpy array
        frame = picam2.capture_array()
        # Convert from 4 channels to 3 channels (drop the alpha channel)
        frame = frame[:, :, :3]

        # Run YOLO inference on the frame (stream mode for efficiency)
        results = model(frame, stream=True)

        cup_count = 0
        probabilities = []
        for result in results:
            for box in result.boxes:
                # Get the detected class index (COCO index 41 for cup)
                cls_index = int(box.cls[0]) if box.cls is not None else -1
                if cls_index == 41:
                    cup_count += 1
                    probabilities.append(box.conf[0])
        
        # Output the results to the console
        if cup_count > 0:
            prob_str = ", ".join([f"{prob:.2f}" for prob in probabilities])
            print(f"Detected {cup_count} cup(s) with probabilities: {prob_str}")
        else:
            print("No cups detected")
        
        time.sleep(1)  # wait 1 second between frames

except KeyboardInterrupt:
    print("\nDetection stopped by user.")

finally:
    picam2.stop()
