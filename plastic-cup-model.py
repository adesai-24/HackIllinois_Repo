#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import time

# Load the pretrained YOLOv5s model (trained on COCO)
model = YOLO("yolov5s.pt") 

# Initialize the robot camera using Picamera2
picam2 = Picamera2()
# Create a preview configuration with desired resolution (e.g., 640x480)
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Press 'q' to exit.")

while True:
    # Capture a frame from the robot camera as a numpy array
    frame = picam2.capture_array()
    
    # Run the YOLO model inference on the frame; use stream=True for faster inference
    results = model(frame, stream=True)
    
    # Process detections in the results
    for result in results:
        for box in result.boxes:
            # Check the detected class index using model.names for reference if needed
            cls_index = int(box.cls[0]) if box.cls is not None else -1
            # For COCO, index 41 corresponds to "cup"
            if cls_index == 41:
                # Get bounding box coordinates and confidence score
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0]
                label = f"Cup {confidence:.2f}"
                # Draw bounding box and label on the frame
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (0, 255, 0), 2)

    # Display the processed frame on the computer
    cv2.imshow("Robot Camera YOLO Detection", frame)

    # Quit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Stop the camera and close OpenCV windows
picam2.stop()
cv2.destroyAllWindows()