#!/usr/bin/env python3
import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import time

# Load a smaller model if available (change to your model file as needed)
model = YOLO("yolov8n.pt")
model.model.eval()  # ensure evaluation mode

# Initialize the robot camera using Picamera2
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

print("Press 'q' to exit.")

while True:
    # Capture a frame from the robot camera as a numpy array
    frame = picam2.capture_array()
    
    # Optionally, resize the frame for faster inference
    frame_resized = cv2.resize(frame, (320, 240))
    
    # Ensure the frame is of type uint8
    frame_resized = frame_resized.astype('uint8')
    
    # Run YOLO inference on the resized frame
    results = model(frame_resized, stream=True)
    
    # Process detections
    for result in results:
        for box in result.boxes:
            cls_index = int(box.cls[0]) if box.cls is not None else -1
            # Check for class index 41 (cup for COCO) or adjust as needed
            if cls_index == 41:
                # Map bounding box coordinates back to original frame size if needed
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0]
                label = f"Cup {confidence:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("Robot Camera YOLO Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
