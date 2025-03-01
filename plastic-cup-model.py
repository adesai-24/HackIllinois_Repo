import cv2
from ultralytics import YOLO

# Load the pretrained YOLOv5s model (trained on COCO)
model = YOLO("yolov5s.pt") 

cap = cv2.VideoCapture(0)  

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture frame.")
        break

    results = model(frame, stream=True)

    for result in results:
        for box in result.boxes:
            # Using model.names can be helpful to see what each index means:
            # print(model.names) 
            cls_index = int(box.cls[0]) if box.cls is not None else -1
            # Check if the detected class is "cup" (COCO index 41)
            if cls_index == 41:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                confidence = box.conf[0]  # Confidence score
                label = f"Cup {confidence:.2f}"
                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (0, 255, 0), 2)

    # Display the frame
    cv2.imshow("YOLOv5 Cup Detection", frame)

    # Quit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
