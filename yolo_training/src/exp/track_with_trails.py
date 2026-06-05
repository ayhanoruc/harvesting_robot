import cv2
import numpy as np  # <--- THIS WAS MISSING
from ultralytics import YOLO
from collections import deque

# Load Model
model = YOLO('best.pt')

# Initialize Webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# --- TRAIL CONFIGURATION ---
# Dictionary to store the history of center points for each ID
track_history = {} 
TRAIL_LENGTH = 30  # How long the tail should be (in frames)

print("Starting Tracking with Trails... Press 'q' to exit.")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # Run Tracking with the "Sticky" Config
    # source=frame fixes the "WARNING source is missing"
    results = model.track(source=frame, persist=True, tracker="sticky_tracker.yaml", verbose=False)

    # Get the boxes and IDs
    if results[0].boxes.id is not None:
        boxes = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id.int().cpu().tolist()

        # Loop through each detected object
        for box, track_id in zip(boxes, track_ids):
            x, y, w, h = box
            center = (float(x), float(y))

            # 1. Update the trail history
            if track_id not in track_history:
                track_history[track_id] = deque(maxlen=TRAIL_LENGTH)
            track_history[track_id].append(center)

            # 2. Draw the Trail
            points = track_history[track_id]
            for i in range(1, len(points)):
                if points[i - 1] is None or points[i] is None:
                    continue
                
                # Math to make the line fade out (requires numpy)
                thickness = int(np.sqrt(TRAIL_LENGTH / float(i + 1)) * 2)
                
                cv2.line(frame, 
                         (int(points[i-1][0]), int(points[i-1][1])), 
                         (int(points[i][0]), int(points[i][1])), 
                         (0, 255, 0), thickness)

            # 3. Draw the Bounding Box & ID
            top_left_x = int(x - w / 2)
            top_left_y = int(y - h / 2)
            bottom_right_x = int(x + w / 2)
            bottom_right_y = int(y + h / 2)
            
            cv2.rectangle(frame, (top_left_x, top_left_y), (bottom_right_x, bottom_right_y), (0, 255, 0), 2)
            # Put text slightly above the box
            cv2.putText(frame, f"ID: {track_id}", (top_left_x, top_left_y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Show the result
    cv2.imshow("Professional Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()