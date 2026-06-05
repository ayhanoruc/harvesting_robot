import cv2
from ultralytics import YOLO

# --- Load the Model ---
model = YOLO('best.pt') 

# --- Open Video ---
video_path = 'test_video.mp4'
cap = cv2.VideoCapture(video_path)

# Video Properties
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# --- Setup Output ---
out = cv2.VideoWriter(
    'output_tracking.mp4', 
    cv2.VideoWriter_fourcc(*'mp4v'), 
    fps, 
    (w, h)
)

print(f"Tracking started on {video_path}...")

while cap.isOpened():
    success, frame = cap.read()
    if success:
        # --- THE CHANGE IS HERE ---
        # 1. Use model.track() instead of predict()
        # 2. Add persist=True to keep IDs between frames
        results = model.track(
            frame, 
            persist=True,      # <--- CRITICAL: Keeps ID numbers (ID: 1, ID: 2...)
            device=0,          # Use your RTX 3050
            half=True,         # Faster math
            imgsz=640,
            conf=0.5,          # Confidence threshold
            tracker="bytetrack.yaml", # Options: "botsort.yaml" (default) or "bytetrack.yaml"
            verbose=False
        )
        
        # Visualize the results (YOLO automatically draws the ID numbers now)
        annotated_frame = results[0].plot()
        
        # Write to file
        out.write(annotated_frame)
        
        # Show live
        cv2.imshow("YOLO Tracking", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
out.release()
cv2.destroyAllWindows()
print("Done! Tracking video saved as 'output_tracking.mp4'")