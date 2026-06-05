import cv2
from ultralytics import YOLO

# Load Model
model = YOLO('best.pt')

# Open Webcam (Source 0)
cap = cv2.VideoCapture(0)

# Set Resolution (Optional - improves detection distance)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("Starting Clean Webcam Tracking... Press 'q' to exit.")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Failed to read from webcam.")
        break

    # --- Run Sticky Tracking ---
    # source=frame: Explicitly tells YOLO what image to use
    # persist=True: Essential for keeping IDs alive
    # tracker="sticky_tracker.yaml": Uses your robust settings
    results = model.track(
        source=frame, 
        persist=True, 
        tracker="sticky_tracker.yaml", 
        verbose=False
    )

    # --- Visualize ---
    # This draws the clean Bounding Box + ID Number (No trails)
    annotated_frame = results[0].plot()

    # Show live feed
    cv2.imshow("Robust Webcam Tracking", annotated_frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()