import cv2
from ultralytics import YOLO

# --- Step 1: Load the Model ---
# Load your custom trained model
model = YOLO('best.pt') 

# --- Step 2: Open Webcam ---
# Source 0 is the default webcam. 
# If you have a USB camera, change this to 1.
cap = cv2.VideoCapture(0)

# Optional: Set Webcam Resolution (to ensure it doesn't default to low quality)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("Starting Webcam Tracking with ByteTrack... Press 'q' to exit.")

while cap.isOpened():
    success, frame = cap.read()
    
    if success:
        # --- Step 3: Run ByteTrack ---
        # persist=True:  Essential for tracking (remembers objects from previous frame)
        # tracker="bytetrack.yaml": Explicitly forces the ByteTrack algorithm
        results = model.track(
            frame, 
            persist=True,
            tracker="custom_tracker.yaml",  # <--- Specifying ByteTrack here
            device=0,                  # Run on your RTX 3050
            half=True,                 # Use FP16 (Faster)
            conf=0.5,                  # Only track confident detections
            verbose=False              # Keeps the terminal clean
        )
        
        # --- Step 4: Visualize ---
        # The plot() method draws the boxes and the ID numbers automatically
        annotated_frame = results[0].plot()
        
        # Show the live feed
        cv2.imshow("YOLOv11 Live ByteTrack", annotated_frame)
        
        # Break loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        print("Failed to read from webcam.")
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()