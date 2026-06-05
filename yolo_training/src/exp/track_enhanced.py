import cv2
from ultralytics import YOLO
import numpy as np

# Load the model
model = YOLO('best.pt') 

# Open Webcam
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# --- THE MAGIC FILTER (CLAHE) ---
def enhance_contrast(image):
    # Convert to LAB color space (separate Lightness from Color)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    
    # Apply CLAHE to the 'L' (Lightness) channel
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    
    # Merge back and convert to BGR
    limg = cv2.merge((cl, a, b))
    final = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    return final

print("Starting Enhanced Tracking... Press 'q' to exit.")

while cap.isOpened():
    success, frame = cap.read()
    if success:
        # 1. Apply the "Digital Glasses" to fix lighting
        enhanced_frame = enhance_contrast(frame)
        
        # 2. Run Tracking on the ENHANCED image
        results = model.track(
            enhanced_frame, 
            persist=True, 
            tracker="robust_botsort.yaml", # Use your existing yaml
            device=0,
            half=True,
            verbose=False
        )
        
        # 3. Visualize
        annotated_frame = results[0].plot()
        
        # Show both specific windows to compare
        cv2.imshow("Original (What you see)", frame)
        cv2.imshow("Enhanced (What AI sees)", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()