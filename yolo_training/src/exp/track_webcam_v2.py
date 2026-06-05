import cv2
from ultralytics import YOLO

model = YOLO('best.pt') 

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print("Starting Robust BoT-SORT Tracking...")

while cap.isOpened():
    success, frame = cap.read()
    if success:
        results = model.track(
            frame, 
            persist=True, 
            tracker="robust_botsort.yaml", # Use the Re-ID tracker
            device=0,
            half=True,
            
            # --- STABILITY IMPROVEMENTS ---
            agnostic_nms=True,  # Prevents multiple boxes on one cotton boll
            verbose=False
        )
        
        annotated_frame = results[0].plot()
        cv2.imshow("Robust Tracking", annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        break

cap.release()
cv2.destroyAllWindows()