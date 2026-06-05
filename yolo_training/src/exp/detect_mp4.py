import cv2
from ultralytics import YOLO

# --- Step 1: Load Model & Video ---
# Make sure to point to your trained model
model = YOLO('best.pt') 
video_path = 'test_video.mp4' # The video you want to test

# Open the video file
cap = cv2.VideoCapture(video_path)

# --- Step 2: Read Video Properties ---
# We need these to ensure the output video looks exactly like the input
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# --- Step 3: Initialize the MP4 Writer ---
# 'mp4v' is the standard codec for MP4 files
out = cv2.VideoWriter(
    'output_result.mp4',          # Output filename
    cv2.VideoWriter_fourcc(*'mp4v'), # Codec
    fps,
    (w, h)
)

print("Processing video... This might take a minute.")

while cap.isOpened():
    success, frame = cap.read()
    
    if success:
        # Run YOLO inference on the frame
        results = model(frame)
        
        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        
        # Write the processed frame to the mp4 file
        out.write(annotated_frame)
        
        # Optional: Show the video live while processing
        cv2.imshow("YOLOv8 Inference", annotated_frame)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    else:
        # Break the loop if the video ends
        break

# --- Step 4: Cleanup ---
cap.release()
out.release()
cv2.destroyAllWindows()

print("Done! Video saved as 'output_result.mp4'")