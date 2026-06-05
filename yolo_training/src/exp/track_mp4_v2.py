import cv2
from ultralytics import YOLO

# --- Configuration ---
input_video = 'test_video.mp4'       # Replace with your video file name
output_video = 'final_output.mp4'    # The name of the saved video
model_path = 'best.pt'               # Your trained model

# Load Model
model = YOLO(model_path)

# Open Video File
cap = cv2.VideoCapture(input_video)

# Get Video Properties (so output matches input)
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# Setup Video Writer
out = cv2.VideoWriter(
    output_video, 
    cv2.VideoWriter_fourcc(*'mp4v'), 
    fps, 
    (w, h)
)

print(f"Processing {input_video}... Press 'q' to stop early.")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break

    # --- Run Sticky Tracking ---
    # We use the robust yaml file to keep IDs solid
    # We remove the 'trails' logic to keep the video clean
    results = model.track(
        source=frame, 
        persist=True, 
        tracker="sticky_tracker.yaml", 
        verbose=False
    )

    # --- Visualize ---
    # plot() draws the bounding box and the ID number automatically
    annotated_frame = results[0].plot()

    # Write to file
    out.write(annotated_frame)

    # Show live (Optional - you can comment this out for faster speed)
    cv2.imshow("Robust Tracking", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
print(f"Done! Saved as {output_video}")