from ultralytics import YOLO

# Load your custom trained model
model = YOLO('runs/detect/train2/weights/best.pt')

# Run inference on a video file
# Replace 'test_video.mp4' with the name of your video file
results = model.predict(
    source='test_video.mp4', 
    save=True,     # Save the video with boxes drawn
    show=True,     # Show the video while processing (press 'q' to exit window)
    conf=0.5       # Only show detections with > 50% confidence
)

print("Processing complete. Saved video is in 'runs/detect/predict/'")