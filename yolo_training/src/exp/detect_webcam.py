from ultralytics import YOLO

# Load your custom trained model
model = YOLO('runs/detect/train2/weights/best.pt')

# Run inference on live webcam
# source=0 is usually your default laptop webcam
results = model.predict(
    source=0,      
    show=True,     # Essential to see the live feed
    conf=0.89,      # Confidence threshold
    save=False     # Change to True if you want to record the webcam stream
)