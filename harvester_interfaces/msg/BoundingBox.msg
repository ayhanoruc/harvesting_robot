# 2D Bounding Box from YOLO detection
int32 u_min           # Left pixel coordinate
int32 v_min           # Top pixel coordinate
int32 u_max           # Right pixel coordinate
int32 v_max           # Bottom pixel coordinate
float32 confidence    # Detection confidence [0-1]
string label          # Class label (e.g., "cotton")
int32 area            # Box area in pixels (width * height)
