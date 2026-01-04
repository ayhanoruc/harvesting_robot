# Detected cotton cluster with 3D position
string cluster_id                    # Unique identifier
geometry_msgs/Point position         # 3D position in world frame
float32 confidence                   # Best detection confidence
int32 best_bbox_area                 # Area of best bounding box (highest = most complete)
int32 num_detections                 # How many times this cluster was detected
string best_scan_position            # Scan position name where best detection occurred
