# YOLO Object Detection Message
# Contains information about a single detected object

int32 class_id          # COCO class ID (e.g., 47=apple, 49=orange, 39=bottle, 41=cup, 74=clock)
string class_name       # Human-readable class name (e.g., "apple", "orange")
float32 confidence      # Detection confidence score (0.0 to 1.0)
int32[] bbox            # Bounding box [x, y, width, height] in pixels
