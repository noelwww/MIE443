#!/usr/bin/env python3
"""
YOLO Object Detection Service for MIE443 Contest 2
Detects objects using YOLOv8/v13 and returns the highest confidence detection.
"""

import rclpy
from rclpy.node import Node
from mie443_contest2.srv import DetectObject
import cv2
import numpy as np
import os
from ultralytics import YOLO

# Directory for saved detection images
DETECTION_DIR = os.path.expanduser('~/ros2_ws/detection_images')


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.25

        # How many top YOLO predictions to check for an allowed object.
        # If top-1 is "vase" but top-2 is "cup", we'll still match "cup".
        self.top_k_candidates = 3

        # Minimum bounding box area (in pixels) to accept a detection.
        # Rejects small detections of far-away objects that the robot is not near.
        # For a 640×480 image, 3000 px² ≈ ~55×55 px minimum.
        self.min_bbox_area = 3000

        # Only detect these contest objects (COCO class names)
        self.allowed_objects = {'cup', 'motorcycle', 'clock', 'potted plant', 'bottle'}

        # Fallback aliases: YOLO sometimes confuses certain objects.
        # If a detected class matches a key here, treat it as the mapped value.
        self.class_aliases = {
            'toilet': 'cup',
        }
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')
        
        # Sequential counter for saved images
        self.detection_count = 0
        os.makedirs(DETECTION_DIR, exist_ok=True)
        self.get_logger().info(f'Detection images will be saved to: {DETECTION_DIR}')

    def detect_callback(self, request, response):
        """Process image and return first allowed detection from top-K predictions."""
        
        # Decode compressed image
        np_arr = np.frombuffer(request.image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is None:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "Failed to decode image"
            return response

        # Flip 180° if requested (e.g. wrist camera mounted upside-down)
        if request.flip_image:
            image = cv2.rotate(image, cv2.ROTATE_180)
            self.get_logger().info('Image flipped 180° (upside-down camera correction)')

        # Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes

        # ---- Collect ALL valid detections, sorted by confidence ----
        all_detections = []
        for box in boxes:
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            if conf < self.confidence_threshold:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            bbox_area = (x2 - x1) * (y2 - y1)
            all_detections.append({
                'conf': conf,
                'class_id': class_id,
                'class_name': class_name,
                'box': box,
                'bbox_area': bbox_area,
                'coords': (x1, y1, x2, y2),
            })

        # Sort by confidence descending
        all_detections.sort(key=lambda d: d['conf'], reverse=True)

        # Log top-K for debugging (show what YOLO sees)
        top_k = min(self.top_k_candidates, len(all_detections))
        if all_detections:
            top_summary = ', '.join(
                f"{d['class_name']}({d['conf']:.2f}, {d['bbox_area']}px²)"
                for d in all_detections[:top_k]
            )
            self.get_logger().info(f'YOLO top-{top_k}: [{top_summary}]')
        else:
            self.get_logger().info('YOLO: no detections above confidence threshold')

        # ---- Find the first top-K candidate that is in allowed_objects ----
        best = None
        for d in all_detections[:top_k]:
            # Reject small bounding boxes (far-away objects)
            if d['bbox_area'] < self.min_bbox_area:
                self.get_logger().info(
                    f"Rejected {d['class_name']} ({d['conf']:.2f}): "
                    f"bbox area {d['bbox_area']}px² < {self.min_bbox_area}px² (too far)")
                continue
            # Check alias mapping first (e.g. toilet → cup)
            effective_name = self.class_aliases.get(d['class_name'], d['class_name'])
            if effective_name != d['class_name']:
                self.get_logger().info(
                    f"Alias: '{d['class_name']}' → '{effective_name}' (conf={d['conf']:.2f})")
                d['original_class_name'] = d['class_name']
                d['class_name'] = effective_name
            if d['class_name'] in self.allowed_objects:
                best = d
                break
            else:
                self.get_logger().info(
                    f"Top-K candidate '{d['class_name']}' ({d['conf']:.2f}) not in allowed list — skipping")

        if best is not None:
            response.success = True
            response.class_id = best['class_id']
            response.class_name = best['class_name']
            response.confidence = best['conf']
            response.message = "Detection successful"

            if request.save_detected_image:
                self.detection_count += 1
                # Draw top-K boxes in different colors for debugging
                colors = [(0, 255, 0), (0, 165, 255), (0, 0, 255)]  # green, orange, red
                for i, d in enumerate(all_detections[:top_k]):
                    x1, y1, x2, y2 = d['coords']
                    color = colors[i % len(colors)]
                    thickness = 3 if d is best else 1
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
                    # Show remapped name if aliased (e.g. "cup (was: toilet)")
                    display_name = d['class_name']
                    if 'original_class_name' in d:
                        display_name = f"{d['class_name']} (was: {d['original_class_name']})"
                    label = f"#{i+1} {display_name} {d['conf']:.2f} ({d['bbox_area']}px²)"
                    cv2.putText(image, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)
                fname = os.path.join(
                    DETECTION_DIR,
                    f'detect_{self.detection_count:03d}_{best["class_name"]}.jpg')
                cv2.imwrite(fname, image)
                self.get_logger().info(f'Saved annotated image to {fname}')
        else:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No object detected above threshold"

            if request.save_detected_image:
                self.detection_count += 1
                # Save the raw image anyway so the user can inspect what the camera sees
                fname = os.path.join(DETECTION_DIR, f'detect_{self.detection_count:03d}_NONE.jpg')
                cv2.imwrite(fname, image)
                self.get_logger().info(f'No detection — saved raw image to {fname}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
