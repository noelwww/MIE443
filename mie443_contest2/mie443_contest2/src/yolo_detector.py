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
from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')
        self.get_logger().info('YOLO model loaded')
        
        # Confidence threshold
        self.confidence_threshold = 0.5

        # Only detect these contest objects (COCO class names)
        self.allowed_objects = {'cup', 'motorcycle', 'clock', 'potted plant', 'bottle'}
        
        # Create service
        self.service = self.create_service(
            DetectObject,
            'detect_object',
            self.detect_callback
        )
        
        self.get_logger().info('YOLO Detector Service ready')

    def detect_callback(self, request, response):
        """Process image and return highest confidence detection."""
        
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
        
        # Run YOLO inference
        results = self.model(image, verbose=False, device='cpu')
        boxes = results[0].boxes

        ### YOUR CODE HERE ###
        best_conf = 0.0
        best_class_id = -1
        best_class_name = ""
        best_box = None

        for box in boxes:
            conf = float(box.conf[0])
            class_id = int(box.cls[0])
            class_name = self.model.names[class_id]
            if class_name not in self.allowed_objects:
                continue
            if conf > self.confidence_threshold and conf > best_conf:
                best_conf = conf
                best_class_id = class_id
                best_class_name = class_name
                best_box = box

        if best_box is not None:
            response.success = True
            response.class_id = best_class_id
            response.class_name = best_class_name
            response.confidence = best_conf
            response.message = "Detection successful"

            if request.save_detected_image:
                # Draw bounding box
                x1, y1, x2, y2 = map(int, best_box.xyxy[0])
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{best_class_name} {best_conf:.2f}"
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.imwrite("detected_object.jpg", image)
                self.get_logger().info("Saved annotated image to detected_object.jpg")
        else:
            response.success = False
            response.class_id = -1
            response.class_name = ""
            response.confidence = 0.0
            response.message = "No object detected above threshold"

            if request.save_detected_image:
                # Save the raw image anyway so the user can inspect what the camera sees
                cv2.imwrite("detected_object_raw.jpg", image)
                self.get_logger().info("No detection — saved raw image to detected_object_raw.jpg")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
