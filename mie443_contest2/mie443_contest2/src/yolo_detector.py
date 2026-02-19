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
  

        return response


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
