import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import ultralytics

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        # Initialize YOLO model once during node creation
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        # Create window once
        cv2.namedWindow('YOLO Object Detection', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        self.get_logger().info('Image received')
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.detect_objects(cv_image)

    # def detect_objects(self, image):
    #     # Convert image to grayscale
    #     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
    #     # Apply Gaussian blur to reduce noise
    #     blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
    #     # Use Canny edge detection
    #     edges = cv2.Canny(blurred, 50, 150)
        
    #     # Find contours
    #     contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
    #     # Draw contours and detect objects
    #     for contour in contours:
    #         # Filter small contours
    #         if cv2.contourArea(contour) > 500:  # Minimum area threshold
    #             # Get bounding rectangle
    #             x, y, w, h = cv2.boundingRect(contour)
                
    #             # Draw rectangle around object
    #             cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
    #             # Log detection
    #             self.get_logger().info(f'Object detected at position ({x}, {y})')
        
    #     # Display the processed image
    #     cv2.imshow('Object Detection', image)
    #     cv2.waitKey(1)

    def detect_objects(self, image):
        # Convert OpenCV image to RGB (YOLO expects RGB)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Run inference
        results = self.model(image_rgb)
        
        # Get detections
        detections = results.pandas().xyxy[0]  # Get detection results as pandas DataFrame
        
        # Draw bounding boxes and labels
        for idx, detection in detections.iterrows():
            x1, y1, x2, y2 = int(detection['xmin']), int(detection['ymin']), int(detection['xmax']), int(detection['ymax'])
            conf = detection['confidence']
            label = detection['name']
            
            # Draw rectangle
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Add label with confidence
            label_text = f'{label}: {conf:.2f}'
            cv2.putText(image, label_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Log detection
            self.get_logger().info(f'Detected {label} with confidence {conf:.2f} at position ({x1}, {y1})')
        
        # Update the existing window
        cv2.imshow('YOLO Object Detection', image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 