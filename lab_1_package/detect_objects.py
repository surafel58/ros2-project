import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Image received')
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.detect_objects(cv_image)

    def detect_objects(self, image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Use Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw contours and detect objects
        for contour in contours:
            # Filter small contours
            if cv2.contourArea(contour) > 500:  # Minimum area threshold
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Draw rectangle around object
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                
                # Log detection
                self.get_logger().info(f'Object detected at position ({x}, {y})')
        
        # Display the processed image
        cv2.imshow('Object Detection', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 