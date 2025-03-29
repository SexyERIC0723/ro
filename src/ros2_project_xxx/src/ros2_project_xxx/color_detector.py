
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        
        self.bridge = CvBridge()
        
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        self.red_box_pub = self.create_publisher(
            Float32MultiArray,
            'red_box_detection',
            10)
        
        self.green_box_pub = self.create_publisher(
            Float32MultiArray,
            'green_box_detection',
            10)
        
        self.blue_box_pub = self.create_publisher(
            Float32MultiArray,
            'blue_box_detection',
            10)
        
        self.processed_image_pub = self.create_publisher(
            Image,
            'processed_image',
            10)
        
        self.red_lower = np.array([0, 100, 100])
        self.red_upper = np.array([10, 255, 255])
        
        self.green_lower = np.array([40, 100, 100])
        self.green_upper = np.array([80, 255, 255])
        
        self.blue_lower = np.array([100, 100, 100])
        self.blue_upper = np.array([140, 255, 255])
        
        self.get_logger().info('Color detector initialized')
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        output_image = cv_image.copy()
        
        red_mask = cv2.inRange(hsv_image, self.red_lower, self.red_upper)
        self.detect_and_publish(red_mask, cv_image, output_image, self.red_box_pub, (0, 0, 255), "Red")
        
        green_mask = cv2.inRange(hsv_image, self.green_lower, self.green_upper)
        self.detect_and_publish(green_mask, cv_image, output_image, self.green_box_pub, (0, 255, 0), "Green")
        
        blue_mask = cv2.inRange(hsv_image, self.blue_lower, self.blue_upper)
        self.detect_and_publish(blue_mask, cv_image, output_image, self.blue_box_pub, (255, 0, 0), "Blue")
        
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(output_image, "bgr8")
            processed_msg.header = msg.header
            self.processed_image_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to convert processed image: {e}')
    
    def detect_and_publish(self, mask, original_image, output_image, publisher, color, label):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detection_msg = Float32MultiArray()
        detection_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0]  # [detected, center_x, center_y, size, distance_estimate]
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > 100:
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(output_image, (x, y), (x + w, y + h), color, 2)
                    
                    cv2.circle(output_image, (cx, cy), 5, color, -1)
                    
                    cv2.putText(output_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                    distance_estimate = 1000.0 / (w * h)  # Just an example formula
                    
                    detection_msg.data = [1.0, float(cx), float(cy), float(area), float(distance_estimate)]
                    
                    self.get_logger().debug(f'Detected {label} box at ({cx}, {cy}) with area {area}')
        
        publisher.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    
    color_detector = ColorDetector()
    
    try:
        rclpy.spin(color_detector)
    except KeyboardInterrupt:
        pass
    finally:
        color_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
