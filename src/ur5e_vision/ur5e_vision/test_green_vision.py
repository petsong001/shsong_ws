#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class GreenVisionTester(Node):
    def __init__(self):
        super().__init__('green_vision_tester')
        self.bridge = CvBridge()
        
        # Subscribe to the camera using Best Effort (Sensor Data) QoS
        self.image_sub = self.create_subscription(
            Image, 
            '/zed/zed_node/rgb/color/rect/image', 
            self.image_callback, 
            qos_profile=qos_profile_sensor_data
        )
        self.get_logger().info("Vision Test Started! Press 'q' in the window to quit.")

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert to HSV color space
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Define Green Range (Adjust these if needed)
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            
            # Create a mask (White = Green, Black = Background)
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
            # Find contours just to draw a box around detected items
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            output_img = cv_image.copy()
            
            if contours:
                # Find the largest green object
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 500:
                    x, y, w, h = cv2.boundingRect(largest)
                    # Draw a green rectangle around it
                    cv2.rectangle(output_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(output_img, "Green Object", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            # Show the windows
            cv2.imshow("Original View", output_img)
            cv2.imshow("Green Mask (White=Seen)", mask)
            
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GreenVisionTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
