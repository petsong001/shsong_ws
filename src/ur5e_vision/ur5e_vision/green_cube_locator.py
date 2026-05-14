#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
import tf2_ros
from tf2_geometry_msgs import do_transform_point

class FinalLocator(Node):
    def __init__(self):
        super().__init__('green_cube_locator')
        self.coord_pub = self.create_publisher(Point, '/vision/cube_coordinates', 10)
        
        # Distance from camera to the ground (Z-height)
        self.estimated_depth = 0.876 
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos_profile=qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, qos_profile=qos_profile_sensor_data)
        
        # TF Buffer (Ready for use if you switch to dynamic transforms)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.camera_model = None
        
        self.get_logger().info("✅ VISION NODE STARTED (Synced with Isaac Sim 16:9 Aspect Ratio)")

    def info_callback(self, msg):
        if self.camera_model is None: self.camera_model = msg

    def image_callback(self, msg):
        if self.camera_model is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([35, 50, 50]), np.array([85, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 100:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    self.process_coordinates(cx, cy, cv_image)

        cv2.imshow("Strict Locator", cv_image)
        cv2.waitKey(1)

    def process_coordinates(self, u, v, image):
        # 1. Unpack Camera Intrinsics
        fx = self.camera_model.k[0]
        fy = self.camera_model.k[4]
        cx = self.camera_model.k[2]
        cy = self.camera_model.k[5]
        z = self.estimated_depth
        
        # 2. Calculate Raw Position in Camera Frame (Relative to Camera Center)
        # Note: We trust fy now because Isaac Sim Vertical Aperture was fixed.
        x_relative = (u - cx) * z / fx
        y_relative = -1*(v - cy) * z / fy

        # ---------------------------------------------------------
        # 🔧 TRANSFORM TO WORLD FRAME
        # ---------------------------------------------------------
        
        # METHOD A: HARDCODED (SIMPLEST & ROBUST)
        # Since we know the camera is fixed at X=0.5, Y=0.33 in Sim:
        
        final_x = x_relative + 0.50  
        final_y = y_relative + 0.33

        # METHOD B: USING TF (ADVANCED)
        # If you want to use the TF tree instead of hardcoding, uncomment this:
        """
        try:
            # Create a Point in Camera Frame
            p_cam = PointStamped()
            p_cam.header.frame_id = self.camera_model.header.frame_id
            p_cam.header.stamp = rclpy.time.Time().to_msg()
            p_cam.point.x = x_relative
            p_cam.point.y = y_relative
            p_cam.point.z = z

            # Transform to 'base_link' (or 'world')
            transform = self.tf_buffer.lookup_transform('base_link', p_cam.header.frame_id, rclpy.time.Time())
            p_base = do_transform_point(p_cam, transform)
            
            final_x = p_base.point.x
            final_y = p_base.point.y
        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")
            return
        """
        # ---------------------------------------------------------

        text = f"X:{final_x:.2f} Y:{final_y:.2f}"
        cv2.putText(image, text, (u+10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        
        msg = Point()
        msg.x, msg.y, msg.z = final_x, final_y, z
        self.coord_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FinalLocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
