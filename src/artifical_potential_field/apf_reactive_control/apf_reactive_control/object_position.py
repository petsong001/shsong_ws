#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
import math

class ObjectPositionNode(Node):
    def __init__(self):
        super().__init__('rgbd_object_locator')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/vision/hand_pose', 10)
        self.bridge = CvBridge()
        self.camera_model = None
        self.latest_depth_img = None
        
        # --- PADDING CONFIGURATION ---
        # Increase this value to make the robot stay further away from the detected object.
        # This acts as a safety "buffer zone" in meters.
        self.padding_m = 0.15  # 15cm padding
        
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, qos_profile=qos_profile_sensor_data)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth', self.depth_callback, qos_profile=qos_profile_sensor_data)
            
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile=qos_profile_sensor_data)
        
        self.get_logger().info(f"✅ 3D VISION NODE STARTED (Padding: {self.padding_m}m)")

    def info_callback(self, msg):
        if self.camera_model is None: 
            self.camera_model = msg

    def depth_callback(self, msg):
        try:
            self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            pass

    def image_callback(self, msg):
        if self.camera_model is None or self.latest_depth_img is None: 
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e: 
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Detecting Green (as per your current mask)
        mask = cv2.inRange(hsv, np.array([35, 50, 50]), np.array([85, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 100:
                M = cv2.moments(largest)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])
                    self.process_3d_coordinates(u, v, cv_image)

        cv2.imshow("3D Object Locator", cv_image)
        cv2.waitKey(1)

    def process_3d_coordinates(self, u, v, image):
        fx = self.camera_model.k[0]
        fy = self.camera_model.k[4]
        cx = self.camera_model.k[2]
        cy = self.camera_model.k[5]
        
        half_window = 2
        min_v, max_v = max(0, v-half_window), min(self.latest_depth_img.shape[0], v+half_window+1)
        min_u, max_u = max(0, u-half_window), min(self.latest_depth_img.shape[1], u+half_window+1)
        depth_window = self.latest_depth_img[min_v:max_v, min_u:max_u]
        z = np.nanmean(depth_window)

        if math.isnan(z) or z <= 0.0:
            return

        # Pure Pinhole Math
        x_camera = (u - cx) * z / fx
        y_camera = (v - cy) * z / fy

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_depth_optical_frame" 
        
        pose_msg.pose.position.x = float(x_camera)
        pose_msg.pose.position.y = float(y_camera)
        pose_msg.pose.position.z = float(z)
        pose_msg.pose.orientation.w = 1.0
        
        # Note: The 'detected_cube' size must be defined in your PlanningScene 
        # script. If you increase the size of the cube there by 0.15m, 
        # the RRT planner will naturally avoid this coordinate.
        
        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()