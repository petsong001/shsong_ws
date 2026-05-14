#!/usr/bin/env python3
import rclpy
import time
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

class ColorPicker(Node):
    def __init__(self):
        super().__init__('color_picker_node')
        
        # --- TRANSFORM MATRIX ---
        self.T_cam_to_base = np.array([
            [-6.01097e-6, -7.54586e-8, -1.0,         0.499999],
            [-9.18783e-7,  1.0,        -7.54531e-8,  0.33],
            [ 1.0,         9.18782e-7, -6.01097e-6,  0.889998],
            [ 0.0,         0.0,         0.0,         1.0]
        ])

        self.bridge = CvBridge()
        
        # 1. Subscribe to Camera Info
        self.cam_info_sub = self.create_subscription(
            CameraInfo, 
            '/zed/zed_node/rgb/color/rect/camera_info', 
            self.info_callback, 
            qos_profile=qos_profile_sensor_data
        )
        self.cam_info = None

        # 2. Subscribe to Color Image
        self.image_sub = self.create_subscription(
            Image, 
            '/zed/zed_node/rgb/color/rect/image', 
            self.image_callback, 
            qos_profile=qos_profile_sensor_data
        )
        self.latest_cv_image = None
        
        # 3. Subscribe to Depth Image
        self.depth_sub = self.create_subscription(
            Image, 
            '/zed/zed_node/depth/depth_registered', 
            self.depth_callback, 
            qos_profile=qos_profile_sensor_data
        )
        self.latest_depth_img = None

        self.move_group = ActionClient(self, MoveGroup, 'move_action')
        self.gripper_open = self.create_client(Trigger, '/hande_gripper_controller_node/open')
        self.gripper_close = self.create_client(Trigger, '/hande_gripper_controller_node/close')
        
        self.get_logger().info("Color Picker Ready! Waiting for data...")

    def info_callback(self, msg): self.cam_info = msg 
    
    def image_callback(self, msg):
        try: self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: pass

    def depth_callback(self, msg):
        try: self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except: pass

    def control_gripper(self, open=True):
        client = self.gripper_open if open else self.gripper_close
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Gripper Service NOT found. Skipping gripper command.")
            return
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Gripper {'Opened' if open else 'Closed'}")

    def find_green_object(self):
        if self.latest_cv_image is None:
            self.get_logger().warn("Waiting for Color Image...")
            return None
        if self.latest_depth_img is None:
            self.get_logger().warn("Waiting for Depth Image...")
            return None
        if self.cam_info is None:
            self.get_logger().warn("Waiting for Camera Info...")
            return None

        # Vision Processing
        hsv = cv2.cvtColor(self.latest_cv_image, cv2.COLOR_BGR2HSV)
        
        # Color Range
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        
        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            self.get_logger().info("No green color found.")
            return None

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        
        # Log the area size so we can see it
        self.get_logger().info(f"Saw Green Blob of size: {area}")

        # --- THE FIX: Lowered threshold from 500 to 50 ---
        if area < 50:
            self.get_logger().info("Green object still too small (ignored).")
            return None

        M = cv2.moments(largest_contour)
        if M["m00"] == 0: return None
        u = int(M["m10"] / M["m00"])
        v = int(M["m01"] / M["m00"])

        try: 
            depth = self.latest_depth_img[v, u] 
        except IndexError: 
            return None
        
        if np.isnan(depth) or depth <= 0.0:
            self.get_logger().warn(f"I see Green at ({u},{v}), but depth is invalid.")
            return None

        self.get_logger().info(f"Target Found! Pixel: ({u},{v}), Depth: {depth:.3f}m")

        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]

        X_cam = (u - cx) * depth / fx
        Y_cam = (v - cy) * depth / fy
        Z_cam = depth

        P_cam = np.array([X_cam, Y_cam, Z_cam, 1.0])
        P_base = self.T_cam_to_base @ P_cam

        return P_base[0], P_base[1], P_base[2] 

    def move_robot_linear(self, x, y, z):
        self.get_logger().info(f"MOVING TO: {x:.3f}, {y:.3f}, {z:.3f}")
        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.group_name = "ur_manipulator"
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        # Orientation: Down
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 1.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 0.0

        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"
        pos_constraint.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])]
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        pos_constraint.weight = 1.0
        
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.05
        ori_constraint.absolute_y_axis_tolerance = 0.05
        ori_constraint.absolute_z_axis_tolerance = 0.05
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        self.move_group.wait_for_server()
        future = self.move_group.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move Rejected!")
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)

def main(args=None):
    rclpy.init(args=args)
    picker = ColorPicker()
    
    print("Waiting for camera connection...")
    time.sleep(2.0)
    for _ in range(10): rclpy.spin_once(picker)

    picker.control_gripper(open=True)

    coords = None
    for i in range(20):
        rclpy.spin_once(picker)
        coords = picker.find_green_object()
        if coords: break
        time.sleep(0.1)

    if coords:
        x, y, z = coords
        print(f"Target locked at: {x:.3f}, {y:.3f}, {z:.3f}")
        picker.move_robot_linear(x, y, z + 0.15) 
        picker.move_robot_linear(x, y, z + 0.02) 
        picker.control_gripper(open=False)       
        picker.move_robot_linear(x, y, z + 0.20) 
    else:
        print("Could not find any GREEN object.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
