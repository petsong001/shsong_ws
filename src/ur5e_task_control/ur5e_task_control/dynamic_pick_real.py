#!/usr/bin/env python3
import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import JointConstraint, Constraints
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.action import GripperCommand 
from tf2_ros import TransformException, Buffer, TransformListener

class UnifiedSmartPick(Node):
    def __init__(self):
        super().__init__('unified_smart_pick')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        self.target_sub = self.create_subscription(
            Pose, 
            '/vision/cube_coordinates', 
            self.vision_callback, 
            qos_profile
        )
        
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._gripper_client = ActionClient(self, GripperCommand, '/hande_gripper_controller/gripper_cmd')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.GROUP_NAME = 'ur_manipulator'
        self.WP1_HOME_JOINTS = [1.3435, -1.8282, 2.3295, -2.0909, -1.5556, -0.0243]
        
        self.GRIPPER_OPEN = 0.0    
        self.GRIPPER_CLOSED = 0.05 

        self.APPROACH_Z = 0.20        
        self.GRIPPER_OFFSET = 0.155 
        self.GRASP_Z = -0.005 + self.GRIPPER_OFFSET  
        
        # --- ROTATION TUNING ---
        # Set to 0 as requested. 
        # Increase this (positive) to rotate more Counter-Clockwise.
        # Decrease this (negative) to rotate more Clockwise.
        self.ROTATION_OFFSET = -1.57
        
        self.current_target = None
        
        self.get_logger().info("⏳ Connecting to Servers...")
        time.sleep(1.0)
        
        self._move_group_client.wait_for_server(timeout_sec=5.0)
        self._gripper_client.wait_for_server(timeout_sec=5.0)
        
        self.get_logger().info("🔄 Clearing Gripper Faults...")
        self.reset_gripper()
        
        self.get_logger().info("✅ Ready.")

    def reset_gripper(self):
        self.control_gripper(self.GRIPPER_OPEN, wait=True)
        time.sleep(0.5)

    def vision_callback(self, msg):
        self.current_target = msg

    def move_to_joints(self, joint_values):
        goal = MoveGroup.Goal()
        goal.request.group_name = self.GROUP_NAME
        c = Constraints(name="JointGoal")
        names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                 "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        
        for i, val in enumerate(joint_values):
            jc = JointConstraint(joint_name=names[i], position=val, weight=1.0)
            jc.tolerance_above = jc.tolerance_below = 0.01
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        
        send_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        if not send_future.result().accepted: return False
        
        res_future = send_future.result().get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def track_and_pick(self):
        try:
            self.get_logger().info("🚀 Moving to Home...")
            self.move_to_joints(self.WP1_HOME_JOINTS)
            
            self.get_logger().info("👀 Waiting for Vision Target...")
            while self.current_target is None and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            
            target_x = self.current_target.position.x
            target_y = self.current_target.position.y
            vision_q = self.current_target.orientation 
            
            self.get_logger().info(f"🔒 Target Locked at X:{target_x:.3f} Y:{target_y:.3f}")

            # 1. Extract the Yaw from the vision signal
            _, _, vision_yaw = self.quaternion_to_euler(vision_q)
            
            # 2. Build orientation: Roll=PI (face down), Pitch=0, Yaw=Object+Offset
            final_yaw = vision_yaw + self.ROTATION_OFFSET
            final_orientation = self.euler_to_quaternion(math.pi, 0, final_yaw)

            # --- WP2: HOVER ---
            wp2_hover = Pose()
            wp2_hover.position.x, wp2_hover.position.y = target_x, target_y
            wp2_hover.position.z = self.APPROACH_Z
            wp2_hover.orientation = final_orientation
            if not self.execute_cartesian_path([wp2_hover]): return

            # --- WP3: DIVE ---
            wp3_dive = Pose()
            wp3_dive.position.x, wp3_dive.position.y = target_x, target_y
            wp3_dive.position.z = self.GRASP_Z
            wp3_dive.orientation = final_orientation
            if not self.execute_cartesian_path([wp3_dive]): return
            
            # --- GRAB ---
            self.get_logger().info("⚡ GRAB!")
            self.control_gripper(self.GRIPPER_CLOSED, wait=False)
            time.sleep(0.6) 

            # --- WP4: LIFT ---
            wp4_lift = Pose()
            wp4_lift.position.x, wp4_lift.position.y = target_x, target_y
            wp4_lift.position.z = self.APPROACH_Z 
            wp4_lift.orientation = final_orientation
            if not self.execute_cartesian_path([wp4_lift]): return

            # --- WP5: HOME ---
            self.move_to_joints(self.WP1_HOME_JOINTS)

            # --- WP6: DROP ZONE ---
            try:
                t = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
                wp6_drop = Pose()
                wp6_drop.position.x, wp6_drop.position.y = t.transform.translation.x, t.transform.translation.y
                wp6_drop.position.z = self.GRASP_Z 
                wp6_drop.orientation = t.transform.rotation 
                self.execute_cartesian_path([wp6_drop])
                
                self.get_logger().info("🖐️ DROP!")
                self.control_gripper(self.GRIPPER_OPEN, wait=True)
                time.sleep(0.3) 
                
                wp6_safety = Pose()
                wp6_safety.position.x, wp6_safety.position.y = t.transform.translation.x, t.transform.translation.y
                wp6_safety.position.z = self.APPROACH_Z 
                wp6_safety.orientation = t.transform.rotation
                self.execute_cartesian_path([wp6_safety])
                
            except TransformException as e:
                self.get_logger().error(f"Failed to calculate drop: {e}")

        except Exception as e:
            self.get_logger().error(f"CRITICAL FAILURE: {e}")
        finally:
            self.move_to_joints(self.WP1_HOME_JOINTS)
            self.get_logger().info("✅ Mission Complete.")

    def control_gripper(self, position, wait=True):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 1000.0 
        send_future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted: return False
        if not wait: return True
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    def execute_cartesian_path(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = self.GROUP_NAME
        req.waypoints = waypoints
        req.max_step = 0.01
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.fraction > 0.9:
            goal = ExecuteTrajectory.Goal(trajectory=res.solution)
            send_goal_future = self._execute_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            if not send_goal_future.result().accepted: return False
            res_future = send_goal_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, res_future) 
            return True
        return False

    def quaternion_to_euler(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = UnifiedSmartPick()
    try:
        node.track_and_pick()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
