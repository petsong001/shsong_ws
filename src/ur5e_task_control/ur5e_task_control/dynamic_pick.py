#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import CollisionObject, JointConstraint, Constraints
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose, Point, Quaternion
from control_msgs.action import GripperCommand 
from tf2_ros import TransformException, Buffer, TransformListener

class SmartPick(Node):
    def __init__(self):
        super().__init__('smart_pick')
        
        self.target_sub = self.create_subscription(Point, '/vision/cube_coordinates', self.vision_callback, 10)
        
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._gripper_client = ActionClient(self, GripperCommand, '/hande_gripper_controller/gripper_cmd')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- CONFIGURATION ---
        self.GROUP_NAME = 'ur_manipulator'
        self.APPROACH_Z = 0.25            
        self.GRASP_Z = 0.015         # 1.5cm Grasp Height
        self.GRIPPER_OFFSET = 0.155
        
        # 1. HOME POSITION
        self.HOME_JOINTS = [1.3162, -1.2358, 1.3143, -1.5492, -1.5683, -0.0183]

        # 2. DROP LOCATION
        self.DROP_JOINTS = [1.714, -0.8263, 1.3858, -2.0408, -1.5293, 0.3894]

        self.received_target = None
        
        time.sleep(1.0)
        self.get_logger().info("✅ Ready. Waiting for target...")

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except TransformException as ex:
            self.get_logger().error(f'Could not get transform: {ex}')
            return None

    def verify_height(self, max_allowed_height=0.195):
        pose = self.get_current_pose()
        if pose is None: return False
        current_z = pose.position.z
        self.get_logger().info(f"📏 Height Check: Current Z = {current_z:.3f}m")
        if current_z > max_allowed_height:
            self.get_logger().error(f"❌ TOO HIGH! (Is {current_z:.3f}m, needs < {max_allowed_height}m)")
            return False
        return True

    def vision_callback(self, msg):
        if self.received_target is None:
            fixed_target = Point()
            fixed_target.x = msg.x 
            fixed_target.y = msg.y 
            fixed_target.z = msg.z
            self.received_target = fixed_target
            self.get_logger().info(f"👁️ Target Locked: X:{msg.x:.3f} Y:{msg.y:.3f}")

    def control_gripper(self, position, force=500.0):
        self.get_logger().info(f"🖐️ Gripper Action: Moving to {position}...")
        if not self._gripper_client.wait_for_server(timeout_sec=5.0): 
            self.get_logger().error("❌ Gripper Server not available!")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = force
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Gripper Goal Rejected!")
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info("⏳ Waiting for fingers to physically close...")
        time.sleep(1.5) 
        self.get_logger().info("✅ Gripper Action Complete.")
        return True

    def move_to_joints(self, joint_values, description="Joint Move"):
        self.get_logger().info(f"🤖 {description}...")
        if not self._move_group_client.wait_for_server(timeout_sec=2.0): return False
        goal = MoveGroup.Goal()
        goal.request.group_name = self.GROUP_NAME
        goal.request.max_velocity_scaling_factor = 1.0 
        goal.request.max_acceleration_scaling_factor = 1.0
        c = Constraints()
        c.name = "JointGoal"
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for i, val in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        goal.request.goal_constraints.append(c)
        send_goal_future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted: return False
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        return True

    def execute_cartesian_path(self, waypoints, description="Linear Move"):
        self.get_logger().info(f"📏 {description}")
        if not self._cartesian_client.wait_for_service(timeout_sec=1.0): return False
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.GROUP_NAME
        req.waypoints = waypoints
        req.max_step = 0.01 
        req.jump_threshold = 0.0 
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.fraction < 0.95: 
            self.get_logger().warn(f"⚠️ Path Incomplete! (Only {response.fraction:.2f} computed)")
            return False
        exec_goal = ExecuteTrajectory.Goal()
        exec_goal.trajectory = response.solution
        self._execute_client.wait_for_server()
        send_goal_future = self._execute_client.send_goal_async(exec_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted: return False
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        final_result = get_result_future.result()
        if final_result.result.error_code.val != 1: 
             self.get_logger().error(f"❌ Movement Failed Error: {final_result.result.error_code.val}")
             return False
        return True

    def run_sequence(self):
        # --- INITIALIZATION ---
        self.move_to_joints(self.HOME_JOINTS, "WP1: Home Start")
        self.control_gripper(position=0.05) 
        time.sleep(1.0) 

        self.get_logger().info("👀 Waiting for Vision Target...")
        while rclpy.ok() and self.received_target is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.received_target:
            t = self.received_target
            pose = self.get_current_pose()
            current_orientation = pose.orientation
            self.get_logger().info(f"🔒 Locked Orientation: {current_orientation.w:.2f}")

            # =========================================================
            #              DEFINE WAYPOINTS (WP)
            # =========================================================
            
            # WP1: Home (Already there)
            
            # WP2: HOVER (Above target)
            wp2_hover = Pose()
            wp2_hover.position.x = t.x
            wp2_hover.position.y = t.y
            wp2_hover.position.z = self.APPROACH_Z 
            wp2_hover.orientation = current_orientation 

            # WP3: DIVE (Grasp Position)
            wp3_dive = Pose()
            wp3_dive.position.x = t.x
            wp3_dive.position.y = t.y
            wp3_dive.position.z = self.GRASP_Z + self.GRIPPER_OFFSET 
            wp3_dive.orientation = current_orientation

            # WP4: LIFT (Same as Hover)
            wp4_lift = wp2_hover

            # WP5: DIVE AGAIN (Revisit Center)
            wp5_revisit = wp3_dive 

            # WP6: HOME (Return)
            wp6_home = self.HOME_JOINTS

            # WP7: DROP ZONE
            wp7_drop = self.DROP_JOINTS
            
            # WP8: CENTER AGAIN (Return to Hover)
            wp8_center = wp2_hover

            # =========================================================
            #              EXECUTE SEQUENCE
            # =========================================================

            # --- MOVE TO WP2 (Hover) ---
            if not self.execute_cartesian_path([wp2_hover], "WP2: Hover"): return
            time.sleep(0.5)

            # --- MOVE TO WP3 (Dive) ---
            self.execute_cartesian_path([wp3_dive], "WP3: Dive")
            self.get_logger().info("🛑 Stabilizing...")
            time.sleep(1.0) 

            # --- ACTION: GRASP ---
            if self.verify_height(max_allowed_height=0.195):
                self.get_logger().info("✅ Grasping...")
                self.control_gripper(position=0.0, force=1000.0)
                
                # --- MOVE TO WP4 (Lift) ---
                self.execute_cartesian_path([wp4_lift], "WP4: Lift")
                
                # --- MOVE TO WP5 (Dive Again) ---
                self.get_logger().info("🔄 Re-Diving to Center...")
                self.execute_cartesian_path([wp5_revisit], "WP5: Dive Again")
                time.sleep(0.5)
                self.execute_cartesian_path([wp4_lift], "Safety Lift") 

                # --- MOVE TO WP6 (Home) ---
                self.move_to_joints(wp6_home, "WP6: Go Home")
                
                # --- MOVE TO WP7 (Drop Zone) ---
                self.move_to_joints(wp7_drop, "WP7: Go to Drop Zone")
                
                # --- ACTION: DROP ---
                self.get_logger().info("🔓 WP7: Dropping Object...")
                self.control_gripper(position=0.05) 
                time.sleep(1.0) 
                
                # --- MOVE TO WP8 (Center Again) ---
                self.get_logger().info("🔄 WP8: Returning to Center...")
                self.move_to_joints(wp6_home, "WP8: Move to Home (Safe)") # Move to Joint Home first
                self.execute_cartesian_path([wp8_center], "WP8: Slide to Center Hover") # Then Slide

                self.get_logger().info("🛑 MISSION COMPLETE.")
            else:
                self.get_logger().error("❌ SAFETY ABORT: Height check failed.")

def main(args=None):
    rclpy.init(args=args)
    node = SmartPick()
    node.run_sequence()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
