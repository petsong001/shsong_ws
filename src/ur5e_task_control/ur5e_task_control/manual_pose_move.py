#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from control_msgs.action import GripperCommand
import time

class FinalRobotClient(Node):
    def __init__(self):
        super().__init__('final_robot_client')
        
        # Arm Action Client
        self._arm_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Gripper Action Client
        self._gripper_client = ActionClient(
            self, 
            GripperCommand, 
            '/hande_gripper_controller/gripper_cmd'
        )

    def move_arm(self, joint_values, speed=0.1):
        """Moves the UR5e arm to specific joint angles using MoveIt."""
        if not self._arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm MoveGroup server not found!")
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        goal_msg.request.max_velocity_scaling_factor = speed
        goal_msg.request.max_acceleration_scaling_factor = speed

        joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name, jc.position = name, value
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._arm_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        handle = future.result()
        if not handle.accepted: return False
        
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result().result.error_code.val == 1

    def control_gripper(self, position):
        """
        Synchronous Gripper Control: Waits for the server to accept the goal.
        This fixes the lag/bug by ensuring the command is sent before moving on.
        """
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper Action Server not found!")
            return False

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position 
        goal_msg.command.max_effort = 130.0   
        
        self.get_logger().info(f"Sending gripper goal: {position}m")
        
        # Send goal and WAIT for acknowledgement
        send_goal_future = self._gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal REJECTED")
            return False

        self.get_logger().info("Gripper goal ACCEPTED")
        return True

def main(args=None):
    rclpy.init(args=args)
    client = FinalRobotClient()

    # --- WAYPOINT DATA (Captured from Joint States) ---
    home_pose = [0.0, -1.571, 0.0, -1.571, 0.0, 0.0]
    wp1 = [-1.487, -1.569, 0.000, -1.569, 1.619, 0.001]
    wp2 = [-1.027, -1.533, -0.906, -1.904, 1.595, 0.305]
    wp3 = [-1.020, -1.896, -1.905, -0.877, 1.604, 0.261] # PICK POSITION
    wp4 = [-1.286, -1.654, -1.486, -1.562, 1.597, 0.257] # LIFT POSITION
    wp5 = [-1.519, -2.146, -1.698, -0.857, 1.549, 0.065] # PLACE POSITION

    # --- EXECUTION SEQUENCE ---

    # 1. Start with Gripper Open
    print("\n--- Step 1: Opening Gripper ---")
    client.control_gripper(0.0)
    time.sleep(1.0)

    # 2. Move to Pick Location
    print("\n--- Step 2: Approaching Pick (WP1 -> WP2 -> WP3) ---")
    client.move_arm(wp1)
    client.move_arm(wp2)
    client.move_arm(wp3)

    # 3. Close Gripper at WP3
    print("\n--- Step 3: Closing Gripper ---")
    client.control_gripper(0.045) 
    time.sleep(2.0) # Wait for physical close on object

    # 4. Move to Place Location
    print("\n--- Step 4: Moving to Place (WP4 -> WP5) ---")
    client.move_arm(wp4)
    client.move_arm(wp5)

    # 5. Open Gripper at WP5
    print("\n--- Step 5: Releasing Object ---")
    client.control_gripper(0.0)
    time.sleep(1.0)

    # 6. Return to Home
    print("\n--- Step 6: Returning Home ---")
    client.move_arm(home_pose)

    print("\nDemo Complete.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
