#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class SimpleMover(Node):
    def __init__(self):
        super().__init__('simple_mover_node')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.group_name = "ur_manipulator"  # CHECK THIS: Standard for UR robots. Might be "manipulator"
        self.end_effector_link = "tool0"    # Standard for UR5e

    def move_to_pose(self, x, y, z, qx, qy, qz, qw):
        """Sends a Cartesian goal (x, y, z) to MoveIt."""
        goal_msg = MoveGroup.Goal()
        
        # 1. Basic Setup
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5  # Move at 50% speed
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # 2. Define the Target Pose (Point B)
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw

        # 3. Create Constraints (Tell MoveIt: "End effector MUST be at this Pose")
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = self.end_effector_link
        pos_constraint.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01])] # 1cm tolerance
        pos_constraint.constraint_region.primitive_poses = [target_pose.pose]
        pos_constraint.weight = 1.0

        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = self.end_effector_link
        ori_constraint.orientation = target_pose.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.01
        ori_constraint.absolute_y_axis_tolerance = 0.01
        ori_constraint.absolute_z_axis_tolerance = 0.01
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(ori_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        # 4. Send Goal
        self.get_logger().info(f"Sending robot to: X={x}, Y={y}, Z={z}...")
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('SUCCESS: Arrived at Point B!')
        else:
            self.get_logger().error(f'FAILURE: Error Code {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    mover = SimpleMover()

    # --- INPUT YOUR POINT B HERE ---
    # Coordinates are in 'base_link' (Meters)
    # This example point is usually safe "in front and above" the robot base.
    target_x = 0.0
    target_y = 0.1
    target_z = 0.4
    
    # Orientation (Quaternions) - This usually points the gripper DOWN
    target_qx = 0.0
    target_qy = 1.0
    target_qz = 0.0
    target_qw = 0.0

    mover.move_to_pose(target_x, target_y, target_z, target_qx, target_qy, target_qz, target_qw)
    rclpy.spin(mover)

if __name__ == '__main__':
    main()
