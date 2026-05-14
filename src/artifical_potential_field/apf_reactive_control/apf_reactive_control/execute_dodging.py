#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import sys
import time

class DodgeSequenceClient(Node):
    def __init__(self):
        super().__init__('dodge_sequence_client')
        # Connecting to the MoveGroup action server
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info('🚧 Connecting to MoveGroup...')
        self._action_client.wait_for_server()

        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        ]

        # Waypoints for RRT testing
        self.waypoints = [
            [-0.50, -1.22, 2.16, -2.51, -1.50, 0.05], 
            [1.25, -1.22, 2.01, -2.43, -1.55, 1.13]   
        ]
        self.current_wp_index = 0

    def send_next_waypoint(self):
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('🏁 SUCCESS: Robot successfully completed the sequence.')
            sys.exit()

        joints = self.waypoints[self.current_wp_index]
        self.get_logger().info(f'🚀 Moving to Waypoint {self.current_wp_index + 1}...')
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        
        # --- PLANNING & EXECUTION SETTINGS ---
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False
        
        # Use diff to ignore state differences between planning and execution start
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        
        # High attempt count to find a valid RRT path around the padded cube
        goal_msg.request.planner_id = "" 
        goal_msg.request.num_planning_attempts = 50
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.start_state.is_diff = True
        
        # Slow and steady for safety during hardware/sim tests
        goal_msg.request.max_velocity_scaling_factor = 0.10
        goal_msg.request.max_acceleration_scaling_factor = 0.10
        
        constraints = Constraints()
        for i, val in enumerate(joints):
            jc = JointConstraint(joint_name=self.joint_names[i], position=val, weight=1.0)
            jc.tolerance_above = 0.1
            jc.tolerance_below = 0.1
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveGroup!')
            return 
        
        self.get_logger().info('✅ Planning successful. Executing...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        
        # Code 1: SUCCESS
        if result.error_code.val == 1:
            self.get_logger().info(f'📍 Reached Waypoint {self.current_wp_index + 1}!')
            self.current_wp_index += 1
            if self.current_wp_index < len(self.waypoints):
                time.sleep(2.0)
            self.send_next_waypoint()

        # Code -2: INVALID_MOTION_PLAN (Smoothing clipped the cube)
        # Code -3: MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE (Scene jitter)
        elif result.error_code.val in [-2, -3]:
            self.get_logger().warn(f'⚠️ Path error ({result.error_code.val}). RRT is rolling again...')
            # Trigger a retry for the SAME waypoint
            self.send_next_waypoint()

        else:
            self.get_logger().error(f'❌ Hard Failure. Error code: {result.error_code.val}')
            sys.exit()

def main(args=None):
    rclpy.init(args=args)
    node = DodgeSequenceClient()
    node.send_next_waypoint()
    rclpy.spin(node)

if __name__ == '__main__':
    main()