#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
# CORRECTED IMPORT:
from shape_msgs.msg import SolidPrimitive
import copy
import time

# TF2 imports
from tf2_ros import Buffer, TransformListener

class MoveItMover(Node):
    def __init__(self):
        super().__init__('moveit_mover')
        self.group_name = "ur_manipulator"
        
        # Action Client for Standard Planning (Smart Planning)
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('MoveIt Mover Ready!')

    def get_current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            self.get_logger().info(f"Start Pose: z={p.position.z:.2f}")
            return p
        except Exception as e:
            return None

    def move_smart(self, target_pose):
        """Uses the standard planner to find ANY valid path (curves allowed)"""
        
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Define Constraints
        constraints = Constraints()
        
        # 1. Position Constraint (Target XYZ)
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        
        # Define the target region as a tiny sphere (5mm)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.005] 
        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(target_pose)
        
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        
        # 2. Orientation Constraint (Target Rotation)
        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.1 
        oc.absolute_y_axis_tolerance = 0.1 
        oc.absolute_z_axis_tolerance = 0.1 
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        
        goal_msg.request.goal_constraints.append(constraints)
        
        self.get_logger().info("Planning path...")
        
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Plan Rejected!")
            return

        self.get_logger().info("Plan Accepted. Moving...")
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        self.get_logger().info("Movement Finished!")

def main(args=None):
    rclpy.init(args=args)
    mover = MoveItMover()

    # 1. Get Current Position
    print("Waiting for TF...")
    current_pose = None
    for i in range(10):
        rclpy.spin_once(mover, timeout_sec=0.1)
        current_pose = mover.get_current_pose()
        if current_pose: break
        time.sleep(0.2)
        
    if not current_pose:
        print("Could not find robot pose. Is the driver running?")
        return

    # 2. Define Target
    target_pose = copy.deepcopy(current_pose)
    target_pose.position.x = -0.6
    target_pose.position.y = -0.50
    target_pose.position.z = 0.2

    # IMPORTANT: Force Orientation to point SIDEWAYS/DOWN (not up)
    # This helps break the singularity from the candle position
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 1.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0

    # 3. Move
    mover.move_smart(target_pose)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
