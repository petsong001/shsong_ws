#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import math

class MoveItObstacleManager(Node):
    def __init__(self):
        super().__init__('moveit_obstacle_manager')
        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/vision/hand_pose', self.vision_callback, 10)
        
        self.get_logger().info("🚧 Cleaning scene and waiting for vision...")
        self.object_id = "detected_cube"
        self.first_run = True
        
        self.padding = 0.10 
        self.base_size = 0.04
        
        self.last_pose = None
        self.update_threshold = 0.025 

    def add_safety_floor(self):
        floor = CollisionObject()
        # "base_link" is the standard root frame for your UR5e
        floor.header.frame_id = "base_link" 
        floor.id = "safety_floor"
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        # A massive 2m x 2m table, 2cm thick
        box.dimensions = [2.0, 2.0, 0.02] 
        
        floor_pose = Pose()
        # Center it directly under the robot
        floor_pose.position.x = 0.0
        floor_pose.position.y = 0.0
        # Place it 2cm below the robot's base to ensure the robot itself isn't in collision
        floor_pose.position.z = -0.02 
        floor_pose.orientation.w = 1.0
        
        floor.primitives.append(box)
        floor.primitive_poses.append(floor_pose)
        floor.operation = CollisionObject.ADD
        
        self.collision_pub.publish(floor)
        self.get_logger().info("🛬 Safety floor added to Planning Scene.")

    def vision_callback(self, msg: PoseStamped):
        # 1. First run setup: Spawn floor and clear old ghosts
        if self.first_run:
            self.add_safety_floor()
            
            clear_obj = CollisionObject()
            clear_obj.header.frame_id = msg.header.frame_id
            clear_obj.id = self.object_id
            clear_obj.operation = CollisionObject.REMOVE
            self.collision_pub.publish(clear_obj)
            self.first_run = False

        # 2. Check if the movement is just camera noise (Deadband filter)
        if self.last_pose is not None:
            dx = msg.pose.position.x - self.last_pose.position.x
            dy = msg.pose.position.y - self.last_pose.position.y
            dz = msg.pose.position.z - self.last_pose.position.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Ignore jitter under 2.5cm
            if distance < self.update_threshold:
                return
                
        self.last_pose = msg.pose

        # 3. Add the fresh padded cube
        collision_object = CollisionObject()
        collision_object.header.frame_id = msg.header.frame_id
        collision_object.id = self.object_id
        
        padded_dim = self.base_size + self.padding
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [padded_dim, padded_dim, padded_dim] 
        
        box_pose = Pose()
        box_pose.position.x = msg.pose.position.x
        box_pose.position.y = msg.pose.position.y
        box_pose.position.z = msg.pose.position.z + (padded_dim / 2.0)
        box_pose.orientation.w = 1.0
        
        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD
        
        self.collision_pub.publish(collision_object)
        self.get_logger().info(f"✅ Padded Cube Moved (Update sent to MoveIt)")

def main(args=None):
    rclpy.init(args=args)
    node = MoveItObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()