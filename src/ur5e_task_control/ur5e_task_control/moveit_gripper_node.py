import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import GripperCommand
import time

class MoveitGripperNode(Node):
    def __init__(self):
        super().__init__('hande_gripper_controller_node') # Name matching the controller
        self.get_logger().info("Hand-E Gripper Action Interface Node Initialized.")
        
        # 1. ACTION SERVER: This server listens for GripperCommand goals from MoveIt
        self._action_server = ActionServer(
            self,
            GripperCommand,
            'hande_gripper_controller/gripper_cmd', # Topic where MoveIt publishes goals
            self.execute_callback
        )
        self.get_logger().info('Action server ready on hande_gripper_controller/gripper_cmd.')
        
    def execute_callback(self, goal_handle):
        """Processes a GripperCommand goal received from MoveIt."""
        
        target_position = goal_handle.request.command.position
        
        self.get_logger().info(f'--- Received Gripper Command ---')
        
        if target_position > 0.03:
            self.get_logger().info(f'Simulating Gripper OPEN to {target_position:.4f} m...')
        else:
            self.get_logger().info(f'Simulating Gripper CLOSE to {target_position:.4f} m...')
        
        # NOTE: In a real system, you would insert the Hand-E driver communication here.
        
        # Simulate movement time
        time.sleep(1.0) 
        
        # Set goal to succeeded and publish the result back to MoveIt
        goal_handle.succeed()
        
        result = GripperCommand.Result()
        result.position = target_position
        result.stalled = False
        result.reached_goal = True
        
        self.get_logger().info('Gripper operation simulated successfully.')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MoveitGripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
