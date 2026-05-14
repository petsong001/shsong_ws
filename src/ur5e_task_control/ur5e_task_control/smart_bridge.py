#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
import time

try:
    from control_msgs.msg import JointTrajectoryControllerState
except ImportError:
    print("❌ ERROR: Missing 'control_msgs'. Run: sudo apt install ros-humble-control-msgs")
    exit()

class SmartBridge(Node):
    def __init__(self):
        super().__init__('smart_bridge')
        
        # --- CONFIGURATION ---
        # 1. We pad with 2 zeros (one for each finger)
        self.GRIPPER_ZEROS = [0.0, 0.0] 
        
        # 2. EXACT NAMES from your ros2 topic echo output
        self.GRIPPER_NAMES = ["robotiq_hande_left_finger_joint", "robotiq_hande_right_finger_joint"]
        
        # 3. THROTTLING (30Hz) to prevent lag
        self.TARGET_PUBLISH_RATE = 30.0 
        self.min_period = 1.0 / self.TARGET_PUBLISH_RATE
        self.last_pub_time = 0.0

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # --- THE FIX: Point to the 'scaled' controller ---
        self.target_topic = '/scaled_joint_trajectory_controller/controller_state'
        
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            self.target_topic,
            self.listener_callback,
            qos_profile
        )
            
        # Speak to Isaac Sim
        self.publisher = self.create_publisher(
            JointState,
            '/joint_commands', 
            10
        )
        
        self.has_received = False
        self.get_logger().info(f"✅ BRIDGE ONLINE: Padding {len(self.GRIPPER_ZEROS)} joints (robotiq_hande)...")

    def listener_callback(self, msg):
        current_time = time.time()
        
        # Throttling Logic
        if (current_time - self.last_pub_time) < self.min_period:
            return
        
        self.last_pub_time = current_time

        if not self.has_received:
            self.get_logger().info(f"📩 CONNECTED! Forwarding Arm + Gripper data to Isaac Sim.")
            self.has_received = True

        command_msg = JointState()
        command_msg.header.stamp = self.get_clock().now().to_msg()
        
        # MERGE ARM + GRIPPER
        command_msg.name = list(msg.joint_names) + self.GRIPPER_NAMES
        command_msg.position = list(msg.desired.positions) + self.GRIPPER_ZEROS
        
        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = SmartBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()