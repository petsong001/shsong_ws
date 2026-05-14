#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

class QoSBridge(Node):
    def __init__(self):
        super().__init__('qos_bridge')
        
        # 1. Listen to Isaac Sim (Best Effort)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(
            JointState, 
            '/isaac_joint_states', 
            self.callback, 
            qos_sub
        )

        # 2. Publish RELIABLE data
        self.pub = self.create_publisher(
            JointState, 
            '/isaac_joint_states_reliable', 
            10
        )
        self.get_logger().info('QoS Bridge Running: Transparent Forwarding')

    def callback(self, msg):
        # FIX: Do NOT overwrite the timestamp. 
        # Pass the simulation time from Isaac directly to MoveIt.
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
