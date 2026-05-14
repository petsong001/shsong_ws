#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import time
import minimalmodbus
import serial
import threading

# --- CONFIGURATION ---
# Matches your working setup
PORT = '/tmp/ttyTool'
SLAVE_ADDRESS = 9
BAUD = 115200

# Registers
ACTION_REGISTER = 0x03E8
STATUS_REGISTER = 0x07D0
ACTIVATE_VALUE = 0x0100
RESET_VALUE = 0x0000

class GripperActionNode(Node):
    def __init__(self):
        super().__init__("hande_gripper_driver")
        self.get_logger().info("Initializing Hand-E Action Driver...")

        # 1. Setup Connection (Your working logic)
        try:
            self.instrument = minimalmodbus.Instrument(PORT, SLAVE_ADDRESS)
            self.instrument.serial.baudrate = BAUD
            self.instrument.serial.bytesize = 8
            self.instrument.serial.parity = serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout = 0.5
            self.instrument.clear_buffers_before_each_transaction = True
            self.get_logger().info(f"Connected to {PORT}")
        except Exception as e:
            self.get_logger().fatal(f"Connection Failed: {e}")
            return

        # 2. Auto-Activate on Startup
        self.activate_gripper()

        # 3. Create Action Server for MoveIt
        # Topic must match what MoveIt expects (check moveit_controllers.yaml)
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/hande_gripper_controller/gripper_cmd',
            self.execute_callback
        )

        # 4. Publisher for Fake Joint States
        # MoveIt needs to know where the finger is
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_state)
        self.current_pos = 0.0

    def activate_gripper(self):
        self.get_logger().info("Resetting...")
        try:
            self.instrument.write_register(ACTION_REGISTER, RESET_VALUE, functioncode=16)
            time.sleep(0.5)
            self.instrument.write_register(ACTION_REGISTER, ACTIVATE_VALUE, functioncode=16)
            time.sleep(1.0)
            self.get_logger().info("Gripper Activated!")
        except Exception as e:
            self.get_logger().error(f"Activation failed: {e}")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received MoveIt Goal...')
        target_pos = goal_handle.request.command.position
        
        # Logic: 0.0 = Open, >0.0 = Close
        # Hand-E mapping: 0 (Open) -> 255 (Close)
        if target_pos > 0.02: # Closing
            # Action Request(09) + Position(FF) + Speed(FF) + Force(FF)
            # 0x09FF = GoTo + Position 255
            # We write to register 0x03E8 (Action Request) and 0x03E9 (Pos)
            # minimalmodbus handles registers easily
            
            # Write 3 registers starting at ACTION_REGISTER
            # Reg 1: Action (0900)
            # Reg 2: Pos (00FF) -> 00 is reserved byte? No, check manual.
            # Manual says: 
            # Byte 0: Action Request (09)
            # Byte 1: Reserved (00)
            # Byte 2: Reserved (00)
            # Byte 3: Position Request
            
            # Let's use the write_registers method carefully matching your byte logic
            # Your code: reg0=(09<<8)|00, reg1=(00<<8)|Pos, reg2=(Spd<<8)|Frc
            
            reg0 = 0x0900
            reg1 = 0x00FF # Close (255)
            reg2 = 0x8080 # Default Speed/Force
            
            self.current_pos = 0.05 # Feedback as closed
            self.get_logger().info("Closing...")
        else:
            # Opening
            reg0 = 0x0900
            reg1 = 0x0000 # Open (0)
            reg2 = 0x8080
            
            self.current_pos = 0.0 # Feedback as open
            self.get_logger().info("Opening...")

        try:
            self.instrument.write_registers(ACTION_REGISTER, [reg0, reg1, reg2])
            time.sleep(1.0) # Wait for move
            goal_handle.succeed()
            result = GripperCommand.Result()
            result.position = self.current_pos
            result.reached_goal = True
            return result
        except Exception as e:
            self.get_logger().error(f"Move failed: {e}")
            goal_handle.abort()
            return GripperCommand.Result()

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['hande_left_finger_joint']
        msg.position = [self.current_pos]
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
