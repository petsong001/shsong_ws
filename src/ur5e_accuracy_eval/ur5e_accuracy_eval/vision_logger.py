import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener

class VisionLogger(Node):
    def __init__(self):
        super().__init__('vision_logger')
        
        # 1. Setup TF2 for Robot Ground Truth
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 2. Setup Synchronized Subscribers for RGB and Depth
        self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/aligned_depth_to_color/image_raw')
        
        # This ensures the callback only fires when frames have matching timestamps
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.02)
        self.ts.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, rgb_msg, depth_msg):
        # MediaPipe logic goes here using rgb_msg
        # Depth lookup goes here using depth_msg
        
        # Look up where the robot thinks the hand is AT THIS EXACT TIME
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('camera_link', 'hand_base_link', now)
            # Ground Truth = transform.transform.translation
        except Exception as e:
            self.get_logger().warn(f'Could not find robot transform: {e}')