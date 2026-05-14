#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

# --- Calibration Parameters (MUST MATCH YOUR BOARD) ---
# Values confirmed from your RViz setup screenshots:
SQUARES_X = 5
SQUARES_Y = 7
SQUARE_SIZE_M = 0.027  # Size of a square side (0.027m)
MARKER_SIZE_M = 0.017  # Size of a marker side (0.017m)
ARUCO_DICT = cv2.aruco.DICT_5X5_250

class CharucoDetector(Node):
    def __init__(self):
        super().__init__('charuco_detector_node')
        self.br = CvBridge()
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Create Charuco detector objects
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.board = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_SIZE_M, MARKER_SIZE_M, self.aruco_dict)
        self.detector = cv2.aruco.CharucoDetector(self.board)

        # Camera Intrinsics placeholder (will be filled by CameraInfo subscription)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Subscriptions: Note: Topics assume standard ZED D-series node naming
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.get_logger().info("Charuco Detector Node initialized. Waiting for camera info...")

    def camera_info_callback(self, msg):
        # Fill in the Camera Intrinsics using the data published by the ZED driver
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        
        # Unsubscribe after receiving info, as intrinsics don't change
        self.destroy_subscription(self.info_sub)
        self.get_logger().info("Camera Intrinsics received. Starting detection.")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            
            # --- Detection Logic ---
            # Detect markers and then refine to find the ChArUco corners
            # OpenCV's detect_board handles both marker detection and corner interpolation
            charuco_corners, charuco_ids, marker_corners, marker_ids = self.detector.detect_board(gray_frame)

            if charuco_ids is not None and len(charuco_ids) > 0:
                # --- Pose Estimation ---
                # Calculate the rotation and translation vector (rvec, tvec)
                # This pose is T_camera^object
                success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                    charuco_corners, charuco_ids, self.board, self.camera_matrix, self.dist_coeffs, None, None
                )

                if success:
                    # Draw pose for visualization (optional, but good for debugging) 
                    cv2.drawFrameAxes(current_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
                    
                    # --- Publish TF Transform ---
                    self.publish_object_tf(rvec, tvec, msg.header.stamp)

            # Display the result (Optional: uncomment to see detection live)
            # cv2.imshow("Charuco Detection", current_frame)
            # cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def publish_object_tf(self, rvec, tvec, stamp):
        t = TransformStamped()
        
        # Header Info
        t.header.stamp = stamp
        t.header.frame_id = 'zed_left_camera_optical_frame'  # Parent: Camera frame
        t.child_frame_id = 'detected_object'                 # Child: Board origin (T_camera^object)

        # Translation (m)
        t.transform.translation.x = tvec[0][0]
        t.transform.translation.y = tvec[1][0]
        t.transform.translation.z = tvec[2][0]

        # Rotation (Convert rvec Axis-Angle to Quaternion for TF)
        r = R.from_rotvec([rvec[0][0], rvec[1][0], rvec[2][0]])
        quat = r.as_quat()

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    detector = CharucoDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
