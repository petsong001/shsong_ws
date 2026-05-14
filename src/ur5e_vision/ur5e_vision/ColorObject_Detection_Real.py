#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import math
import pyzed.sl as sl
from geometry_msgs.msg import PointStamped, Pose
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from collections import deque 

class ZedLocator(Node):
    def __init__(self):
        super().__init__('zed_cube_locator')
        self.coord_pub = self.create_publisher(Pose, '/vision/cube_coordinates', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.zed = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.depth_mode = sl.DEPTH_MODE.NEURAL 
        self.init_params.coordinate_units = sl.UNIT.METER
        
        err = self.zed.open(self.init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            print(f"❌ Failed to open ZED: {err}")
            exit(1)
        
        self.runtime_params = sl.RuntimeParameters()
        self.point_cloud = sl.Mat()
        self.image_sl = sl.Mat()

        self.history_len = 10
        self.angle_history = deque(maxlen=self.history_len) # Stores (sin, cos)
        self.x_history = deque(maxlen=self.history_len)
        self.y_history = deque(maxlen=self.history_len)

        self.get_logger().info("✅ ZED Detection Node Started (Inverted Direction Fix)")

    def run(self):
        while rclpy.ok():
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                self.zed.retrieve_image(self.image_sl, sl.VIEW.LEFT)
                image_ocv = cv2.cvtColor(self.image_sl.get_data(), cv2.COLOR_BGRA2BGR)
                self.zed.retrieve_measure(self.point_cloud, sl.MEASURE.XYZRGBA)
                self.process_image(image_ocv)
            rclpy.spin_once(self, timeout_sec=0.001)

    def process_image(self, cv_image):
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([35, 70, 30]), np.array([90, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            cnt = max(contours, key=cv2.contourArea)
            if 300 < cv2.contourArea(cnt) < 12000:
                M = cv2.moments(cnt)
                if M["m00"] == 0: return
                cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
                
                rect = cv2.minAreaRect(cnt)
                (bx, by), (w, h), raw_angle = rect
                
                # 1. Normalize angle to [-45, 45]
                angle = (raw_angle + 180 if w < h else raw_angle + 90)
                angle = (angle + 45) % 90 - 45
                
                # 2. INVERSION FIX: Flip the sign so CCW rotation is Positive (+)
                # This ensures vision matches the Robot Base Link coordinate system
                angle = -angle
                
                # 3. Vector Smoothing (Sin/Cos averaging)
                rad = math.radians(angle)
                self.angle_history.append((math.sin(rad), math.cos(rad)))
                avg_sin = sum(s for s, c in self.angle_history) / len(self.angle_history)
                avg_cos = sum(c for s, c in self.angle_history) / len(self.angle_history)
                avg_angle_rad = math.atan2(avg_sin, avg_cos)

                # Draw for debug
                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
                cv2.putText(cv_image, f"A: {int(math.degrees(avg_angle_rad))}", (cx, cy-20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

                err, point3D = self.point_cloud.get_value(cx, cy)
                if err == sl.ERROR_CODE.SUCCESS:
                    val_x, val_y, val_z = point3D[0], point3D[1], point3D[2]
                    if np.isfinite(val_x) and np.isfinite(val_z):
                        self.x_history.append(val_x)
                        self.y_history.append(val_y)
                        self.transform_and_publish(np.mean(self.x_history), np.mean(self.y_history), val_z, avg_angle_rad)

        cv2.imshow("ZED Debug", cv_image)
        cv2.waitKey(1)

    def transform_and_publish(self, x, y, z, angle_rad):
        try:
            p_cam = PointStamped()
            p_cam.header.frame_id = "zed_left_camera_frame_optical"
            p_cam.header.stamp = self.get_clock().now().to_msg()
            p_cam.point.x, p_cam.point.y, p_cam.point.z = float(x), float(y), float(z)
            
            # Lookup transform from camera to robot base
            transform = self.tf_buffer.lookup_transform('base_link', p_cam.header.frame_id, rclpy.time.Time())
            p_base = do_transform_point(p_cam, transform)

            # Build quaternion: Roll=PI (down), Pitch=0, Yaw=Detected Angle
            q = self.euler_to_quaternion(3.14159, 0, angle_rad)
            
            msg = Pose()
            msg.position = p_base.point
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = q
            self.coord_pub.publish(msg)
        except Exception as e:
            pass

    def euler_to_quaternion(self, r, p, y):
        cr, cp, cy = math.cos(r/2), math.cos(p/2), math.cos(y/2)
        sr, sp, sy = math.sin(r/2), math.sin(p/2), math.sin(y/2)
        return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]

def main():
    rclpy.init()
    node = ZedLocator()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.zed.close()
        rclpy.shutdown()

if __name__ == '__main__': main()
