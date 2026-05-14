#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from controller_manager_msgs.srv import SwitchController
import numpy as np
from scipy.spatial.transform import Rotation as R
import json
import os

class WholeArmAPFNode(Node):
    def __init__(self):
        super().__init__('whole_arm_apf_task')
        
        self.state = "INIT"
        self.tf_warming_up = True 
        self.startup_wait_time = None

        self.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        self.link_names = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link", "wrist_3_link", "tool0"]
        
        # Waypoints for the task
        self.wp1_joints = np.deg2rad([-181.0, -119.0, -87.0, -64.0, 90.0, 0.0]) 
        self.wp2_joints = np.deg2rad([-90.0, -119.0, -87.0, -64.0, 90.0, 0.0])  
        self.waypoints = [self.wp1_joints, self.wp2_joints]
        self.current_wp_idx = 0
        
        # --- OPTIMIZABLE PARAMETERS (Declared for Launch Argument Overrides) ---
        self.declare_parameter('alpha_att', 11.30)
        self.declare_parameter('ki', 1.02)      
        self.declare_parameter('kd', 1.75)      
        self.declare_parameter('alpha_rep', 25.0)
        self.declare_parameter('zeta', 0.65)
        
        self.alpha_att = self.get_parameter('alpha_att').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.alpha_rep = self.get_parameter('alpha_rep').value
        self.zeta = self.get_parameter('zeta').value
        
        # --- STATIC TUNING PARAMETERS ---
        self.rho_0 = 0.30             # Repulsion Influence Distance (30cm)
        self.rho_floor = 0.12         # Floor safety buffer (12cm)
        self.alpha_rep_floor = 15.0   # Floor repulsion gain
        self.max_joint_speed = 3.14   # Rad/s limit

        # --- STATE TRACKING ---
        self.start_time = None
        self.min_dist_observed = 100.0 
        self.current_joint_pos = np.zeros(6)
        self.integral_error = np.zeros(6) 
        self.last_joint_error = np.zeros(6)
        
        # --- JITTER/COST TRACKING ---
        self.last_velocities = np.zeros(6)
        self.total_jitter_score = 0.0
        
        # =================================================================
        # STATIC OBSTACLE POSITION
        # =================================================================
        self.obstacle_position = np.array([0.6, 0.3, 0.05]) 
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.servo_start_cli = self.create_client(Trigger, '/servo_node/start_servo')
        self.switch_ctrl_cli = self.create_client(SwitchController, '/controller_manager/switch_controller')
        self.servo_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        self.timer = self.create_timer(0.02, self.control_loop) 
        self.get_logger().info(f'🚀 Static APF Node Active | Obstacle at: {self.obstacle_position}')
        self.engage_streaming_controller()

    def joint_callback(self, msg):
        temp_pos = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                temp_pos.append(msg.position[idx])
        if len(temp_pos) == 6:
            self.current_joint_pos = np.array(temp_pos)

    def engage_streaming_controller(self):
        if not self.switch_ctrl_cli.wait_for_service(timeout_sec=10.0):
            self.save_results_and_exit(success=False, error="Controller manager not ready")
            return
        req = SwitchController.Request()
        req.activate_controllers = ['apf_servo_controller']
        req.deactivate_controllers = ['scaled_joint_trajectory_controller', 'forward_position_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT 
        self.switch_ctrl_cli.call_async(req).add_done_callback(self.switch_response_callback)

    def switch_response_callback(self, future):
        if future.result().ok:
            if not self.servo_start_cli.wait_for_service(timeout_sec=10.0):
                self.save_results_and_exit(success=False, error="Servo node not ready")
                return
            self.servo_start_cli.call_async(Trigger.Request()).add_done_callback(self.servo_ready_callback)

    def servo_ready_callback(self, future):
        if future.result().success:
            self.state = "RUNNING_APF"
            self.start_time = self.get_clock().now()
        else:
            self.save_results_and_exit(success=False, error="Servo start failed")

    def get_transform(self, link_name):
        try: return self.tf_buffer.lookup_transform('base_link', link_name, rclpy.time.Time())
        except: return None

    def compute_geometric_jacobian(self, target_link_idx, link_positions):
        J_v = np.zeros((3, 6))
        target_pos = link_positions[target_link_idx]
        for i in range(6):
            if i <= target_link_idx:
                t = self.get_transform(self.link_names[i])
                if t is None: continue
                q = t.transform.rotation
                rot = R.from_quat([q.x, q.y, q.z, q.w])
                z_axis_world = rot.apply([0, 0, 1]) 
                joint_pos = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
                J_v[:, i] = np.cross(z_axis_world, (target_pos - joint_pos))
        return J_v

    def save_results_and_exit(self, success, error=""):
        total_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9 if self.start_time else 60.0
        data = {
            "time": float(total_time),
            "min_dist": float(self.min_dist_observed),
            "jitter": float(self.total_jitter_score), 
            "success": success,
            "error_diag": error
        }
        with open('/tmp/sim_results.json', 'w') as f:
            json.dump(data, f)
        raise SystemExit 

    def control_loop(self):
        if self.state != "RUNNING_APF": return
        now = self.get_clock().now()
        
        # TF Warmup Logic
        if self.tf_warming_up:
            if self.startup_wait_time is None: self.startup_wait_time = now
            transforms_ready = all(self.get_transform(link) is not None for link in self.link_names)
            if transforms_ready:
                self.tf_warming_up = False
                self.start_time = now
                self.last_joint_error = self.waypoints[self.current_wp_idx] - self.current_joint_pos
            else:
                if (now - self.startup_wait_time).nanoseconds / 1e9 > 10.0:
                    self.save_results_and_exit(success=False, error="TF Tree Incomplete")
                return 

        target_joints = self.waypoints[self.current_wp_idx]
        joint_error = target_joints - self.current_joint_pos
        dist_to_goal = np.linalg.norm(joint_error)
        
        # Waypoint Switching Logic
        if dist_to_goal < 0.20: 
            if self.current_wp_idx < len(self.waypoints) - 1:
                self.current_wp_idx += 1
                self.integral_error = np.zeros(6)
                self.last_joint_error = np.zeros(6) 
                return 
            else:
                self.publish_joints(np.zeros(6))
                self.state = "FINISHED"
                self.save_results_and_exit(success=True)
                return
            
        # --- 1. PID ATTRACTION ---
        self.integral_error += joint_error * 0.02 
        self.integral_error = np.clip(self.integral_error, -0.5, 0.5)
        derivative_error = (joint_error - self.last_joint_error) / 0.02
        self.last_joint_error = joint_error
        
        T_att = (joint_error * self.alpha_att) + (self.integral_error * self.ki) + (derivative_error * self.kd)

        # --- 2. APF/VDPF REPULSION ---
        T_rep_total = np.zeros(6)
        is_avoiding_cube = False 
        
        link_positions = []
        for link in self.link_names:
            t = self.get_transform(link)
            if t is None: return 
            link_positions.append(np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]))

        for idx, P_cr in enumerate(link_positions):
            J_cr = self.compute_geometric_jacobian(idx, link_positions)
            
            # Static distance check
            dist_vec = P_cr - self.obstacle_position
            d_min = np.linalg.norm(dist_vec)
            
            if d_min < self.min_dist_observed: self.min_dist_observed = d_min
            if d_min < 0.04: d_min = 0.04 
            
            if d_min < self.rho_0:
                is_avoiding_cube = True 
                rep_mag = (1.0/d_min - 1.0/self.rho_0) * (1.0/d_min**2)
                
                # 45-Degree Rotation for VDPF Spiral Bypass
                cos_phi = np.cos(np.pi / 4)
                sin_phi = np.sin(np.pi / 4)
                
                F_out_x = dist_vec[0]
                F_out_y = dist_vec[1]
                
                # Counter-Clockwise Option
                F_ccw_x = F_out_x * cos_phi - F_out_y * sin_phi
                F_ccw_y = F_out_x * sin_phi + F_out_y * cos_phi
                dir_ccw = np.array([F_ccw_x, F_ccw_y, 0.0])
                dir_ccw = dir_ccw / (np.linalg.norm(dir_ccw) + 1e-6)
                
                # Clockwise Option
                F_cw_x = F_out_x * cos_phi + F_out_y * sin_phi
                F_cw_y = -F_out_x * sin_phi + F_out_y * cos_phi
                dir_cw = np.array([F_cw_x, F_cw_y, 0.0])
                dir_cw = dir_cw / (np.linalg.norm(dir_cw) + 1e-6)
                
                T_rep_ccw = np.dot(J_cr.T, rep_mag * dir_ccw)
                T_rep_cw = np.dot(J_cr.T, rep_mag * dir_cw)
                
                # Select the bypass direction that aligns better with the attraction goal
                if np.dot(T_rep_ccw, T_att) > np.dot(T_rep_cw, T_att):
                    T_rep_total += T_rep_ccw
                else:
                    T_rep_total += T_rep_cw

            # Floor Avoidance Logic
            z_height = P_cr[2]
            if z_height < self.rho_floor:
                z_height = max(z_height, 0.01)
                rep_mag_floor = self.alpha_rep_floor * (1.0/z_height - 1.0/self.rho_floor) * (1.0/z_height**2)
                F_rep_floor = rep_mag_floor * np.array([0.0, 0.0, 1.0])
                T_rep_total += np.dot(J_cr.T, F_rep_floor)

        # --- 3. MERGE FORCES ---
        if is_avoiding_cube:
            T_total = (self.zeta * T_att) + ((1 - self.zeta) * self.alpha_rep * T_rep_total)
        else:
            T_total = T_att + (self.alpha_rep * T_rep_total)

        # --- 4. OUTPUT TO SERVO ---
        mag = np.linalg.norm(T_total)
        dq = (T_total / mag) * self.max_joint_speed if mag > self.max_joint_speed else T_total
        final_velocities = np.nan_to_num(dq)
        
        # Accumulate Jitter for Cost Function
        velocity_diff = final_velocities - self.last_velocities
        self.total_jitter_score += np.sum(velocity_diff ** 2)
        self.last_velocities = final_velocities

        self.publish_joints(final_velocities)

    def publish_joints(self, velocities):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"   
        msg.joint_names = self.joint_names
        msg.velocities = velocities.tolist()
        self.servo_pub.publish(msg)

def main():
    rclpy.init()
    node = WholeArmAPFNode()
    try: rclpy.spin(node)
    except SystemExit: pass
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()