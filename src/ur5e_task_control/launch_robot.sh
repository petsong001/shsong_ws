#!/bin/bash

# --- TRAP: Ensure socat dies when this script exits ---
cleanup() {
    echo "Shutting down..."
    if [ ! -z "$SOCAT_PID" ]; then
        kill -9 $SOCAT_PID 2>/dev/null
    fi
    rm -f /tmp/ttyTool
}
trap cleanup EXIT INT TERM

# 1. Force Clean previous ghosts
pkill -9 -f "socat pty,link=/tmp/ttyTool" 2>/dev/null
rm -f /tmp/ttyTool

# 2. Start Bridge (Manual Mode)
echo "Starting Tool Communication Bridge..."
socat pty,link=/tmp/ttyTool,raw,ignoreeof,waitslave tcp:192.168.1.100:54321,tcp-nodelay &
SOCAT_PID=$!

# 3. Wait for port
echo "Waiting for connection to robot..."
while [ ! -e /tmp/ttyTool ]; do
  sleep 0.2
done
echo "Connection established at /tmp/ttyTool"

# 4. Launch Driver
source ~/shsong_ws/install/setup.bash

echo "Starting Robot Driver..."
ros2 launch ur_robot_driver ur_control.launch.py \
 ur_type:=ur5e \
 robot_ip:=192.168.1.100 \
 reverse_ip:=192.168.1.102 \
 kinematics_params_file:=/home/iar/shsong_ws/src/descriptions/my_robot_description/config/my_robot_calibration.yaml \
 launch_rviz:=false \
 description_package:=my_robot_description \
 description_file:=ur5e_hande.urdf.xacro \
 controllers_file:=$(ros2 pkg prefix ur5e_hande_moveit_config)/share/ur5e_hande_moveit_config/config/ros2_controllers.yaml \
 use_fake_hardware:=false \
 use_tool_communication:=false \
 tool_device_name:=/tmp/ttyTool \
 tool_tcp_port:=54321 \
 tool_voltage:=24 \
 tool_parity:=0 \
 tool_baud_rate:=115200 \
 tool_stop_bits:=1 \
 tool_rx_idle_chars:=1.5 \
 tool_tx_idle_chars:=3.5 \
 initial_joint_controller:=scaled_joint_trajectory_controller \
 activate_joint_controller:=true &

DRIVER_PID=$!

# 5. Wait and Spawn Gripper
echo "Waiting 15 seconds for driver stabilization..."
sleep 15
echo "Spawning Gripper Controller..."
ros2 run controller_manager spawner hande_gripper_controller --controller-manager /controller_manager

# 6. Keep script alive
wait $DRIVER_PID
