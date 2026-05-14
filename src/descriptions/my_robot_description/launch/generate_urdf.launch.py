import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # --- Declare ALL arguments needed by ur.urdf.xacro and our top-level xacro ---
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument("ur_type", default_value="ur5e"))
    declared_arguments.append(DeclareLaunchArgument("robot_ip", default_value="0.0.0.0"))
    declared_arguments.append(DeclareLaunchArgument("tf_prefix", default_value=""))
    declared_arguments.append(DeclareLaunchArgument("use_fake_hardware", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("fake_sensor_commands", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("safety_limits", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("safety_pos_margin", default_value="0.15"))
    declared_arguments.append(DeclareLaunchArgument("safety_k_position", default_value="20"))
    
    # --- These are the file paths ---
    declared_arguments.append(DeclareLaunchArgument("kinematics_params_file_path", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("ur_type"), "default_kinematics.yaml"])))
    declared_arguments.append(DeclareLaunchArgument("physical_params_file_path", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("ur_type"), "physical_parameters.yaml"])))
    declared_arguments.append(DeclareLaunchArgument("visual_params_file_path", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("ur_type"), "visual_parameters.yaml"])))
    declared_arguments.append(DeclareLaunchArgument("joint_limit_params_file_path", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration("ur_type"), "joint_limits.yaml"])))
    
    declared_arguments.append(DeclareLaunchArgument("script_filename", default_value=""))
    declared_arguments.append(DeclareLaunchArgument("input_recipe_filename", default_value=""))
    declared_arguments.append(DeclareLaunchArgument("output_recipe_filename", default_value=""))
    declared_arguments.append(DeclareLaunchArgument("headless_mode", default_value="false"))
    declared_arguments.append(DeclareLaunchArgument("use_tool_communication", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("tool_parity", default_value="0"))
    declared_arguments.append(DeclareLaunchArgument("tool_baud_rate", default_value="115200"))
    declared_arguments.append(DeclareLaunchArgument("tool_stop_bits", default_value="1"))
    declared_arguments.append(DeclareLaunchArgument("tool_rx_idle_chars", default_value="1.5"))
    declared_arguments.append(DeclareLaunchArgument("tool_tx_idle_chars", default_value="3.5"))
    declared_arguments.append(DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR"))
    declared_arguments.append(DeclareLaunchArgument("tool_tcp_port", default_value="54321"))
    declared_arguments.append(DeclareLaunchArgument("tool_voltage", default_value="0"))
    declared_arguments.append(DeclareLaunchArgument("reverse_ip", default_value="0.0.0.0"))
    declared_arguments.append(DeclareLaunchArgument("script_command_port", default_value="50004"))
    declared_arguments.append(DeclareLaunchArgument("reverse_port", default_value="50001"))
    declared_arguments.append(DeclareLaunchArgument("script_sender_port", default_value="50002"))
    declared_arguments.append(DeclareLaunchArgument("trajectory_port", default_value="50003"))
    declared_arguments.append(DeclareLaunchArgument("robotiq_gripper", default_value="true"))
    declared_arguments.append(DeclareLaunchArgument("robotiq_gripper_model", default_value="hande"))


    # Path to your combined xacro file
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'ur5e_hande.urdf.xacro'
    )

    # --- Corrected xacro command ---
    # Note: The xacro argument name matches the error log: 'kinematics_parameters_file'
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        xacro_file, ' ',
        'name:=', LaunchConfiguration("ur_type"), ' ', # Pass 'name'
        'tf_prefix:=', LaunchConfiguration("tf_prefix"), ' ',
        'ur_type:=', LaunchConfiguration("ur_type"), ' ',
        'robot_ip:=', LaunchConfiguration("robot_ip"), ' ',
        'use_fake_hardware:=', LaunchConfiguration("use_fake_hardware"), ' ',
        'fake_sensor_commands:=', LaunchConfiguration("fake_sensor_commands"), ' ',
        'safety_limits:=', LaunchConfiguration("safety_limits"), ' ',
        'safety_pos_margin:=', LaunchConfiguration("safety_pos_margin"), ' ',
        'safety_k_position:=', LaunchConfiguration("safety_k_position"), ' ',
        # --- THIS IS THE FIX ---
        'kinematics_parameters_file:=', LaunchConfiguration("kinematics_params_file_path"), ' ',
        'physical_parameters_file:=', LaunchConfiguration("physical_params_file_path"), ' ',
        'visual_parameters_file:=', LaunchConfiguration("visual_params_file_path"), ' ',
        'joint_limits_parameters_file:=', LaunchConfiguration("joint_limit_params_file_path"), ' ',
        # --- END OF FIX ---
        'script_filename:=', LaunchConfiguration("script_filename"), ' ',
        'input_recipe_filename:=', LaunchConfiguration("input_recipe_filename"), ' ',
        'output_recipe_filename:=', LaunchConfiguration("output_recipe_filename"), ' ',
        'headless_mode:=', LaunchConfiguration("headless_mode"), ' ',
        'use_tool_communication:=', LaunchConfiguration("use_tool_communication"), ' ',
        'tool_parity:=', LaunchConfiguration("tool_parity"), ' ',
        'tool_baud_rate:=', LaunchConfiguration("tool_baud_rate"), ' ',
        'tool_stop_bits:=', LaunchConfiguration("tool_stop_bits"), ' ',
        'tool_rx_idle_chars:=', LaunchConfiguration("tool_rx_idle_chars"), ' ',
        'tool_tx_idle_chars:=', LaunchConfiguration("tool_tx_idle_chars"), ' ',
        'tool_device_name:=', LaunchConfiguration("tool_device_name"), ' ',
        'tool_tcp_port:=', LaunchConfiguration("tool_tcp_port"), ' ',
        'tool_voltage:=', LaunchConfiguration("tool_voltage"), ' ',
        'reverse_ip:=', LaunchConfiguration("reverse_ip"), ' ',
        'script_command_port:=', LaunchConfiguration("script_command_port"), ' ',
        'reverse_port:=', LaunchConfiguration("reverse_port"), ' ',
        'script_sender_port:=', LaunchConfiguration("script_sender_port"), ' ',
        'trajectory_port:=', LaunchConfiguration("trajectory_port"), ' ',
        'robotiq_gripper:=', LaunchConfiguration("robotiq_gripper"), ' ',
        'robotiq_gripper_model:=', LaunchConfiguration("robotiq_gripper_model"), ' ',
    ])

    return LaunchDescription(declared_arguments + [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        )
    ])
