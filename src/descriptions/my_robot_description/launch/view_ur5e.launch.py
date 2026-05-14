import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. Path to your combined xacro file
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'ur5e_hande.urdf.xacro'
    )

    # 2. Define all the arguments your xacro file needs
    #    (These are the ones we discovered from all the errors)
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ',
        xacro_file, ' ',
        'ur_type:=ur5e', ' ',
        'robot_ip:=0.0.0.0', ' ',
        'tf_prefix:=""', ' ',
        'use_fake_hardware:=true', ' ',
        'fake_sensor_commands:=false', ' ',
        'safety_limits:=true', ' ',
        'safety_pos_margin:=0.15', ' ',
        'safety_k_position:=20', ' ',
        'kinematics_params_file:=$(find ur_description)/config/ur5e/default_kinematics.yaml', ' ',
        'physical_params_file:=$(find ur_description)/config/ur5e/physical_parameters.yaml', ' ',
        'visual_params_file:=$(find ur_description)/config/ur5e/visual_parameters.yaml', ' ',
        'joint_limit_params_file:=$(find ur_description)/config/ur5e/joint_limits.yaml', ' ',
        'script_filename:=""', ' ',
        'input_recipe_filename:=""', ' ',
        'output_recipe_filename:=""', ' ',
        'headless_mode:=false', ' ',
    ])

    return LaunchDescription([
        # No static transform needed, your xacro defines the 'world' link
        Node( # Robot State Publisher
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node( # Joint State Publisher (Sliders)
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node( # RViz
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('ur_description'), 'rviz', 'view_robot.rviz')]
        )
    ])
