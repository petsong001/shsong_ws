import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Define the path to the URDF file we want to display
    urdf_file_path = os.path.expanduser('~/shsong_ws/hande_only.urdf')

    # Read the URDF file content
    try:
        robot_description_raw = open(urdf_file_path, 'r').read()
    except FileNotFoundError:
        print(f"ERROR: URDF file not found at {urdf_file_path}")
        return LaunchDescription([]) # Return empty if file not found

    robot_description = {'robot_description': robot_description_raw}

    # Rviz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('ur_description'),
        'rviz',
        'view_robot.rviz' # Use the standard UR Rviz config
    )

    # Node to publish joint states (static dummy values)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Node to publish robot state (transforms based on URDF and joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])