from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 1. Robot Description (Hand-E Only)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("my_robot_description"), "urdf", "hande_only.urdf.xacro"]
            ),
            " ",
            "com_port:=/tmp/ttyTool",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Controller Config Path
    gripper_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_hande_moveit_config"),
            "config",
            "gripper_controllers.yaml",
        ]
    )

    # 3. The Gripper Node
    # We use the standard 'controller_manager' name inside the namespace
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="gripper_manager",  # Namespace without leading slash often helps
        parameters=[robot_description, gripper_controllers],
        output="both",
        remappings=[
            ("/controller_manager/robot_description", "/gripper_manager/robot_description"),
        ],
    )

    # 4. The Spawner
    # Explicitly targeting the manager in the namespace
    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hande_gripper_controller", "-c", "/gripper_manager/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        gripper_spawner,
    ])
