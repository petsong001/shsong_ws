from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = []
    
    # 1. DECLARE ARGUMENTS
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # 2. GET THE URDF CONTENT (Uses the Hand-E only URDF)
    # We hardcode the port here to be absolutely safe
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("my_robot_description"), "urdf", "hande_only.urdf.xacro"] 
            ),
            " ",
            "use_fake_hardware:=", use_fake_hardware,
            " ",
            "com_port:=/tmp/ttyTool", 
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    # 3. Get the Controller Config
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ur5e_hande_moveit_config"),
            "config",
            "gripper_controllers.yaml",
        ]
    )

    # 4. Start the Gripper Driver Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # name="gripper_control_node",  <-- REMOVED TO MATCH YAML HEADER
        namespace="/gripper_manager",   
        parameters=[robot_description, robot_controllers], 
        output="screen",
        remappings=[
            ("/controller_manager/robot_description", "/gripper_manager/robot_description"),
        ],
    )

    # 5. Spawn the Gripper Controller
    # We target the namespaced controller manager
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hande_gripper_controller", "-c", "/gripper_manager/controller_manager"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            control_node,
            gripper_controller_spawner,
        ]
    )
