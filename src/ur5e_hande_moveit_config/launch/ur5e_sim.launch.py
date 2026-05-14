from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
import subprocess
import re  
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # 1. Path to your Xacro
    desc_pkg_path = get_package_share_directory("my_robot_description")
    xacro_path = os.path.join(desc_pkg_path, "urdf", "ur5e_hande.urdf.xacro")
    
    # 2. Process Xacro with the Isaac Sim argument enabled
    robot_description_content = subprocess.check_output([
        "xacro", xacro_path, 
        "use_isaac_sim:=true",
        "use_fake_hardware:=true"
    ]).decode("utf-8")
    
    # Clean up Xacro for Isaac Sim Compatibility (Remove sensors/gpios)
    robot_description_content = re.sub(r'<sensor name=".*?">.*?</sensor>', '', robot_description_content, flags=re.DOTALL)
    robot_description_content = re.sub(r'<gpio name=".*?">.*?</gpio>', '', robot_description_content, flags=re.DOTALL)
    
    # --- THE MAGIC MERGE: Fusing the Arm and Gripper Hardware Interfaces ---
    # Step A: Extract all hardware joints from any existing ros2_control blocks
    joints = []
    ros2_control_blocks = re.findall(r'<ros2_control.*?</ros2_control>', robot_description_content, flags=re.DOTALL)
    for block in ros2_control_blocks:
        joint_matches = re.findall(r'<joint name=".*?">.*?</joint>', block, flags=re.DOTALL)
        joints.extend(joint_matches)
        
    # Step B: Create ONE unified block that controls all 7 joints together
    unified_ros2_control = f"""
    <ros2_control name="IsaacSimUnifiedInterface" type="system">
        <hardware>
            <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
            <param name="joint_commands_topic">/joint_commands</param>
            <param name="joint_states_topic">/joint_states</param>
        </hardware>
        {''.join(joints)}
    </ros2_control>
    """
    
    # Step C: Erase the old conflicting blocks and inject the unified one
    robot_description_content = re.sub(r'<ros2_control.*?</ros2_control>', '', robot_description_content, flags=re.DOTALL)
    robot_description_content = robot_description_content.replace('</robot>', unified_ros2_control + '\n</robot>')
    # -----------------------------------------------------------------------

    # 3. Build MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("ur5e_hande", package_name="ur5e_hande_moveit_config")
        .robot_description(file_path=xacro_path) 
        .trajectory_execution(file_path="config/moveit_controllers_sim.yaml") 
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    
    # Overwrite with our newly merged content
    moveit_config.robot_description = {'robot_description': robot_description_content}

    # 4. Define Nodes
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description, 
            os.path.join(moveit_config.package_path, "config", "ros2_controllers.yaml"), 
            {'use_sim_time': True}
        ],
        output="screen"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher", 
        parameters=[moveit_config.robot_description, {'use_sim_time': True}]
    )

    move_group_node = Node(
        package="moveit_ros_move_group", 
        executable="move_group", 
        output="screen",
        parameters=[
            moveit_config.to_dict(), 
            {
                'use_sim_time': True,
                'trajectory_execution.allowed_start_tolerance': 0.05, 
                'default_planning_pipeline': 'ompl',
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/ResolveConstraintFrames '
                                    'default_planner_request_adapters/ValidateWorkspaceBounds '
                                    'default_planner_request_adapters/CheckStartStateBounds '
                                    'default_planner_request_adapters/CheckStartStateCollision '
                                    'default_planner_request_adapters/AddTimeOptimalParameterization',
            }
        ]
    )

    rviz_node = Node(
        package="rviz2", 
        executable="rviz2", 
        arguments=["-d", os.path.join(moveit_config.package_path, "config", "moveit.rviz")], 
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {'use_sim_time': True}
        ]
    )

    # 5. Spawners
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])
    arm_spawner = Node(package="controller_manager", executable="spawner", arguments=["scaled_joint_trajectory_controller", "--controller-manager", "/controller_manager"])
    grip_spawner = Node(package="controller_manager", executable="spawner", arguments=["hande_gripper_controller", "--controller-manager", "/controller_manager"])

    return LaunchDescription([
        use_sim_time, ros2_control_node, robot_state_publisher, move_group_node, rviz_node,
        RegisterEventHandler(event_handler=OnProcessStart(target_action=ros2_control_node, on_start=[jsb_spawner])),
        RegisterEventHandler(event_handler=OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner, grip_spawner])),
    ])