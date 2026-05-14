import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_pkg = "ur5e_hande_moveit_config"
    description_pkg = "my_robot_description"

    xacro_file = PathJoinSubstitution([
        FindPackageShare(description_pkg), "urdf", "ur5e_hande.urdf.xacro"
    ])

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file, 
            " use_fake_hardware:=", "false" 
        ]),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Load MoveIt Configuration
    moveit_config = (
        MoveItConfigsBuilder("ur5e_hande", package_name=moveit_pkg)
        .robot_description_semantic(file_path="config/ur5e_hande.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        
        # --- FIX: Point to the REAL ROBOT controller config ---
        .trajectory_execution(file_path="config/moveit_controllers_real.yaml")
        
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Convert to dict and add the missing Controller Manager parameter
    move_group_params = moveit_config.to_dict()
    move_group_params.update(robot_description)
    
    # --- Explicitly define the controller manager to prevent Segfault ---
    move_group_params.update({
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_manage_controllers": True,
    })
    # -----------------------------------------------------------------------

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            move_group_params,
            {"use_sim_time": False},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([FindPackageShare(moveit_pkg), "config", "moveit.rviz"])],
        parameters=[
            robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            {"use_sim_time": False},
        ],
    )

    # Only spawn the Arm Controller. 
    # NOTE: Ensure this name matches the YAML file below
    spawn_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
        output="screen",
    )
    
    return LaunchDescription([
        move_group_node,
        rviz_node,
        spawn_arm,
    ])
