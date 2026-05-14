import os
import yaml
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    try:
        package_path = get_package_share_directory(package_name)
        absolute_file_path = os.path.join(package_path, file_path)
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML {file_path}: {e}")
        return None

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- 1. Robot Description (URDF) ---
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]), " ",
        "ur_type:=", ur_type, " ",
        "use_fake_hardware:=true ",
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- 2. Semantic Description (SRDF) ---
    moveit_config_pkg_path = get_package_share_directory(moveit_config_package.perform(context))
    srdf_path = os.path.join(moveit_config_pkg_path, "config", moveit_config_file.perform(context))
    
    # Fallback to 'srdf' folder if not in 'config'
    if not os.path.exists(srdf_path):
        srdf_path = os.path.join(moveit_config_pkg_path, "srdf", moveit_config_file.perform(context))

    with open(srdf_path, 'r') as f:
        semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": semantic_content}

    # --- 3. Load Configuration YAMLs ---
    kinematics_yaml = load_yaml(moveit_config_package.perform(context), "config/kinematics.yaml")
    joint_limits_yaml = load_yaml(moveit_config_package.perform(context), os.path.join("config", moveit_joint_limits_file.perform(context)))
    ompl_yaml = load_yaml(moveit_config_package.perform(context), "config/ompl_planning.yaml")
    sensors_3d_yaml = load_yaml(moveit_config_package.perform(context), "config/sensors_3d.yaml")

    # --- 4. Assemble MoveGroup Parameters ---
    # We combine them into a single dictionary for the Node 'parameters' argument
    move_group_params = {
        "use_sim_time": use_sim_time,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    # Add URDF and SRDF
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)

    # Add Kinematics, Joint Limits, and OMPL
    if kinematics_yaml:
        move_group_params.update({"robot_description_kinematics": kinematics_yaml})
    if joint_limits_yaml:
        move_group_params.update({"robot_description_planning": joint_limits_yaml})
    if ompl_yaml:
        move_group_params.update(ompl_yaml)
    
    # PERCEPTION CONFIGURATION (Phase 1 Fix)
    if sensors_3d_yaml:
        move_group_params.update(sensors_3d_yaml)

    # --- 5. Define MoveGroup Node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_params], # Pass as a list containing the dict
    )

    return [move_group_node]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur5e"),
        DeclareLaunchArgument("description_package", default_value="my_robot_description"),
        DeclareLaunchArgument("description_file", default_value="ur5e_hande.urdf.xacro"),
        DeclareLaunchArgument("moveit_config_package", default_value="my_ur5e_hande_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="ur5e_hande.srdf"),
        DeclareLaunchArgument("moveit_joint_limits_file", default_value="joint_limits.yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
