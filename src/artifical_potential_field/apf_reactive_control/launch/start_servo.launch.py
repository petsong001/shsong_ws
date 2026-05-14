import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load MoveIt Configs
    moveit_config = (
        MoveItConfigsBuilder("ur5e_hande", package_name="my_ur5e_hande_moveit_config")
        .to_moveit_configs()
    )

    # 2. Declare Optimizer Arguments
    alpha_att_arg = DeclareLaunchArgument('alpha_att', default_value='20.0')
    ki_arg = DeclareLaunchArgument('ki', default_value='1.0')
    kd_arg = DeclareLaunchArgument('kd', default_value='0.5')

    servo_params = {
        "use_gazebo": False,
        "move_group_name": "ur_manipulator",
        "planning_frame": "base_link",
        "is_primary_planning_scene_monitor": True,
        "robot_link_command_frame": "base_link",
        "ee_frame_name": "tool0",
        "cartesian_command_in_topic": "~/delta_twist_cmds",
        "joint_command_in_topic": "~/delta_joint_cmds",
        "command_in_type": "unitless",
        "command_out_topic": "/apf_servo_controller/commands",
        "command_out_type": "std_msgs/Float64MultiArray",
        "publish_joint_positions": True,
        "publish_joint_velocities": False,
        "publish_joint_accelerations": False,
        "incoming_command_timeout": 0.5,
        "publish_period": 0.02, 
        "low_pass_filter_coeff": 1.5,
        "scale": {"linear": 1.0, "rotational": 1.0, "joint": 1.0},
        "lower_singularity_threshold": 70.0, 
        "hard_stop_singularity_threshold": 100.0,
        "joint_limit_margin": 0.1,
        "check_collisions": True,         
        "collision_check_rate": 10.0,
    }

    # 3. Define the Nodes
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        parameters=[
            {"moveit_servo": servo_params},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics, 
            {"use_sim_time": True}
        ],
        output="screen",
    )

    load_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["apf_servo_controller", "--inactive", "-p", os.path.join(get_package_share_directory("apf_reactive_control"), "config", "ur_servo.yaml")],
        output="screen"
    )

    apf_controller_node = Node(
        package="apf_reactive_control",
        executable="controller", 
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'alpha_att': LaunchConfiguration('alpha_att'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd')
        }]
    )

    # 4. CRITICAL: The Shutdown Event Handler
    # This detects when the 'controller' node finishes (on success or timeout) 
    # and shuts down the rest of the launch (like servo_node).
    shutdown_on_controller_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=apf_controller_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason='Optimization iteration finished.'))
            ],
        )
    )

    return LaunchDescription([
        alpha_att_arg,
        ki_arg,
        kd_arg,
        servo_node, 
        load_controller,
        # Start the controller after a 5s delay to ensure servo_node is ready
        TimerAction(period=5.0, actions=[apf_controller_node]),
        # Register the shutdown logic
        shutdown_on_controller_exit
    ])