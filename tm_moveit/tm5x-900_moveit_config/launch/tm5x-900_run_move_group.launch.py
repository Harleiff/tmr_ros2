############################################################################################### 
#  tm5x-900_run_move_group.launch.py
#   
#  Various portions of the code are based on original source from 
#  The reference: "https://github.com/moveit/moveit2/tree/main/moveit_ros/moveit_servo/launch"
#  and are used in accordance with the following license.
#  "https://github.com/moveit/moveit2/blob/main/LICENSE.txt"
############################################################################################### 

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Configure robot_description
    moveit_config_path = 'tm5x-900_moveit_config'
    srdf_path = 'config/tm5x-900.srdf'
    rviz_path = '/launch/run_move_group.rviz'

    moveit_config_share = get_package_share_directory(moveit_config_path)

    # -------------------------------------------------------------------------
    # Load the robot_description (moveit_config xacro includes the
    # mock_components/GenericSystem fake hardware needed for standalone mode)
    robot_description_config = xacro.process_file(
        os.path.join(moveit_config_share, 'config', 'tm5x-900.urdf.xacro'),
        mappings={
            'initial_positions_file': os.path.join(
                moveit_config_share, 'config', 'initial_positions.yaml'
            )
        },
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    # -------------------------------------------------------------------------

    # SRDF Configuration
    robot_description_semantic_config = load_file(moveit_config_path, srdf_path)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # Kinematics
    kinematics_yaml = load_yaml(moveit_config_path, 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # Planning Configuration
    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(moveit_config_path, 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)
    

    # Trajectory Execution Configuration
    # Controllers
    controllers_yaml = load_yaml(moveit_config_path, 'config/controllers.yaml')
    moveit_controllers = {'moveit_simple_controller_manager': controllers_yaml, 'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}

    # Trajectory Execution Functionality
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.1,
        'trajectory_execution.execution_duration_monitoring': False,
        'trajectory_execution.wait_for_trajectory_completion': True,
    }

    # Planning scene
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Joint limits
    joint_limits_yaml = {
        'robot_description_planning': load_yaml(
            moveit_config_path, 'config/joint_limits.yaml'
        )
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        emulate_tty=True,
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": False},
        ],
    )

    # RViz configuration
    rviz_config_file = (
        get_package_share_directory(moveit_config_path) + rviz_path
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        emulate_tty=True,
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            joint_limits_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # ros2_control: controller_manager driving the mock (fake) hardware
    ros2_controllers_path = os.path.join(
        moveit_config_share, 'config', 'ros2_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        emulate_tty=True,
        parameters=[robot_description, ros2_controllers_path],
    )

    # Spawn joint_state_broadcaster -> publishes /joint_states
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # Spawn the arm trajectory controller -> executes MoveIt trajectories
    tm_arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['tmr_arm_controller', '--controller-manager', '/controller_manager'],
    )
    # TCP Bridge Node (for Windows communication)
    tcp_bridge_node = Node(
        package='ella_controller',  # package name
        executable='tcp_communication',  #  TCP bridge executable
        name='tcp_communication',
        output='screen',
        parameters=[{
            'port': 9999,
            'host': '0.0.0.0'
        }]
    )
    tm_control_node = Node(
            package="ella_controller",
            executable="test_angles",
            output="screen",
        )

    # Launching all the nodes
    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            tm_arm_controller_spawner,
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            tcp_bridge_node,
            tm_control_node,
        ]
    )
