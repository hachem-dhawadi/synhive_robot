import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import yaml

def generate_launch_description():
    package_name = 'synhive_robot'
    pkg_path = get_package_share_directory(package_name)
    
    # Process URDF
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    # Pass robot_description to rsp.launch.py
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'robot_description': robot_description
        }.items()
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Robot spawner with delay
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )
    delayed_spawn = TimerAction(period=6.0, actions=[spawn_entity])

    # Controller manager with proper parameters
    controllers_yaml = os.path.join(pkg_path, 'config', 'my_controllers.yaml')
    with open(controllers_yaml, 'r') as f:
        controller_params = yaml.safe_load(f)
    
    controller_manager_params = {
        'controller_manager': controller_params['controller_manager']
    }


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_manager_params  # PASS EXTRACTED PARAMS
        ],
        output="screen",
    )

    # Controller spawners with DELAY
    load_joint_state = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    load_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # ADD CONTROLLER DELAY
    delayed_controllers = TimerAction(
        period=8.0,
        actions=[load_joint_state, load_diff_controller]
    )

    return LaunchDescription([
        rsp,
        gazebo,
        controller_manager,
        delayed_spawn,
        delayed_controllers,  # USE DELAYED CONTROLLERS
    ])