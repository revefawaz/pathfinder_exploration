#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('pathfinder')

    # Allow use of simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    # Allow choosing which world to load
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_path, 'worlds', 'maze.world'),
        description='Full path to the world file to load'
    )
    world_arg = LaunchConfiguration('world')

    # Robot State Publisher (URDF → /tf)
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[    
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Static transform to connect odom → base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        output='screen',
        arguments=[
            '0', '0', '0',    # x y z
            '0', '0', '0',    # roll pitch yaw
            'odom', 'base_link'
        ]
    )

    # Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_arg,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn the robot into Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'pathfinder'
        ]
    )

    # SLAM (async online)
    slam_params = os.path.join(pkg_path, 'config', 'slam_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'slam_online_async_launch.py')
        ),
        launch_arguments={
            'params_file': slam_params,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Nav2 bringup
    nav2_params = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': use_sim_time
        }.items()
    )

    # In-repo exploration node
    explore_node = Node(
        package='pathfinder',
        executable='exploration_node.py',
        name='frontier_explorer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'goal_timeout': 90.0}
        ]
    )

    return LaunchDescription([
        declare_sim_time,
        declare_world_cmd,
        rsp,
        static_tf,
        gazebo,
        spawn,
        slam,
        nav2,
        explore_node,
    ])
