"""
Bring up the baseline system: Gazebo (turtlebot3_world) + Nav2 (AMCL + DWB) + RViz2.

This is the control condition for the thesis: stock Nav2 navigating the static
turtlebot3_world. Later phases reuse this launch file's structure, swapping in
the dynamic-obstacle world and the custom DWA controller plugin.

Usage:
  ros2 launch tb3_dynamic_nav_bringup baseline_nav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_pkg = get_package_share_directory('tb3_dynamic_nav_bringup')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_pkg, 'maps', 'turtlebot3_world.yaml')
    default_params = os.path.join(bringup_pkg, 'params', 'nav2_params.yaml')
    default_rviz = os.path.join(bringup_pkg, 'rviz', 'nav2_default_view.rviz')

    # NOTE: deliberately NOT named 'params_file' / 'map' — those LaunchConfiguration
    # names are also used internally by gazebo_ros's gzserver.launch.py and would
    # leak into it (DeclareLaunchArgument doesn't override an already-set global
    # LaunchConfiguration), causing gzserver to be started with --params-file
    # pointing at the Nav2 params and breaking its ROS plugin initialization
    # (no /spawn_entity service, no /odom).
    map_yaml_file = LaunchConfiguration('nav2_map')
    params_file = LaunchConfiguration('nav2_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_cmd = DeclareLaunchArgument(
        'nav2_map',
        default_value=default_map,
        description='Full path to the map yaml file to load for AMCL localization')

    declare_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=default_params,
        description='Full path to the Nav2 params file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock')

    gazebo_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py'))
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
        }.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        declare_map_cmd,
        declare_params_cmd,
        declare_use_sim_time_cmd,
        gazebo_world_cmd,
        nav2_bringup_cmd,
        rviz_cmd,
    ])
