"""
Bring up the dynamic-obstacle system: Gazebo + scripted obstacle mover + Nav2 + RViz2.

Two visible cylinders move on scripted paths (config/obstacles.yaml).
Nav2 uses the controller plugin selected by nav2_params.yaml.

Usage:
  ros2 launch tb3_dynamic_nav_bringup dynamic_nav.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle_pi')

    bringup_pkg = get_package_share_directory('tb3_dynamic_nav_bringup')
    obstacles_pkg = get_package_share_directory('tb3_moving_obstacles')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(bringup_pkg, 'maps', 'turtlebot3_world.yaml')
    default_params = os.path.join(bringup_pkg, 'params', 'nav2_params.yaml')
    default_rviz = os.path.join(bringup_pkg, 'rviz', 'nav2_default_view.rviz')
    obstacles_config = os.path.join(obstacles_pkg, 'config', 'obstacles.yaml')
    world_file = os.path.join(obstacles_pkg, 'worlds', 'turtlebot3_dynamic_world.world')
    nav2_bt_navigator_pkg = get_package_share_directory('nav2_bt_navigator')
    default_bt_xml = os.path.join(
        nav2_bt_navigator_pkg, 'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml')

    map_yaml_file = LaunchConfiguration('nav2_map')
    params_file = LaunchConfiguration('nav2_params_file')
    bt_xml_file = LaunchConfiguration('default_nav_to_pose_bt_xml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    declare_map_cmd = DeclareLaunchArgument(
        'nav2_map',
        default_value=default_map,
        description='Full path to the map yaml file to load for AMCL localization')

    declare_params_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=default_params,
        description='Full path to the Nav2 params file to load')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=default_bt_xml,
        description='Full path to the BT XML file used by bt_navigator')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock')

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='-2.0', description='Initial robot x position')

    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-0.5', description='Initial robot y position')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_pkg, 'launch', 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    obstacle_mover_cmd = Node(
        package='tb3_moving_obstacles',
        executable='obstacle_mover',
        name='obstacle_mover',
        output='screen',
        parameters=[obstacles_config, {'use_sim_time': use_sim_time}]
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_nav_to_pose_bt_xml': bt_xml_file,
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
        declare_bt_xml_cmd,
        declare_use_sim_time_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        obstacle_mover_cmd,
        nav2_bringup_cmd,
        rviz_cmd,
    ])
