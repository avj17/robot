import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('autonomous_docking')
    tb3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg, 'worlds', 'docking_world.world')
    map_file   = os.path.join(pkg, 'maps',   'docking_map.yaml')
    nav2_params= os.path.join(pkg, 'config', 'nav2_params.yaml')
    wp_config  = os.path.join(pkg, 'config', 'waypoints.yaml')
    rviz_cfg   = os.path.join(pkg, 'rviz',   'docking.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        # ── Env ──────────────────────────────────────────
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),

        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── Gazebo ───────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_pkg, 'launch', 'turtlebot3_world.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # ── Nav2 ─────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_pkg, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
            }.items(),
        ),

        # ── Mission nodes ─────────────────────────────────
        Node(
            package='autonomous_docking',
            executable='battery_manager_node.py',
            name='battery_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autonomous_docking',
            executable='mission_planner_node.py',
            name='mission_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'waypoints_config': wp_config}],
        ),
        Node(
            package='autonomous_docking',
            executable='navigation_manager_node.py',
            name='navigation_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='autonomous_docking',
            executable='docking_controller_node.py',
            name='docking_controller',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── RViz ─────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
