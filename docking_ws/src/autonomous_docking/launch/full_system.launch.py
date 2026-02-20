import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    pkg      = get_package_share_directory('autonomous_docking')
    world    = os.path.join(pkg, 'worlds', 'docking_world.world')
    map_yaml = os.path.join(pkg, 'maps',   'docking_map.yaml')
    nav2_cfg = os.path.join(pkg, 'config', 'nav2_params.yaml')
    wp_cfg   = os.path.join(pkg, 'config', 'waypoints.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz',   'docking.rviz')
    urdf     = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_burger.urdf'

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    tb3_models = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    mdl_path   = tb3_models + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')

    return LaunchDescription([

        # ── Env vars ────────────────────────────────────────────────────────
        SetEnvironmentVariable('TURTLEBOT3_MODEL',  'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', mdl_path),

        # ── 1. Gazebo server with city world + embedded TurtleBot3 ──────────
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                 '-slibgazebo_ros_init.so',
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so'],
            output='screen', name='gzserver',
            additional_env={'GAZEBO_MODEL_PATH': mdl_path}
        ),

        # ── 2. Gazebo client (GUI) ───────────────────────────────────────────
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen', name='gzclient'
        ),

        # ── 3. robot_state_publisher (TF: base_footprint→base_scan etc.) ────
        #    Needed so AMCL can match laser scan to map frame.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
            }]
        ),

        # ── 4. Nav2 bringup (AMCL + costmap + planner + controller) ─────────
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
                    f'map:={map_yaml}',
                    'use_sim_time:=true',
                    f'params_file:={nav2_cfg}',
                ],
                output='screen', name='nav2'
            ),
        ]),


            Node(
                package='autonomous_docking',
                executable='battery_manager_node.py',
                name='battery_manager',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),

            Node(
                package='autonomous_docking',
                executable='navigation_manager_node.py',
                name='navigation_manager',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),

            Node(
                package='autonomous_docking',
                executable='mission_planner_node.py',
                name='mission_planner',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'waypoints_config': wp_cfg,
                }]
            ),

            Node(
                package='autonomous_docking',
                executable='docking_controller_node.py',
                name='docking_controller',
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),

        # ── 6. RViz ─────────────────────────────────────────────────────────
        TimerAction(period=6.0, actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen',
                parameters=[{'use_sim_time': True}]
            ),
        ]),
    ])
