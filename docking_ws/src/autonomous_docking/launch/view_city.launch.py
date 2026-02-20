import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    pkg = get_package_share_directory('autonomous_docking')
    world = os.path.join(pkg, 'worlds', 'docking_world.world')

    # TurtleBot3 models path so Gazebo resolves model://turtlebot3_burger
    tb3_models = '/opt/ros/humble/share/turtlebot3_gazebo/models'
    existing = os.environ.get('GAZEBO_MODEL_PATH', '')
    model_path = tb3_models + (':' + existing if existing else '')

    return LaunchDescription([
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),

        # Start gzserver with city world (robot included in world SDF)
        ExecuteProcess(
            cmd=['gzserver', '--verbose', world,
                 '-slibgazebo_ros_init.so',
                 '-slibgazebo_ros_factory.so',
                 '-slibgazebo_ros_force_system.so'],
            output='screen',
            name='gzserver',
            additional_env={'GAZEBO_MODEL_PATH': model_path}
        ),

        # Start gzclient (GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen',
            name='gzclient'
        ),
    ])
