import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ai_worker_sorting'), 'config', 'worker_services.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('mock_mode', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='true'),
        # Prefix is useful for the existing system-site-packages YOLO venv.
        DeclareLaunchArgument('python_prefix', default_value=''),
        Node(package='ai_worker_sorting', executable='vision_service', name='vision_service',
             prefix=LaunchConfiguration('python_prefix'), output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'mock_mode': ParameterValue(LaunchConfiguration('mock_mode'), value_type=bool)}]),
        Node(package='ai_worker_sorting', executable='worker_service', name='worker_service',
             prefix=LaunchConfiguration('python_prefix'), output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'dry_run': ParameterValue(LaunchConfiguration('dry_run'), value_type=bool)}]),
    ])
