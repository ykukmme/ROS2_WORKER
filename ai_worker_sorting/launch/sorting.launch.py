from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ai_worker_sorting'), 'config', 'sorting.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='ai_worker_sorting', executable='perception', name='perception',
             parameters=[LaunchConfiguration('config')]),
        Node(package='ai_worker_sorting', executable='task_manager', name='task_manager',
             parameters=[LaunchConfiguration('config')]),
    ])
