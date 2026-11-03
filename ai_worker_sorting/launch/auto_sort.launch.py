import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('ai_worker_sorting'), 'config', 'auto_sort.yaml')
    actions = [DeclareLaunchArgument('config', default_value=config),
               DeclareLaunchArgument('mock_mode', default_value='false'),
               DeclareLaunchArgument('python_prefix', default_value='')]
    for executable, name in [('vision_service','vision_service'), ('zone_service','zone_service'),
                             ('auto_worker','worker_service')]:
        params = [LaunchConfiguration('config')]
        if executable != 'auto_worker':
            params.append({'mock_mode': ParameterValue(LaunchConfiguration('mock_mode'), value_type=bool)})
        actions.append(Node(package='ai_worker_sorting', executable=executable, name=name,
                            prefix=LaunchConfiguration('python_prefix'), output='screen', parameters=params))
    return LaunchDescription(actions)
