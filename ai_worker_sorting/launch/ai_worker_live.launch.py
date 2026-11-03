"""Real RGB-D/YOLO/controller stack. Starting nodes does not start a sorting job."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    from ai_worker_sorting.live_config import validate_live_config
    config = LaunchConfiguration('config').perform(context)
    validate_live_config(config)
    actions = []
    camera_driver = LaunchConfiguration('camera_driver').perform(context)
    if camera_driver=='realsense':
        serial = LaunchConfiguration('camera_serial').perform(context)
        if not serial:
            raise ValueError('camera_serial is required to select the intended camera among head/wrist devices')
        path = os.path.join(get_package_share_directory('realsense2_camera'),'launch','rs_launch.py')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments={
            'camera_namespace':'','camera_name':'camera','enable_color':'true','enable_depth':'true',
            'align_depth.enable':'true','enable_sync':'true',
            'serial_no':'_'+serial.lstrip('_'),
        }.items()))
    elif camera_driver!='external':
        raise ValueError('camera_driver must be external or realsense')
    for executable,name in [('rgbd_input','rgbd_input'),('rgbd_detector','rgbd_detector'),
                            ('vision_service','vision_service'),('zone_service','zone_service'),
                            ('auto_worker','worker_service')]:
        actions.append(Node(package='ai_worker_sorting',executable=executable,name=name,
                            prefix=LaunchConfiguration('python_prefix'),parameters=[config],output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config',description='Absolute path to the measured live local YAML profile'),
        DeclareLaunchArgument('camera_driver',default_value='external'),
        DeclareLaunchArgument('camera_serial',default_value=''),
        DeclareLaunchArgument('python_prefix',default_value=''),
        OpaqueFunction(function=setup),
    ])
