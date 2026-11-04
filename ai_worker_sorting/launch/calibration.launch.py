"""Camera-only setup stack: no worker and no actuator publishers."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    value = lambda key:LaunchConfiguration(key).perform(context)
    if not Path(value('weights')).is_file():
        raise ValueError('Local YOLO weights are required')
    common = dict(mock_mode=False,enable_depth=True,calibrated=True,base_frame=value('base_frame'),
        detections_topic='/worker_camera/detections',depth_topic='/worker_camera/depth_rect',
        camera_info_topic='/worker_camera/camera_info')
    specs = [('rgbd_input',dict(color_topic=value('color_topic'),depth_topic=value('depth_topic'),
                camera_info_topic=value('camera_info_topic'))),
             ('rgbd_detector',dict(weights=value('weights'),device=value('device'),image_topic='/worker_camera/image_rect')),
             ('vision_service',common),
             ('zone_service',dict(common,image_topic='/worker_camera/image_rect'))]
    return [Node(package='ai_worker_sorting',executable=exe,name=exe,
                 prefix=value('python_prefix'),parameters=[params],output='screen') for exe,params in specs]


def generate_launch_description():
    args = dict(device='cpu',base_frame='base_link',python_prefix='',
        color_topic='/camera/color/image_raw',depth_topic='/camera/aligned_depth_to_color/image_raw',
        camera_info_topic='/camera/color/camera_info')
    return LaunchDescription([DeclareLaunchArgument('weights')]+
        [DeclareLaunchArgument(k,default_value=v) for k,v in args.items()]+[OpaqueFunction(function=setup)])
