from glob import glob
from setuptools import setup

setup(
    name='ai_worker_sorting', version='0.1.0',
    packages=['ai_worker_sorting'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ai_worker_sorting']),
        ('share/ai_worker_sorting', ['package.xml']),
        ('share/ai_worker_sorting/config', glob('config/*.yaml')),
        ('share/ai_worker_sorting/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'perception = ai_worker_sorting.perception:main',
        'task_manager = ai_worker_sorting.task_manager:main',
        'demo = ai_worker_sorting.demo:main',
        'image_smoke = ai_worker_sorting.image_smoke:main',
        'webcam = ai_worker_sorting.webcam:main',
        'webcam_bridge = ai_worker_sorting.webcam_bridge:main',
        'ros_camera = ai_worker_sorting.ros_camera:main',
        'worker_service = ai_worker_sorting.worker_service:main',
        'vision_service = ai_worker_sorting.vision_service:main',
        'zone_service = ai_worker_sorting.zone_service:main',
        'auto_worker = ai_worker_sorting.auto_worker:main',
        'rgbd_input = ai_worker_sorting.rgbd_input:main',
        'rgbd_detector = ai_worker_sorting.rgbd_detector:main',
        'configure_live = ai_worker_sorting.configure_live:main',
        'worker_cli = ai_worker_sorting.worker_cli:main',
    ]},
)
