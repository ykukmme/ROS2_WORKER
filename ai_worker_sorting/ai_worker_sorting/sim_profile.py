"""Validated scene defaults; no operator measurements are used for a virtual world."""
from pathlib import Path


def resolve_weights(output):
    import shutil
    weights = Path(output)/'simulation_yolo.pt'
    if not weights.exists():
        bundled = Path('/opt/worker/sim-models/simulation_yolo.pt')
        if not bundled.is_file():
            raise ValueError('Simulation weights missing. Run: bash scripts/sim_worker.sh train')
        shutil.copyfile(bundled,weights)
    return weights


def profile(weights):
    weights = Path(weights)
    if not weights.is_file():
        raise ValueError('Simulation YOLO weights are missing; run train first')
    common = dict(use_sim_time=True)
    vision = dict(common, mock_mode=False, observation_source='mujoco', enable_depth=True,
                  calibrated=True,base_frame='sim_base',max_age=3.,
                  detections_topic='/worker_camera/detections',depth_topic='/worker_camera/depth_rect',
                  camera_info_topic='/worker_camera/camera_info')
    return {
        'rgbd_input': dict(common,color_topic='/sim/camera/color/image_raw',
                           depth_topic='/sim/camera/aligned_depth_to_color/image_raw',
                           camera_info_topic='/sim/camera/color/camera_info',input_is_rectified=True,max_age=3.),
        'rgbd_detector': dict(common,weights=str(weights),device='cpu',confidence=.65,imgsz=416,
                             inference_hz=5.,max_age=3.),
        'vision_service': vision,
        'zone_service': dict(vision,image_topic='/worker_camera/image_rect',min_zone_area=1500.),
        'worker_service': dict(common,runtime_mode='mujoco',dry_run=False,hardware_configured=False,
            robot_backend='mujoco',base_frame='sim_base',controller_frame='sim_base',tool_link='sim_tcp',
            controller_node='/sim/controller',movel_topic='/sim/left/goal_move',
            gripper_trajectory_topic='/sim/left/gripper_trajectory',joint_states_topic='/sim/joint_states',
            gripper_joint='gripper_l_joint1',gripper_open=0.,gripper_closed=1.1,
            gripper_joint_min=0.,gripper_joint_max=1.1,gripper_duration=.8,gripper_tolerance=.025,
            grasp_check_service='/sim/check_grasp',grasp_verification='physics_contact',shared_grasp_verified=True,
            supported_classes=['type_a','type_b'],class_grasp_offsets='{"type_a":-0.005,"type_b":-0.005}',
            auto_grasp_offset=-.005,object_size=.07,slot_margin=.005,place_offset=.07,slot_pitch_extra=.01,
            source_roi=[0.,0.,640.,480.],workspace=[.18,.65,-.02,.58,.73,1.0],
            home_position=[.25,.52,.9],home_mode='current_tcp',tool_quaternion=[0.,0.,0.,1.],
            placement_height_mode='object_surface',transport_above_objects=True,tool_below_tcp=.06,
            max_object_height=.07,min_object_height=.02,clearance=.08,release_clearance=.025,
            check_camera_footprint=True,min_confidence=.65,target_max_age=3.,service_timeout=8.,
            feedback_timeout=3.,move_duration=1.5,cartesian_speed=.08,angular_speed=.3,
            action_timeout=90.,task_timeout=600.,empty_scans=4,position_tolerance=.008,orientation_tolerance=.08)
    }


def validate_profile(config):
    worker = config['worker_service']
    if worker['runtime_mode']!='mujoco' or worker['robot_backend']!='mujoco' or worker['hardware_configured'] or worker['dry_run']:
        raise ValueError('Physical simulation profile required')
    if not all(section.get('use_sim_time') is True for section in config.values()):
        raise ValueError('Every simulation consumer must use /clock')
    if not worker['base_frame'].startswith('sim_') or not worker['tool_link'].startswith('sim_'):
        raise ValueError('Simulation frames required')
    for name in ('vision_service','zone_service'):
        if config[name]['mock_mode'] or config[name]['observation_source']!='mujoco':
            raise ValueError('Rendered observations required')
    return config
