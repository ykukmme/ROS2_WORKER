"""Validate the hardware profile before a live stack can start."""
from pathlib import Path
import math
import json
import yaml


def validate_live_config(path):
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    required = ('rgbd_input','rgbd_detector','vision_service','zone_service','worker_service')
    cfg = {}
    for name in required:
        try:
            cfg[name] = document[name]['ros__parameters']
        except (TypeError,KeyError):
            raise ValueError(f'Missing live configuration section: {name}') from None
    worker = cfg['worker_service']
    offsets = json.loads(worker.get('class_grasp_offsets','{}'))
    if not isinstance(offsets,dict) or any(not isinstance(k,str) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>=.5 for k,v in offsets.items()):
        raise ValueError('Invalid per-class grasp offsets')
    if worker.get('check_camera_footprint') and len(offsets)!=2:
        raise ValueError('Camera setup requires taught grasp offsets for exactly two classes')
    if worker.get('dry_run') is not False or worker.get('robot_backend')!='robotis_movel':
        raise ValueError('Live launch requires dry_run: false and robot_backend: robotis_movel')
    if worker.get('hardware_configured') is not True or worker.get('shared_grasp_verified') is not True:
        raise ValueError('Live hardware and shared fixed-grasp settings must be configured')
    if worker.get('home_mode')!='current_tcp' or worker.get('placement_height_mode')!='object_surface' or worker.get('transport_above_objects') is not True:
        raise ValueError('Live launch requires measured home, measured object height and raised transport')
    height = worker.get('max_object_height',.25)
    if not math.isfinite(height) or not 0<height<=.5:
        raise ValueError('Maximum object height must be a positive measured bound')
    if not math.isfinite(worker.get('auto_grasp_offset',float('nan'))):
        raise ValueError('Measured TCP grasp offset is required')
    tool_extent = worker.get('tool_below_tcp',float('nan'))
    if not math.isfinite(tool_extent) or not 0<=tool_extent<=.5:
        raise ValueError('Measured downward tool extent is required')
    if not worker.get('controller_node'):
        raise ValueError('Actual Cyclo controller node is required')
    if not worker.get('base_frame') or not worker.get('controller_frame'):
        raise ValueError('Robot and controller coordinate frames are required')
    for name in ('vision_service','zone_service'):
        if cfg[name].get('mock_mode') is not False or cfg[name].get('enable_depth') is not True or cfg[name].get('calibrated') is not True:
            raise ValueError(f'{name} must use measured calibrated RGB-D, not mock data')
        if cfg[name].get('base_frame')!=worker.get('base_frame'):
            raise ValueError('Live nodes must use the same robot base frame')
    limits = worker.get('workspace',[])
    if len(limits)!=6 or not all(math.isfinite(x) for x in limits) or any(limits[i]>=limits[i+1] for i in (0,2,4)):
        raise ValueError('Measured workspace must have six finite, ordered limits')
    q = worker.get('tool_quaternion',[])
    if len(q)!=4 or not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1)>1e-3:
        raise ValueError('Measured tool_quaternion is required')
    roi = worker.get('source_roi',[])
    if len(roi)!=4 or not all(math.isfinite(x) for x in roi) or roi[0]>=roi[2] or roi[1]>=roi[3]:
        raise ValueError('Valid pixel source_roi is required')
    if not math.isfinite(worker.get('object_size',0)) or not 0<worker['object_size']<=.25:
        raise ValueError('Object footprint must be measured')
    if not worker.get('tool_link'):
        raise ValueError('A real TCP tool link is required')
    weights = Path(cfg['rgbd_detector'].get('weights',''))
    if not weights.is_file():
        raise ValueError(f'Local YOLO weights not found: {weights}')
    return document
