"""Read existing robot feedback and write an executable local live profile. No motion."""
import argparse
import math
from pathlib import Path
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, JointState
from tf2_ros import Buffer, TransformListener

from .live_config import validate_live_config


def numbers(value):
    return [float(x) for x in value.split(',')]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--weights',required=True,type=Path)
    parser.add_argument('--workspace',required=True,type=numbers,help='xmin,xmax,ymin,ymax,zmin,zmax in robot base metres')
    parser.add_argument('--object-size',required=True,type=float,help='Maximum footprint of both objects, metres')
    parser.add_argument('--grasp-offset',required=True,type=float,help='TCP Z offset from visible object TOP surface, metres')
    parser.add_argument('--tool-below-tcp',required=True,type=float,help='Maximum downward tool extent below the controlled link, metres')
    parser.add_argument('--max-object-height',type=float,default=.25,help='Upper bound for both object heights, metres')
    parser.add_argument('--source-roi',type=numbers,help='left,top,right,bottom in camera pixels; default full image')
    parser.add_argument('--color-topic',default='/camera/color/image_raw')
    parser.add_argument('--depth-topic',default='/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--camera-info-topic',default='/camera/color/camera_info')
    parser.add_argument('--base-frame',default='base_link')
    parser.add_argument('--controller-frame',default='base_link')
    parser.add_argument('--tool-link',default='',help='Must match actual l_gripper_name; default discover from controller')
    parser.add_argument('--controller-node',default='/ai_worker_movel_controller')
    parser.add_argument('--joint-states-topic',default='/joint_states')
    parser.add_argument('--movel-topic',default='/l_goal_move')
    parser.add_argument('--gripper-topic',default='/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory')
    parser.add_argument('--gripper-joint',default='gripper_l_joint1')
    parser.add_argument('--device',default='0')
    parser.add_argument('--input-is-rectified',action='store_true')
    args = parser.parse_args()
    if not args.weights.is_file() or not math.isfinite(args.grasp_offset):
        parser.error('Local weights and a finite measured grasp offset are required')
    if args.output.exists():
        parser.error('Output already exists; choose a new profile path')
    rclpy.init(args=[])
    node = Node('configure_ai_worker_live')
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    data = {}
    info_sub = node.create_subscription(CameraInfo,args.camera_info_topic,lambda m:data.update(info=m),qos_profile_sensor_data)
    joint_sub = node.create_subscription(JointState,args.joint_states_topic,lambda m:data.update(joints=m),qos_profile_sensor_data)
    try:
        from .controller_contract import verify_controller
        args.tool_link = verify_controller(node,dict(controller_node=args.controller_node,
            tool_link=args.tool_link,movel_topic=args.movel_topic,gripper_trajectory_topic=args.gripper_topic,
            gripper_joint=args.gripper_joint,joint_states_topic=args.joint_states_topic),spin=True)
        deadline = time.monotonic()+10
        transform = None
        ready = False
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if 'info' not in data or 'joints' not in data:
                continue
            try:
                transform = buffer.lookup_transform(args.base_frame,args.tool_link,Time())
                buffer.lookup_transform(args.base_frame,data['info'].header.frame_id,Time())
                if args.controller_frame!=args.base_frame:
                    buffer.lookup_transform(args.controller_frame,args.base_frame,Time())
                now = node.get_clock().now().nanoseconds
                for header in (transform.header,data['joints'].header):
                    stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec
                    if stamp<=0 or not 0<=(now-stamp)/1e9<=.5:
                        raise RuntimeError('Robot feedback is stale')
                ready = True
                break
            except Exception:
                pass
        if not ready:
            raise RuntimeError('No real camera intrinsics, joints or TCP/camera TF; start robot bringup first')
        joints = data['joints']
        if args.gripper_joint not in joints.name:
            raise RuntimeError('Left gripper joint not found in feedback')
        movel_types = [x.topic_type for x in node.get_subscriptions_info_by_topic(args.movel_topic)]
        gripper_types = [x.topic_type for x in node.get_subscriptions_info_by_topic(args.gripper_topic)]
        if 'robotis_interfaces/msg/MoveL' not in movel_types or 'trajectory_msgs/msg/JointTrajectory' not in gripper_types:
            raise RuntimeError('Cyclo MoveL/raw gripper subscribers not found; start the official controller first')
        q = transform.transform.rotation
        quaternion = [q.x,q.y,q.z,q.w]
        norm = math.sqrt(sum(x*x for x in quaternion))
        if not math.isfinite(norm) or norm<1e-8:
            raise RuntimeError('Invalid measured TCP orientation')
        roi = args.source_roi or [0.,0.,float(data['info'].width),float(data['info'].height)]
        if len(roi)!=4 or not (0<=roi[0]<roi[2]<=data['info'].width and 0<=roi[1]<roi[3]<=data['info'].height):
            raise ValueError('Source ROI is outside the actual camera image')
        common = dict(mock_mode=False,enable_depth=True,calibrated=True,base_frame=args.base_frame,
                      detections_topic='/worker_camera/detections',depth_topic='/worker_camera/depth_rect',
                      camera_info_topic='/worker_camera/camera_info',max_age=1.,stop_file='')
        document = {
            'rgbd_input':{'ros__parameters':dict(color_topic=args.color_topic,depth_topic=args.depth_topic,
                camera_info_topic=args.camera_info_topic,input_is_rectified=args.input_is_rectified)},
            'rgbd_detector':{'ros__parameters':dict(weights=str(args.weights.resolve()),device=args.device,
                confidence=.8,image_topic='/worker_camera/image_rect')},
            'vision_service':{'ros__parameters':dict(common)},
            'zone_service':{'ros__parameters':dict(common,image_topic='/worker_camera/image_rect',zone_colors=['red','blue'])},
            'worker_service':{'ros__parameters':dict(dry_run=False,hardware_configured=True,shared_grasp_verified=True,
                robot_backend='robotis_movel',base_frame=args.base_frame,tool_link=args.tool_link,
                controller_frame=args.controller_frame,
                controller_node=args.controller_node,tool_below_tcp=args.tool_below_tcp,
                movel_topic=args.movel_topic,gripper_trajectory_topic=args.gripper_topic,
                joint_states_topic=args.joint_states_topic,gripper_joint=args.gripper_joint,
                gripper_open=0.,gripper_closed=.7,tool_quaternion=[x/norm for x in quaternion],
                workspace=args.workspace,source_roi=roi,object_size=args.object_size,
                auto_grasp_offset=args.grasp_offset,placement_height_mode='object_surface',
                home_mode='current_tcp',clearance=.10,transport_above_objects=True,
                max_object_height=args.max_object_height,stop_file='')},
        }
        args.output.parent.mkdir(parents=True,exist_ok=True)
        # Validate a temporary sibling first; a failed profile is never installed.
        temporary = args.output.with_name(args.output.name+'.pending')
        created = False
        try:
            with temporary.open('x',encoding='utf-8') as stream:
                created = True
                yaml.safe_dump(document,stream,sort_keys=False)
            validate_live_config(temporary)
            temporary.replace(args.output)
        finally:
            if created:
                temporary.unlink(missing_ok=True)
        print(f'LIVE profile saved: {args.output}; no commands were sent')
        print('The current measured TCP orientation is the fixed grasp orientation in this profile.')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
