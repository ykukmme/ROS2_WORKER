"""Connected-PC diagnostics and explicit ROS2 sorting service commands."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict


def call(node, kind, endpoint, request, timeout):
    client = node.create_client(kind,endpoint)
    deadline = time.monotonic()+timeout
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'Service not visible: {endpoint}; check robot bringup and ROS2 network')
        future = client.call_async(request)
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if not future.done() or future.result() is None:
            raise TimeoutError(f'Service response timed out: {endpoint}; do not retry start automatically')
        result = future.result()
        print(json.dumps(message_to_ordereddict(result),ensure_ascii=False,indent=2))
        return next((getattr(result,k) for k in ('success','accepted','found') if hasattr(result,k)),True)
    finally:
        node.destroy_client(client)


def doctor(node, config, timeout):
    from sensor_msgs.msg import Image,CameraInfo,JointState
    from tf2_ros import Buffer,TransformListener
    from .controller_contract import verify_controller
    from .live_config import validate_live_config
    document = validate_live_config(config)
    worker = document['worker_service']['ros__parameters']
    camera = document['rgbd_input']['ros__parameters']
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    data,subscriptions = {},[]
    topics = [('color',Image,camera.get('color_topic','/camera/color/image_raw')),
              ('depth',Image,camera.get('depth_topic','/camera/aligned_depth_to_color/image_raw')),
              ('info',CameraInfo,camera.get('camera_info_topic','/camera/color/camera_info')),
              ('joints',JointState,worker.get('joint_states_topic','/joint_states'))]
    for key,kind,topic in topics:
        subscriptions.append(node.create_subscription(kind,topic,lambda msg,k=key:data.update({k:msg}),qos_profile_sensor_data))
    cfg = dict(controller_node='/ai_worker_movel_controller',movel_topic='/l_goal_move',
        gripper_trajectory_topic='/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory',
        gripper_joint='gripper_l_joint1',joint_states_topic='/joint_states')
    cfg.update(worker)
    try:
        verify_controller(node,cfg,spin=True)
        deadline = time.monotonic()+timeout
        error = 'Waiting for RGB, aligned depth, CameraInfo and joint feedback'
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if len(data)!=4:
                continue
            try:
                now = node.get_clock().now().nanoseconds
                for key,msg in data.items():
                    stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
                    if stamp<=0 or not 0<=(now-stamp)/1e9<=1.:
                        raise ValueError(f'{key} stale or clocks differ; synchronize PC/robot clocks')
                rgb,depth,info = data['color'],data['depth'],data['info']
                if len({(m.width,m.height) for m in (rgb,depth,info)})!=1 or not rgb.width:
                    raise ValueError('RGB/depth/intrinsics dimensions differ')
                if len({m.header.frame_id for m in (rgb,depth,info)})!=1:
                    raise ValueError('Depth must be aligned to the color optical frame')
                if depth.encoding not in ('16UC1','32FC1') or info.k[0]<=0 or info.k[4]<=0:
                    raise ValueError('Invalid depth encoding or camera intrinsics')
                if cfg['gripper_joint'] not in data['joints'].name:
                    raise ValueError('Configured gripper feedback missing')
                for frame in (info.header.frame_id,cfg['tool_link'],cfg['controller_frame']):
                    tf = buffer.lookup_transform(cfg['base_frame'],frame,Time())
                    stamp = tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
                    if stamp and not 0<=(now-stamp)/1e9<=1.:
                        raise ValueError(f'Stale TF: {frame}')
                print(json.dumps(dict(result='PASS',motion_commands_sent=False,
                    ros_distro=os.environ.get('ROS_DISTRO',''),ros_domain_id=os.environ.get('ROS_DOMAIN_ID','0'),
                    rmw=os.environ.get('RMW_IMPLEMENTATION','default'),config=str(config.resolve()),
                    topics=dict((k,t) for k,_,t in topics)),indent=2))
                return True
            except Exception as exc:
                error = str(exc)
        raise RuntimeError(error+'; verify ROS_DOMAIN_ID, middleware, routes and timestamps')
    finally:
        for sub in subscriptions:
            node.destroy_subscription(sub)
        listener.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command',required=True)
    for command in ('doctor','prepare','start','status','cancel','session'):
        item = sub.add_parser(command)
        item.add_argument('--timeout',type=float,default=15.)
        if command=='doctor':
            item.add_argument('--config',type=Path,required=True)
        if command in ('start','session'):
            item.add_argument('--session-id',required=True)
        if command in ('status','cancel'):
            item.add_argument('--task-id',required=True)
        if command=='start':
            item.add_argument('--max-objects',type=int,default=0)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout<=0:
        parser.error('timeout must be positive')
    if args.command=='start' and not 0<=args.max_objects<=4294967295:
        parser.error('max-objects must be a uint32')
    rclpy.init(args=[])
    node = Node('worker_pc_cli')
    try:
        if args.command=='doctor':
            ok = doctor(node,args.config,args.timeout)
        else:
            from ai_worker_interfaces.srv import PrepareAutoSort,StartTask,TaskStatus,CancelTask,SessionStatus
            specs = dict(prepare=(PrepareAutoSort,'prepare_auto_sort',{}),
                start=(StartTask,'start_task',dict(session_id=getattr(args,'session_id',''),max_objects=getattr(args,'max_objects',0))),
                status=(TaskStatus,'task_status',dict(task_id=getattr(args,'task_id',''))),
                cancel=(CancelTask,'cancel_task',dict(task_id=getattr(args,'task_id',''))),
                session=(SessionStatus,'session_status',dict(session_id=getattr(args,'session_id',''))))
            kind,endpoint,values = specs[args.command]
            ok = call(node,kind,'/worker/'+endpoint,kind.Request(**values),args.timeout)
        return 0 if ok else 1
    except Exception as exc:
        print(str(exc),file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    sys.exit(main())
