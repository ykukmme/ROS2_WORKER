"""Camera scan + one-time pose teaching, without entering object/gripper dimensions.

No motion commands are published. An operator positions the robot with its
existing jog controls; this tool only records fresh feedback.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from collections import deque
import yaml
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo,Image
from cv_bridge import CvBridge
from tf2_ros import Buffer,TransformListener
from ai_worker_interfaces.srv import DetectObjects,ScanZones
from .auto_geometry import footprint,teaching_offset,depth_top
from .configure_live import numbers
from .controller_contract import verify_controller
from .zones import inside


def service(node,kind,name):
    client = node.create_client(kind,name)
    try:
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Start camera-only calibration launch first: '+name)
        future = client.call_async(kind.Request())
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError('Calibration observation unavailable: '+name)
        return future.result()
    finally:
        node.destroy_client(client)


def fresh(node,header,age=1.):
    stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec
    if stamp<=0 or not 0<=(node.get_clock().now().nanoseconds-stamp)/1e9<=age:
        raise ValueError('Observation/TF is stale; check synchronized clocks')


def pose(node,buffer,base,link):
    deadline = time.monotonic()+5
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        try:
            tf = buffer.lookup_transform(base,link,Time())
            fresh(node,tf.header,.5)
            t,q = tf.transform.translation,tf.transform.rotation
            p,orientation = [t.x,t.y,t.z],[q.x,q.y,q.z,q.w]
            if not all(math.isfinite(x) for x in p+orientation) or abs(sum(x*x for x in orientation)-1)>.001:
                raise ValueError('Invalid robot pose')
            return p,orientation
        except Exception:
            pass
    raise RuntimeError('Fresh TCP feedback unavailable')


def scan(node,buffer,args,info,depths):
    zones = service(node,ScanZones,'/workspace/scan_zones')
    if len(zones.zones)!=2 or any(z.simulated or not z.localized or z.boundary.header.frame_id!=args.base_frame for z in zones.zones):
        raise ValueError('Two real localized tape zones are required')
    for zone in zones.zones:
        fresh(node,zone.boundary.header)
    heights = [p.z for z in zones.zones for p in z.boundary.polygon.points]
    if len(heights)!=8 or max(heights)-min(heights)>.015:
        raise ValueError('Both tape zones must lie on the same level tabletop')
    table = sum(heights)/len(heights)
    objects = service(node,DetectObjects,'/vision/detect_objects')
    fresh(node,objects.header)
    if info is None or info.header.frame_id!=objects.header.frame_id:
        raise ValueError('Matching rectified camera intrinsics unavailable')
    fresh(node,info.header)
    stamp = Time.from_msg(objects.header.stamp)
    deadline = time.monotonic()+1
    while not buffer.can_transform(args.base_frame,objects.header.frame_id,stamp) and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
    tf = buffer.lookup_transform(args.base_frame,objects.header.frame_id,stamp).transform
    t,q = tf.translation,tf.rotation
    candidates = [m for m in list(depths) if m.header.frame_id==objects.header.frame_id and
        abs((m.header.stamp.sec-objects.header.stamp.sec)+(m.header.stamp.nanosec-objects.header.stamp.nanosec)/1e9)<=.06]
    if not candidates:
        raise ValueError('No synchronized complete depth image for maximum object height')
    depth = min(candidates,key=lambda m:abs((m.header.stamp.sec-objects.header.stamp.sec)*1e9+m.header.stamp.nanosec-objects.header.stamp.nanosec))
    fresh(node,depth.header)
    pixels = CvBridge().imgmsg_to_cv2(depth,desired_encoding='passthrough')
    if pixels.shape!=(info.height,info.width):
        raise ValueError('Depth and intrinsics dimensions differ')
    polys = [[(p.x,p.y) for p in z.pixel_boundary.points] for z in zones.zones]
    rows = []
    for obj in objects.objects:
        b = obj.bbox
        u,v = b.center.position.x,b.center.position.y
        if not args.source_roi[0]<=u<=args.source_roi[2] or not args.source_roi[1]<=v<=args.source_roi[3]:
            continue
        if any(inside((u,v),poly) for poly in polys):
            raise ValueError('Destination tape zones must be empty during setup')
        if obj.simulated or not obj.localized or obj.confidence<.8 or obj.surface_pose.header.frame_id!=args.base_frame:
            raise ValueError('All setup objects must have confident real depth localization')
        fresh(node,obj.surface_pose.header)
        p = obj.surface_pose.pose.position
        top = depth_top(pixels,depth.encoding,(u,v,b.size_x,b.size_y),
            (info.p[0],info.p[5],info.p[2],info.p[6]),(t.x,t.y,t.z),(q.x,q.y,q.z,q.w),table)
        size = footprint((u,v,b.size_x,b.size_y),(info.p[0],info.p[5],info.p[2],info.p[6]),
            (t.x,t.y,t.z),(q.x,q.y,q.z,q.w),table,top,(info.width,info.height))
        if not .01<=p.z-table<=.4:
            raise ValueError('Unsupported object height')
        rows.append(dict(name=obj.class_name,point=[p.x,p.y,p.z],size=size,top=top,
                         box=[u-b.size_x/2,v-b.size_y/2,u+b.size_x/2,v+b.size_y/2]))
    classes = sorted({o['name'] for o in rows})
    if len(classes)!=2:
        raise ValueError('Show exactly two isolated, completely visible YOLO object classes')
    for i,a in enumerate(rows):
        for b in rows[i+1:]:
            x,y = a['box'],b['box']
            if x[0]<y[2] and y[0]<x[2] and x[1]<y[3] and y[1]<x[3]:
                raise ValueError('Object boxes overlap; separate objects before setup')
    size = max(o['size'] for o in rows)*1.2+.01
    if size>.25:
        raise ValueError('Observed objects exceed supported footprint with margin')
    return dict(version=1,created=time.time(),base_frame=args.base_frame,controller_node=args.controller_node,
        controller_frame=args.controller_frame,tool_link=args.tool_link,workspace=args.workspace,
        weights=str(args.weights.resolve()),device=args.device,source_roi=args.source_roi,
        table_z=table,object_size=size,max_object_height=max(o['top']-table for o in rows)+.03,
        samples={name:next(o['point'] for o in rows if o['name']==name) for name in classes},
        class_offsets={},tool_below_tcp=None,
        movel_topic=args.movel_topic,gripper_trajectory_topic=args.gripper_topic,
        gripper_joint=args.gripper_joint,joint_states_topic=args.joint_states_topic,
        color_topic=args.color_topic,depth_topic=args.depth_topic,camera_info_topic=args.camera_info_topic)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=['scan','tool','grasp','write'])
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--weights',type=Path)
    parser.add_argument('--workspace',type=numbers)
    parser.add_argument('--source-roi',type=numbers)
    parser.add_argument('--base-frame',default='base_link')
    parser.add_argument('--controller-frame',default='base_link')
    parser.add_argument('--controller-node',default='/ai_worker_movel_controller')
    parser.add_argument('--movel-topic',default='/l_goal_move')
    parser.add_argument('--gripper-topic',default='/leader/joint_trajectory_command_broadcaster_left/raw_joint_trajectory')
    parser.add_argument('--gripper-joint',default='gripper_l_joint1')
    parser.add_argument('--joint-states-topic',default='/joint_states')
    parser.add_argument('--color-topic',default='/camera/color/image_raw')
    parser.add_argument('--depth-topic',default='/camera/aligned_depth_to_color/image_raw')
    parser.add_argument('--camera-info-topic',default='/camera/color/camera_info')
    parser.add_argument('--device',default='cpu')
    parser.add_argument('--class-name')
    parser.add_argument('--tips-at-table',action='store_true')
    parser.add_argument('--grasp-pose-confirmed',action='store_true')
    args = parser.parse_args()
    if args.stage=='scan' and (args.weights is None or not args.weights.is_file() or args.workspace is None):
        parser.error('scan requires local --weights and robot --workspace')
    if args.stage=='scan' and args.state.exists():
        parser.error('State exists; choose a new --state path for a new setup')
    if args.stage!='scan':
        state = yaml.safe_load(args.state.read_text(encoding='utf-8'))
        if state.get('version')!=1 or not 0<=time.time()-state['created']<=1800:
            parser.error('Setup expired; scan again with a new state path')
    if args.stage=='write':
        if args.output is None or state['tool_below_tcp'] is None or set(state['class_offsets'])!=set(state['samples']):
            parser.error('Record tool reference and grasp poses for both classes before write')
        # Existing live configuration tool verifies current hardware and writes atomically.
        command = [sys.executable,'-m','ai_worker_sorting.configure_live','--output',str(args.output),
            '--weights',state['weights'],'--workspace='+','.join(map(str,state['workspace'])),
            '--object-size',str(state['object_size']),'--max-object-height',str(state['max_object_height']),
            '--grasp-offset',str(next(iter(state['class_offsets'].values()))),
            '--tool-below-tcp',str(state['tool_below_tcp']),'--class-grasp-offsets',json.dumps(state['class_offsets']),
            '--check-camera-footprint','--base-frame',state['base_frame'],'--controller-frame',state['controller_frame'],
            '--controller-node',state['controller_node'],'--tool-link',state['tool_link'],
            '--device',state['device'],'--source-roi',','.join(map(str,state['source_roi']))]
        command += ['--expected-tool-quaternion='+','.join(map(str,state['orientation']))]
        command += ['--movel-topic',state['movel_topic'],'--gripper-topic',state['gripper_trajectory_topic'],
                    '--joint-states-topic',state['joint_states_topic'],'--gripper-joint',state['gripper_joint']]
        command += ['--color-topic',state['color_topic'],'--depth-topic',state['depth_topic'],
                    '--camera-info-topic',state['camera_info_topic']]
        return subprocess.run(command,check=False).returncode
    if args.stage=='tool' and not args.tips_at_table:
        parser.error('Position the lowest tool part at tabletop height, then add --tips-at-table')
    if args.stage=='grasp' and (not args.grasp_pose_confirmed or args.class_name not in state['samples']):
        parser.error('Select a scanned --class-name and confirm its taught grasp with --grasp-pose-confirmed')
    rclpy.init(args=[])
    node = Node('configure_auto_readonly')
    buffer = Buffer()
    listener = TransformListener(buffer,node)
    data = {}
    depths = deque(maxlen=60)
    sub = node.create_subscription(CameraInfo,'/worker_camera/camera_info',lambda m:data.update(info=m),qos_profile_sensor_data)
    depth_sub = node.create_subscription(Image,'/worker_camera/depth_rect',depths.append,qos_profile_sensor_data)
    try:
        if args.stage=='scan':
            cfg = dict(base_frame=args.base_frame,controller_node=args.controller_node,tool_link='',
                movel_topic=args.movel_topic,gripper_trajectory_topic=args.gripper_topic,
                gripper_joint=args.gripper_joint,joint_states_topic=args.joint_states_topic)
        else:
            cfg = state
        args.tool_link = verify_controller(node,cfg,spin=True)
        position,orientation = pose(node,buffer,cfg['base_frame'],args.tool_link)
        if any(name=='/worker/start_task' for name,_ in node.get_service_names_and_types()):
            raise RuntimeError('Stop the sorting worker before pose teaching; use only the camera calibration stack')
        if args.stage=='scan':
            deadline = time.monotonic()+5
            while 'info' not in data and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.05)
            if 'info' not in data:
                raise ValueError('Rectified camera info unavailable')
            info = data['info']
            args.source_roi = args.source_roi or [0.,0.,float(info.width),float(info.height)]
            if len(args.source_roi)!=4 or not 0<=args.source_roi[0]<args.source_roi[2]<=info.width or not 0<=args.source_roi[1]<args.source_roi[3]<=info.height:
                raise ValueError('Invalid source ROI')
            state = scan(node,buffer,args,info,depths)
        elif args.stage=='tool':
            extent = position[2]-state['table_z']+.003
            if not 0<=extent<=.5:
                raise ValueError('Tool reference outside supported bounds')
            if state.get('orientation') and abs(sum(a*b for a,b in zip(orientation,state['orientation'])))<.999:
                raise ValueError('Keep the same downward orientation for all teaching stages')
            state['tool_below_tcp'] = max(state['tool_below_tcp'] or 0.,extent)
            state['orientation'] = orientation
            state['class_offsets'] = {}  # A changed tool reference requires renewed grasp teaching.
        else:
            if state['tool_below_tcp'] is None:
                raise ValueError('Record tool reference first')
            if abs(sum(a*b for a,b in zip(orientation,state['orientation'])))<.999:
                raise ValueError('Keep the taught downward tool orientation')
            state['class_offsets'][args.class_name] = teaching_offset(state['samples'][args.class_name],position,
                state['table_z'],state['tool_below_tcp'])
        args.state.parent.mkdir(parents=True,exist_ok=True)
        args.state.write_text(yaml.safe_dump(state,sort_keys=False),encoding='utf-8')
        print(json.dumps(dict(stage=args.stage,classes=list(state['samples']),
            taught_classes=list(state['class_offsets']),motion_commands_sent=False),indent=2))
        return 0
    finally:
        node.destroy_subscription(sub)
        node.destroy_subscription(depth_sub)
        listener.unregister()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    sys.exit(main())
