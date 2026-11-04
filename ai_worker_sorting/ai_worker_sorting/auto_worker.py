"""Discover two classes, freeze zone assignment, and sort using a shared grasp profile."""
import json
import math
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CameraInfo,Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from ai_worker_interfaces.msg import SortRule
from ai_worker_interfaces.srv import DetectObjects, ScanZones, PrepareAutoSort, StartTask, SessionStatus

from .worker_service import WorkerService, DelayedDryRobot
from .tasks import TaskCanceled
from .core import Point, in_workspace, pick_and_place
from .zones import AutoSession, grid_slots, inside, overlap, validate_quad


class AutoWorker(WorkerService):
    def __init__(self, parameter_overrides=None):
        super().__init__(parameter_overrides)
        defaults = dict(object_size=.06, slot_margin=.01, place_offset=.05,
                        auto_grasp_offset=0.0, shared_grasp_verified=False,
                        source_roi=[0.,0.,640.,360.], empty_scans=3, scan_interval=.25,
                        home_mode='configured', placement_height_mode='fixed',
                        min_object_height=.01, max_object_height=.25, transport_above_objects=False,
                        tool_below_tcp=0.,class_grasp_offsets='{}',check_camera_footprint=False,
                        camera_info_topic='/worker_camera/camera_info',geometry_depth_topic='/worker_camera/depth_rect')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
            self.cfg[key] = self.get_parameter(key).value
        roi = self.cfg['source_roi']
        if len(roi) != 4 or not all(math.isfinite(x) for x in roi) or roi[0] >= roi[2] or roi[1] >= roi[3]:
            raise ValueError('source_roi must be [left,top,right,bottom] in detection pixels')
        if self.cfg['empty_scans'] < 2 or not .05 <= self.cfg['scan_interval'] <= 5:
            raise ValueError('Invalid scan settings')
        self.auto_lock = threading.RLock()
        self.class_offsets = json.loads(self.cfg['class_grasp_offsets'])
        if not isinstance(self.class_offsets,dict) or any(not isinstance(k,str) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>=.5 for k,v in self.class_offsets.items()):
            raise ValueError('Invalid per-class grasp offsets')
        if self.cfg['check_camera_footprint'] and len(self.class_offsets)!=2:
            raise ValueError('Camera setup requires taught grasp offsets for both classes')
        self.geometry_info = None
        self.geometry_depths = deque(maxlen=60)
        def depth_callback(msg):
            with self.auto_lock:
                self.geometry_depths.append(msg)
        self.geometry_depth_sub = self.create_subscription(Image,self.cfg['geometry_depth_topic'],depth_callback,qos_profile_sensor_data)
        self.geometry_sub = self.create_subscription(CameraInfo,self.cfg['camera_info_topic'],
            lambda msg:setattr(self,'geometry_info',msg),qos_profile_sensor_data)
        self.robot_tf = Buffer()
        self.robot_tf_listener = TransformListener(self.robot_tf,self)
        self.preparing = False
        self.session = None  # One current session; preparing replaces only unused sessions.
        group = ReentrantCallbackGroup()
        self.all_vision = self.create_client(DetectObjects, '/vision/detect_objects', callback_group=group)
        self.zone_client = self.create_client(ScanZones, '/workspace/scan_zones', callback_group=group)
        self.create_service(PrepareAutoSort, '/worker/prepare_auto_sort', self.prepare,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(StartTask, '/worker/start_task', self.start_auto)
        self.create_service(SessionStatus, '/worker/session_status', self.session_status)

    def sort_once(self, request, response):
        response.message = 'Auto-zone server uses prepare_auto_sort/start_task; fixed-bin sort_once is disabled'
        return response

    def call(self, client, request, check=lambda: None):
        deadline = time.monotonic()+self.cfg['service_timeout']
        while not client.wait_for_service(timeout_sec=.05):
            check()
            if time.monotonic() >= deadline:
                raise TimeoutError('Required service unavailable')
        future = client.call_async(request)
        try:
            while not future.done():
                check()
                if time.monotonic() >= deadline:
                    raise TimeoutError('Required service response timeout')
                time.sleep(.01)
            check()
            result = future.result()
            if not result.success:
                raise ValueError(result.message)
            return result
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def source_objects(self, objects, zones):
        roi = self.cfg['source_roi']
        output = []
        for obj in objects:
            if not math.isfinite(obj.confidence) or not self.cfg['min_confidence'] <= obj.confidence <= 1:
                continue
            c = obj.bbox.center.position
            if not (roi[0] <= c.x < roi[2] and roi[1] <= c.y < roi[3]):
                continue
            if any(inside((c.x,c.y), [(p.x,p.y) for p in z.pixel_boundary.points]) for z in zones):
                continue
            if obj.class_name:
                output.append(obj)
        return output

    def rules(self, session):
        return [SortRule(object_class=c, zone_id=z) for c, z in sorted(session.mapping.items())]

    def prepare(self, request, response):
        with self.auto_lock:
            if self.preparing or self.store.active is not None or self.store.faulted:
                response.message = 'Worker busy or faulted'
                return response
            if self.session is not None and self.session.started:
                response.message = 'Session already used; clear/re-scan workspace and restart auto server before replacing it'
                return response
            # A failed rescan must not leave old, unverified coordinates executable.
            self.session = None
            self.preparing = True
        try:
            zones = self.call(self.zone_client, ScanZones.Request())
            if len(zones.zones) != 2 or len({z.zone_id for z in zones.zones}) != 2:
                raise ValueError('Exactly two distinct zones required')
            polygons = [[(p.x,p.y) for p in z.pixel_boundary.points] for z in zones.zones]
            for polygon in polygons:
                validate_quad(polygon)
            if overlap(*polygons):
                raise ValueError('Overlapping zones')
            snapshot = self.call(self.all_vision, DetectObjects.Request(target_class=''))
            for obj in snapshot.objects:
                c = obj.bbox.center.position
                if obj.confidence >= self.cfg['min_confidence'] and any(inside((c.x,c.y), p) for p in polygons):
                    raise ValueError('Start with empty destination zones')
            objects = self.source_objects(snapshot.objects, zones.zones)
            classes = sorted({o.class_name for o in objects})
            if len(classes) != 2:
                raise ValueError(f'Exactly two confident source classes required; found {classes}')
            if self.class_offsets and set(classes)!=set(self.class_offsets):
                raise ValueError('Detected classes differ from the taught grasp classes; repeat setup')
            if self.cfg['check_camera_footprint']:
                table_heights = [p.z for z in zones.zones for p in z.boundary.polygon.points]
                if len(table_heights)!=8 or max(table_heights)-min(table_heights)>.015:
                    raise ValueError('Camera setup requires a common level tabletop')
                for candidate in objects:
                    self.check_footprint(candidate,snapshot.header,sum(table_heights)/8)
            ordered = sorted(zones.zones, key=lambda z: z.zone_id)
            mapping = dict(zip(classes, [z.zone_id for z in ordered]))
            slots = {}
            for zone in ordered:
                if zone.localized:
                    if zone.boundary.header.frame_id != self.cfg['base_frame']:
                        raise ValueError('Zone frame differs from robot base frame')
                    slots[zone.zone_id] = grid_slots(zone.zone_id,
                        [(p.x,p.y,p.z) for p in zone.boundary.polygon.points], self.cfg['object_size'],
                        self.cfg['slot_margin'], self.cfg['place_offset'])
            session = AutoSession(zones.workspace_id, mapping, ordered, slots)
            with self.auto_lock:
                self.session = session
            response.success, response.session_id, response.workspace_id = True, session.session_id, session.workspace_id
            response.rules = self.rules(session)
            response.message = 'Assignment frozen; 3D localization and grasp checks are still required before motion'
        except Exception as exc:
            response.message = str(exc)
        finally:
            with self.auto_lock:
                self.preparing = False
        return response

    def session_status(self, request, response):
        with self.auto_lock:
            session = self.session
            if session is None or request.session_id != session.session_id:
                response.message = 'Unknown session'
                return response
            response.found, response.workspace_id = True, session.workspace_id
            response.rules, response.placed_count = self.rules(session), session.placed
            response.slots_json = json.dumps({z: [{'slot_id':s.slot_id, 'state':s.state}
                for s in slots] for z,slots in session.slots.items()})
            response.message = 'Assignment remains fixed for this session'
        return response

    def start_auto(self, request, response):
        with self.auto_lock:
            session = self.session
            if self.preparing or session is None or request.session_id != session.session_id:
                response.message = 'Prepare a valid session first'
                return response
            if not all(z.localized and session.slots.get(z.zone_id) for z in session.zones):
                response.message = '2D preview only; measured 3D zones required'
                return response
            world = [[(p.x,p.y,p.z) for p in z.boundary.polygon.points] for z in session.zones]
            if overlap(*world) or not all(in_workspace(s.position, self.cfg['workspace'])
                                         for slots in session.slots.values() for s in slots):
                response.message = 'Overlapping or out-of-workspace placement zones'
                return response
            if not self.cfg['dry_run']:
                if not self.cfg['hardware_configured'] or not self.cfg['shared_grasp_verified']:
                    response.message = 'Hardware and shared fixed-grasp profile must be configured for both object types'
                    return response
                if any(z.simulated for z in session.zones):
                    response.message = 'Mock zones cannot be used for hardware motion'
                    return response
            try:
                task = self.store.admit('__automatic_two_class__')
            except RuntimeError as exc:
                response.message = str(exc)
                return response
            session.started = True
            self.thread = threading.Thread(target=self.run_auto, args=(task, session, request.max_objects), daemon=True)
            self.thread.start()
            response.accepted, response.task_id, response.message = True, task.task_id, 'Automatic sorting queued; poll task_status'
        return response

    def measured_points(self, objects, class_name=None):
        points = []
        for obj in objects:
            if class_name is not None and obj.class_name != class_name:
                continue
            try:
                point = self.select_target([obj], obj.class_name)
            except ValueError:
                continue
            if not obj.simulated:
                points.append((obj,point))
        return points

    def check_footprint(self,obj,header,table_z):
        if not self.cfg['check_camera_footprint']:
            return
        from .auto_geometry import footprint,depth_top
        info = self.geometry_info
        if info is None or info.header.frame_id!=header.frame_id:
            raise ValueError('No matching camera intrinsics for footprint validation')
        stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec
        info_stamp = info.header.stamp.sec*1_000_000_000+info.header.stamp.nanosec
        if abs(info_stamp-stamp)>1_000_000_000:
            raise ValueError('Camera intrinsics are stale')
        observed = Time.from_msg(header.stamp)
        deadline = time.monotonic()+.5
        while not self.robot_tf.can_transform(self.cfg['base_frame'],header.frame_id,observed) and time.monotonic()<deadline:
            time.sleep(.02)
        tf = self.robot_tf.lookup_transform(self.cfg['base_frame'],header.frame_id,observed).transform
        t,q,b = tf.translation,tf.rotation,obj.bbox
        with self.auto_lock:
            candidates = [m for m in self.geometry_depths if m.header.frame_id==header.frame_id and
                abs(m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec-stamp)<=60_000_000]
        if not candidates:
            raise ValueError('No synchronized full object depth for geometry check')
        depth = min(candidates,key=lambda m:abs(m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec-stamp))
        pixels = CvBridge().imgmsg_to_cv2(depth,desired_encoding='passthrough')
        if pixels.shape!=(info.height,info.width):
            raise ValueError('Depth/intrinsics dimensions differ')
        top = depth_top(pixels,depth.encoding,(b.center.position.x,b.center.position.y,b.size_x,b.size_y),
            (info.p[0],info.p[5],info.p[2],info.p[6]),(t.x,t.y,t.z),(q.x,q.y,q.z,q.w),table_z)
        if top-table_z>self.cfg['max_object_height']:
            raise ValueError('Object exceeds camera-calibrated maximum height; repeat setup')
        size = footprint((b.center.position.x,b.center.position.y,b.size_x,b.size_y),
            (info.p[0],info.p[5],info.p[2],info.p[6]),(t.x,t.y,t.z),(q.x,q.y,q.z,q.w),
            table_z,top,(info.width,info.height))
        if size>self.cfg['object_size']:
            raise ValueError('Object exceeds camera-calibrated footprint; repeat setup')

    def verify_placement(self, obj, slot, source, check):
        after = self.get_clock().now().nanoseconds
        limit = time.monotonic()+self.cfg['service_timeout']
        while time.monotonic() < limit:
            snapshot = self.call(self.all_vision, DetectObjects.Request(target_class=''), check)
            stamp = snapshot.header.stamp.sec*1_000_000_000+snapshot.header.stamp.nanosec
            if stamp > after:
                points = [p for _,p in self.measured_points(snapshot.objects, obj.class_name)]
                radius = self.cfg['object_size']/2+self.cfg['slot_margin']
                at_destination = any(math.hypot(p.x-slot.position.x,p.y-slot.position.y) <= radius for p in points)
                still_at_source = any(math.hypot(p.x-source.x,p.y-source.y) <= self.cfg['object_size']/2 for p in points)
                if at_destination and not still_at_source:
                    return
            check()
            time.sleep(.05)
        raise ValueError('PLACEMENT_UNVERIFIED: destination presence/source removal not confirmed')

    def verify_grasp_observation(self,obj,source,check):
        after = self.get_clock().now().nanoseconds
        limit = time.monotonic()+self.cfg['service_timeout']
        while time.monotonic()<limit:
            snapshot = self.call(self.all_vision,DetectObjects.Request(target_class=''),check)
            stamp = snapshot.header.stamp.sec*1_000_000_000+snapshot.header.stamp.nanosec
            if stamp>after:
                tf = self.robot_tf.lookup_transform(self.cfg['base_frame'],self.cfg['tool_link'],Time())
                tf_stamp = tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
                if not 0<=(self.get_clock().now().nanoseconds-tf_stamp)/1e9<=self.cfg['feedback_timeout']:
                    raise ValueError('TCP feedback is stale during grasp verification')
                t = tf.transform.translation
                points = [p for _,p in self.measured_points(snapshot.objects,obj.class_name)]
                source_present = any(math.dist((p.x,p.y,p.z),(source.x,source.y,source.z))
                                     <=self.cfg['object_size']/2 for p in points)
                for point in points:
                    lifted = point.z-source.z >= self.cfg['clearance']*.5
                    near_tcp = math.dist((point.x,point.y,point.z),(t.x,t.y,t.z))<=.12
                    same_column = math.hypot(point.x-source.x,point.y-source.y)<=self.cfg['object_size']/2
                    if lifted and near_tcp and same_column and not source_present:
                        return
            check()
            time.sleep(.05)
        raise ValueError('GRASP_UNVERIFIED: no fresh lifted object near the TCP')

    def run_auto(self, task, session, max_objects):
        deadline = time.monotonic()+self.cfg['task_timeout']
        check = lambda: self.check(task, deadline)
        outcome, message, reserved = 'FAILED', 'Task did not finish', None
        live_robot = None
        completed, empty, last_stamp, after_stamp = 0, 0, 0, 0
        home = Point(*self.cfg['home_position'])
        try:
            def report(stage):
                self.store.stage(task, 'ITEM_DONE' if stage == 'DONE' else stage)
                self.publish(task)
            while not max_objects or completed < max_objects:
                check()
                report('DETECT')
                snapshot = self.call(self.all_vision, DetectObjects.Request(target_class=''), check)
                stamp = snapshot.header.stamp.sec*1_000_000_000+snapshot.header.stamp.nanosec
                age = (self.get_clock().now().nanoseconds-stamp)/1e9
                if stamp <= 0 or not 0 <= age <= self.cfg['target_max_age']:
                    raise ValueError('Stale detection snapshot')
                if stamp <= max(last_stamp, after_stamp):
                    task.cancel.wait(self.cfg['scan_interval'])
                    continue
                last_stamp = stamp
                objects = self.source_objects(snapshot.objects, session.zones)
                objects = [o for o in objects if not (o.simulated and o.target_id in session.completed_mock_ids)]
                if any(o.class_name not in session.mapping for o in objects):
                    raise ValueError('NEW_CLASS: mapping remains unchanged; stop and reconfigure')
                if not objects:
                    empty += 1
                    if empty >= self.cfg['empty_scans']:
                        break
                    task.cancel.wait(self.cfg['scan_interval'])
                    continue
                empty = 0
                obj = max(objects, key=lambda o:o.confidence)
                if self.cfg['check_camera_footprint']:
                    # Bound all recognized obstacles, including ones not picked next.
                    table_z = sum(p.z for z in session.zones for p in z.boundary.polygon.points)/8
                    for candidate in snapshot.objects:
                        if candidate.localized and not candidate.simulated and candidate.class_name in session.mapping:
                            self.check_footprint(candidate,snapshot.header,table_z)
                target = self.select_target([obj], obj.class_name)
                grasp_offset = self.class_offsets.get(obj.class_name,self.cfg['auto_grasp_offset'])
                with self.auto_lock:
                    reserved = session.reserve(obj.class_name)
                    if self.cfg['placement_height_mode']=='object_surface':
                        zone = next(z for z in session.zones if z.zone_id==session.mapping[obj.class_name])
                        table_z = sum(p.z for p in zone.boundary.polygon.points)/4
                        height = target.z-table_z
                        if not self.cfg['min_object_height']<=height<=self.cfg['max_object_height']:
                            raise ValueError('Object surface height is inconsistent with the table')
                        release_z = table_z+height+grasp_offset
                        if release_z-self.cfg['tool_below_tcp']<=table_z+.005:
                            raise ValueError('TCP grasp offset would enter the tabletop')
                        reserved.position = Point(reserved.position.x,reserved.position.y,release_z)
                check()
                if self.cfg['dry_run']:
                    robot = DelayedDryRobot(self.get_logger().info, self.cfg['dry_step_delay'], check, task.cancel)
                else:
                    radius = self.cfg['object_size']/2+self.cfg['slot_margin']
                    if any(math.hypot(p.x-reserved.position.x,p.y-reserved.position.y) < 2*radius-1e-6
                           for _,p in self.measured_points(snapshot.objects)):
                        raise ValueError('UNEXPECTED_OCCUPANCY: reserved slot is not clear')
                    if live_robot is None:
                        from .robot_factory import make_robot
                        live_robot = make_robot(self,self.cfg,check,
                            lambda:self.verify_grasp_observation(obj,target,check))
                        if self.cfg['home_mode']=='current_tcp':
                            if not hasattr(live_robot,'capture_home'):
                                raise ValueError('current_tcp home requires a feedback-capable robot backend')
                            home = live_robot.capture_home()
                    # The callback must follow the current object, not the first one.
                    live_robot.verifier = lambda:self.verify_grasp_observation(obj,target,check)
                    robot = live_robot
                transport_z = None
                if self.cfg['transport_above_objects']:
                    transport_z = max(p.z for z in session.zones for p in z.boundary.polygon.points)
                    transport_z += self.cfg['max_object_height']+self.cfg['clearance']
                    held_height = height if self.cfg['placement_height_mode']=='object_surface' else self.cfg['max_object_height']
                    transport_z += max(self.cfg['tool_below_tcp'],held_height+grasp_offset,0.)
                pick_and_place(robot, target, reserved.position, home,
                               self.cfg['workspace'], self.cfg['clearance'], grasp_offset, check, report,
                               transport_z=transport_z)
                if live_robot is not None:
                    report('VERIFY_PLACEMENT')
                    self.verify_placement(obj,reserved,target,check)
                with self.auto_lock:
                    reserved.state = 'OCCUPIED'
                    session.placed += 1
                    if obj.simulated:
                        session.completed_mock_ids.add(obj.target_id)
                reserved = None
                completed += 1
                after_stamp = self.get_clock().now().nanoseconds
            check()
            prefix = 'DRY: ' if self.cfg['dry_run'] else ''
            outcome, message = 'SUCCEEDED', f'{prefix}sorted {completed} objects; {session.placed} in session'
        except TaskCanceled as exc:
            outcome, message = ('FAILED','Controller cancellation unconfirmed; inspect hardware') if live_robot else ('CANCELED',str(exc))
        except Exception as exc:
            outcome, message = 'FAILED', str(exc)
        finally:
            fault = live_robot is not None and outcome != 'SUCCEEDED'
            if live_robot is not None:
                try:
                    live_robot.destroy()
                except Exception as exc:
                    outcome, message, fault = 'FAILED',f'Robot cleanup failed: {exc}',True
                if task.cancel.is_set():
                    outcome, message, fault = 'FAILED','Cancellation; inspect controller state',True
            if reserved is not None:
                with self.auto_lock:
                    reserved.state = 'UNKNOWN'
            self.store.finish(task, outcome, message, fault=fault)
            self.publish(task)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = AutoWorker()
        if node.cfg['stop_file'] and Path(node.cfg['stop_file']).exists():
            raise ValueError('Auto-worker stop file exists')
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=.2)
            if node.cfg['stop_file'] and Path(node.cfg['stop_file']).exists():
                break
    finally:
        if node:
            node.shutdown_event.set()
            if node.thread:
                node.thread.join(timeout=3)
        executor.shutdown()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()
