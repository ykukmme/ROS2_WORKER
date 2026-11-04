"""Exercise live code with official messages and emulated sensors/controller.

No physical robot is contacted. All controller topics use /live_test/*.
The worker runs dry_run=False and publishes actual robotis_interfaces/MoveL.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState,CameraInfo,Image
from trajectory_msgs.msg import JointTrajectory
from robotis_interfaces.msg import MoveL
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
from ai_worker_interfaces.msg import DetectedObject
from ai_worker_interfaces.srv import DetectObjects,PrepareAutoSort,StartTask,TaskStatus,SessionStatus

from ai_worker_sorting.auto_worker import AutoWorker
from ai_worker_sorting.zone_service import ZoneService
from ai_worker_sorting.rgbd_input import AlignedRGBDInput
from ai_worker_sorting.rgbd_detector import RGBDDetector
from ai_worker_sorting.robotis_robot import RobotisMoveLRobot
from ai_worker_sorting.core import Point


def until(predicate,message,timeout=12):
    deadline = time.monotonic()+timeout
    while time.monotonic()<deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    raise AssertionError(message)


class ControllerFixture(Node):
    def __init__(self):
        super().__init__('live_test_controller_fixture')
        for name,value in dict(l_gripper_name='live_test_tcp',left_movel_topic='/live_test/movel',
            left_raw_traj_topic='/live_test/gripper',left_gripper_joint='gripper_l_joint1',
            joint_states_topic='/live_test/joints').items():
            self.declare_parameter(name,value)
        self.lock = threading.RLock()
        self.pose,self.quaternion = Point(.3,0.,.4),[1.,0.,0.,0.]
        self.gripper,self.held = 0.,None
        self.moves,self.grips = [],[]
        self.freeze = False
        self.joint_feedback = True
        self.objects = [dict(name='can',point=Point(.2,-.05,.15),pixel=(100.,100.),placed=False),
                        dict(name='cup',point=Point(.27,.05,.15),pixel=(180.,190.),placed=False)]
        self.tf = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.joints = self.create_publisher(JointState,'/live_test/joints',10)
        self.info = self.create_publisher(CameraInfo,'/live_test/info',10)
        self.rgb = self.create_publisher(Image,'/live_test/color',10)
        self.depth = self.create_publisher(Image,'/live_test/depth',10)
        self.move_sub = self.create_subscription(MoveL,'/live_test/movel',self.move,10)
        self.grip_sub = self.create_subscription(JointTrajectory,'/live_test/gripper',self.grip,10)
        self.vision = self.create_service(DetectObjects,'/vision/detect_objects',self.detect)
        self.timer = self.create_timer(.03,self.tick)

    def move(self,msg):
        assert msg.pose.header.frame_id=='live_test_base'
        with self.lock:
            self.moves.append(msg)
            if not self.freeze:
                p,q = msg.pose.pose.position,msg.pose.pose.orientation
                self.pose,self.quaternion = Point(p.x,p.y,p.z),[q.x,q.y,q.z,q.w]
                if self.held is not None:
                    self.objects[self.held]['point'] = self.pose.above(.02)

    def grip(self,msg):
        assert msg.joint_names==['gripper_l_joint1']
        with self.lock:
            self.grips.append(msg)
            self.gripper = msg.points[0].positions[0]
            if self.gripper>.6 and self.held is None:
                for i,obj in enumerate(self.objects):
                    if not obj['placed'] and np.linalg.norm(np.array(list(vars(obj['point']).values()))-
                                                           np.array(list(vars(self.pose).values())))<.06:
                        self.held = i
                        break
            elif self.gripper<.05 and self.held is not None:
                obj = self.objects[self.held]
                obj.update(point=self.pose.above(.02),pixel=(500.,90. if self.pose.y<0 else 270.),placed=True)
                self.held = None

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        with self.lock:
            pose,q,gripper = self.pose,list(self.quaternion),self.gripper
        tcp = TransformStamped()
        tcp.header.stamp,tcp.header.frame_id,tcp.child_frame_id = stamp,'live_test_base','live_test_tcp'
        tcp.transform.translation.x,tcp.transform.translation.y,tcp.transform.translation.z = pose.x,pose.y,pose.z
        tcp.transform.rotation.x,tcp.transform.rotation.y,tcp.transform.rotation.z,tcp.transform.rotation.w = q
        camera = TransformStamped()
        camera.header.stamp,camera.header.frame_id,camera.child_frame_id = stamp,'live_test_base','live_test_camera'
        camera.transform.translation.x,camera.transform.translation.z = .4,1.
        camera.transform.rotation.x,camera.transform.rotation.w = 1.,0.
        self.tf.sendTransform([tcp,camera])
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [f'arm_l_joint{i}' for i in range(1,8)]+['gripper_l_joint1']
        joints.position = [0.]*7+[gripper]
        if self.joint_feedback:
            self.joints.publish(joints)
        info = CameraInfo()
        info.header.stamp,info.header.frame_id = stamp,'live_test_camera'
        info.width,info.height = 640,480
        info.k = [500.,0.,320.,0.,500.,240.,0.,0.,1.]
        info.r = [1.,0.,0.,0.,1.,0.,0.,0.,1.]
        info.p = [500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.]
        info.d,info.distortion_model = [0.]*5,'plumb_bob'
        self.info.publish(info)
        color = self.bridge.cv2_to_imgmsg(np.full((480,640,3),180,dtype=np.uint8),encoding='bgr8')
        depth = self.bridge.cv2_to_imgmsg(np.full((480,640),900,dtype=np.uint16),encoding='16UC1')
        color.header = depth.header = info.header
        self.rgb.publish(color)
        self.depth.publish(depth)

    def detect(self,request,response):
        response.success = True
        response.header.stamp,response.header.frame_id = self.get_clock().now().to_msg(),'live_test_camera'
        with self.lock:
            for i,data in enumerate(self.objects):
                if request.target_class and request.target_class!=data['name']:
                    continue
                obj = DetectedObject(class_name=data['name'],confidence=.99,localized=True,simulated=False)
                obj.target_id = f'fixture-{i}'
                obj.surface_pose.header.stamp,obj.surface_pose.header.frame_id = response.header.stamp,'live_test_base'
                p = data['point']
                obj.surface_pose.pose.position.x,obj.surface_pose.pose.position.y,obj.surface_pose.pose.position.z = p.x,p.y,p.z
                obj.bbox.center.position.x,obj.bbox.center.position.y = data['pixel']
                obj.bbox.size_x,obj.bbox.size_y = 40.,40.
                response.objects.append(obj)
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--weights',required=True,type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    fixture = ControllerFixture()
    zones = ZoneService(parameter_overrides=[Parameter('mock_mode',value=True),Parameter('base_frame',value='live_test_base')])
    original_zones = zones.mock_zones
    def measured_fixture_zones():
        result = original_zones()
        for zone in result:
            zone.simulated = False
        return result
    zones.mock_zones = measured_fixture_zones
    values = dict(dry_run=False,hardware_configured=True,shared_grasp_verified=True,
        robot_backend='robotis_movel',base_frame='live_test_base',tool_link='live_test_tcp',
        controller_frame='live_test_base',
        controller_node='/live_test_controller_fixture',tool_below_tcp=.02,
        movel_topic='/live_test/movel',gripper_trajectory_topic='/live_test/gripper',
        joint_states_topic='/live_test/joints',gripper_open=0.,gripper_closed=.7,
        move_duration=.1,gripper_duration=.1,cartesian_speed=.15,action_timeout=8.,
        service_timeout=4.,feedback_timeout=.8,home_mode='current_tcp',
        placement_height_mode='object_surface',auto_grasp_offset=-.02,scan_interval=.05,
        transport_above_objects=True)
    worker = AutoWorker(parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
    rgbd = AlignedRGBDInput(parameter_overrides=[Parameter('color_topic',value='/live_test/color'),
        Parameter('depth_topic',value='/live_test/depth'),Parameter('camera_info_topic',value='/live_test/info')])
    detector = RGBDDetector(parameter_overrides=[Parameter('weights',value=str(args.weights)),
        Parameter('device',value='cpu'),Parameter('imgsz',value=320),Parameter('inference_hz',value=2.),
        Parameter('max_age',value=2.)])
    probe = Node('live_test_probe')
    received = {}
    from vision_msgs.msg import Detection2DArray
    subscriptions = [probe.create_subscription(Image,'/worker_camera/image_rect',lambda m:received.update(rgb=m),qos_profile_sensor_data),
        probe.create_subscription(Image,'/worker_camera/depth_rect',lambda m:received.update(depth=m),qos_profile_sensor_data),
        probe.create_subscription(Detection2DArray,'/worker_camera/detections',lambda m:received.update(detections=m),10)]
    executor = MultiThreadedExecutor(num_threads=8)
    nodes = [fixture,zones,worker,rgbd,detector,probe]
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin,daemon=True)
    thread.start()
    clients = {name:probe.create_client(kind,'/worker/'+name) for name,kind in
               [('prepare_auto_sort',PrepareAutoSort),('start_task',StartTask),('task_status',TaskStatus),('session_status',SessionStatus)]}
    cases = []
    def cli(command,*arguments,expected=0):
        result = subprocess.run([sys.executable,'-m','ai_worker_sorting.worker_cli',command,*arguments],
            capture_output=True,text=True,timeout=25)
        assert result.returncode==expected,(result.stdout,result.stderr)
        return SimpleNamespace(**json.loads(result.stdout))
    def call(name,request):
        client = clients[name]
        assert client.wait_for_service(timeout_sec=5)
        future = client.call_async(request)
        until(future.done,'Service response missing')
        return future.result()
    try:
        until(lambda:all(k in received for k in ('rgb','depth','detections')),'RGB-D/real YOLO pipeline failed')
        assert received['rgb'].encoding=='bgr8' and received['depth'].encoding=='16UC1'
        assert np.median(CvBridge().imgmsg_to_cv2(received['depth']))==900
        cases.append('real camera-topic preprocessing and local YOLO inference execute without Windows bridge')
        with tempfile.TemporaryDirectory() as temporary:
            profile = Path(temporary)/'live.local.yaml'
            result = subprocess.run([sys.executable,'-m','ai_worker_sorting.configure_live',
                '--output',str(profile),'--weights',str(args.weights),'--workspace','.1,.7,-.4,.4,.05,.8',
                '--object-size','.06','--grasp-offset','-.02','--tool-below-tcp','.02','--device','cpu',
                '--controller-node','/live_test_controller_fixture',
                '--camera-info-topic','/live_test/info','--color-topic','/live_test/color','--depth-topic','/live_test/depth',
                '--base-frame','live_test_base','--controller-frame','live_test_base',
                '--tool-link','live_test_tcp','--joint-states-topic','/live_test/joints',
                '--movel-topic','/live_test/movel','--gripper-topic','/live_test/gripper'],capture_output=True,text=True,timeout=15)
            assert result.returncode==0,(result.stdout,result.stderr)
            assert profile.exists()
            import yaml
            settings = yaml.safe_load(profile.read_text())['worker_service']['ros__parameters']
            assert settings['cartesian_speed']==.02 and settings['move_duration']==5.
            result = subprocess.run(['bash',str(Path(__file__).resolve().parents[2]/'scripts/worker.sh'),'doctor'],
                env=dict(os.environ,WORKER_CONFIG=str(profile),WORKER_WS='/tmp/ros2_worker_release_ws',
                         WORKER_PYTHON=sys.executable),capture_output=True,text=True,timeout=25)
            assert result.returncode==0,(result.stdout,result.stderr)
            assert json.loads(result.stdout)['motion_commands_sent'] is False
        cases.append('connected-PC wrapper sources the overlay and doctor checks RGB-D, joints, TF and controller without motion')
        cases.append('read-only configuration tool discovers real typed endpoints and writes a live profile')
        prepared = cli('prepare')
        assert prepared.success,prepared.message
        started = cli('start','--session-id',prepared.session_id,'--max-objects','2')
        assert started.accepted,started.message
        def terminal():
            result = call('task_status',TaskStatus.Request(task_id=started.task_id))
            return result if result.state in ('SUCCEEDED','FAILED','CANCELED') else None
        result = until(terminal,'Real backend task did not terminate',timeout=30)
        assert result.state=='SUCCEEDED',(result.stage,result.message)
        session = call('session_status',SessionStatus.Request(session_id=prepared.session_id))
        assert session.placed_count==2 and fixture.moves and fixture.grips
        assert cli('status','--task-id',started.task_id).state=='SUCCEEDED'
        assert cli('session','--session-id',prepared.session_id).placed_count==2
        assert not cli('cancel','--task-id',started.task_id,expected=1).accepted
        cases.append('PC CLI prepares, starts, queries task/session and returns nonzero for rejected cancellation')
        assert all(obj['placed'] for obj in fixture.objects)
        # table .1 + maximum obstacle .25 + clearance .1 + held extent .03
        assert max(m.pose.pose.position.z for m in fixture.moves)>=.48-1e-6
        assert np.allclose([fixture.pose.x,fixture.pose.y,fixture.pose.z],[.3,0.,.4])
        cases.append('dry_run=false completes two-class placement with actual MoveL/raw-gripper messages and visual grasp/place checks')
        cases.append('captured home TCP and per-object surface height are used for actual goals')
        before = len(fixture.moves)
        grips_before = len(fixture.grips)
        adapter = RobotisMoveLRobot(worker,dict(worker.cfg,tool_link='wrong_tcp'),lambda:None,lambda:None)
        try:
            try:
                adapter.move(Point(.3,0.,.4))
            except RuntimeError as exc:
                assert 'l_gripper_name' in str(exc)
            else:
                raise AssertionError('Mismatched controller TCP was accepted')
            adapter.cancel()
            time.sleep(.1)
            assert len(fixture.moves)==before and len(fixture.grips)==grips_before
        finally:
            adapter.destroy()
        cases.append('mismatched controlled link is rejected before any command')
        # Exercise a controller that accepts messages but never reaches the goal.
        cfg = dict(worker.cfg,action_timeout=1.5,move_duration=.1)
        adapter = RobotisMoveLRobot(worker,cfg,lambda:None,lambda:None)
        try:
            fixture.freeze = True
            try:
                adapter.move(Point(.34,0.,.4))
            except TimeoutError:
                pass
            else:
                raise AssertionError('Unreached TCP was accepted as completed')
            assert adapter.failed and adapter.stopped.is_set()
            adapter.destroy()
            assert adapter in worker._failed_robot_adapters
        finally:
            fixture.freeze = False
        cases.append('unreached TCP times out and sends a measured hold; failed publishers remain alive')
        cfg = dict(worker.cfg,action_timeout=2.,move_duration=.1)
        cancel_started = worker.get_clock().now().nanoseconds
        def recent(message):
            stamp = message.pose.header.stamp
            return stamp.sec*1_000_000_000+stamp.nanosec>=cancel_started
        def cancel_after_goal():
            if any(recent(m) and abs(m.pose.pose.position.x-.34)<1e-6 for m in fixture.moves):
                raise RuntimeError('test cancel after accepted goal')
        adapter = RobotisMoveLRobot(worker,cfg,cancel_after_goal,lambda:None)
        fixture.freeze = True
        try:
            try:
                adapter.move(Point(.34,0.,.4))
            except RuntimeError as exc:
                assert 'test cancel' in str(exc)
            else:
                raise AssertionError('Cancellation was ignored')
            until(lambda:any(recent(m) and m.time_from_start.sec==0 and
                             m.time_from_start.nanosec==200_000_000 and
                             abs(m.pose.pose.position.x-fixture.pose.x)<1e-6 for m in fixture.moves),
                  'No measured hold after cancel')
            adapter.destroy()
            assert adapter.stopped.is_set()
        finally:
            fixture.freeze = False
        cases.append('cancellation interrupts an active command and publishes the measured current TCP hold')
        cfg = dict(worker.cfg,action_timeout=1.5,move_duration=.1,feedback_timeout=.15)
        adapter = RobotisMoveLRobot(worker,cfg,lambda:None,lambda:None)
        adapter.wait_ready(adapter.movel)
        fixture.joint_feedback = False
        before = len(fixture.moves)
        time.sleep(.2)
        try:
            try:
                adapter.move(Point(.32,0.,.4))
            except TimeoutError:
                pass
            else:
                raise AssertionError('Missing joint feedback allowed motion')
            assert len(fixture.moves)==before
        finally:
            fixture.joint_feedback = True
            adapter.destroy()
        cases.append('missing joint feedback rejects motion before a MoveL command is sent')
        report = dict(result='PASS',mode='live_software_path_with_emulated_hardware_no_physical_robot',cases=cases,
                      move_commands=len(fixture.moves),gripper_commands=len(fixture.grips))
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2))
    finally:
        worker.shutdown_event.set()
        if worker.thread:
            worker.thread.join(timeout=3)
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
