"""ROS2 RGB-D/TF/clock and command bridge around real MuJoCo dynamics."""
import json
import threading
import time

import mujoco
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory
from robotis_interfaces.msg import MoveL

from .sim_physics import PhysicsWorld
from .sim_scene import intrinsics


class SimulationBridge(Node):
    def __init__(self):
        super().__init__('controller', namespace='/sim')
        for name, value in dict(controller_kind='mujoco', l_gripper_name='sim_tcp',
                                left_movel_topic='/sim/left/goal_move',
                                left_raw_traj_topic='/sim/left/gripper_trajectory',
                                left_gripper_joint='gripper_l_joint1', joint_states_topic='/sim/joint_states').items():
            self.declare_parameter(name, value)
        self.world, self.lock, self.shutdown_event = PhysicsWorld(), threading.RLock(), threading.Event()
        self.camera = CvBridge()
        self.clock_pub = self.create_publisher(Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.joint_pub = self.create_publisher(JointState, '/sim/joint_states', qos_profile_sensor_data)
        self.color_pub = self.create_publisher(Image, '/sim/camera/color/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/sim/camera/aligned_depth_to_color/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/sim/camera/color/camera_info', qos_profile_sensor_data)
        self.overview_pub = self.create_publisher(Image, '/sim/image_overview', qos_profile_sensor_data)
        self.evaluation_pub = self.create_publisher(String, '/sim/evaluation', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(MoveL, '/sim/left/goal_move', self.move, 10)
        self.create_subscription(JointTrajectory, '/sim/left/gripper_trajectory', self.grip, 10)
        self.create_service(Trigger, '/sim/stop', self.stop)
        self.create_service(Trigger, '/sim/check_grasp', self.check_grasp)
        self.create_service(Trigger, '/sim/evaluate', self.evaluate)
        self.previous_contact = {}
        self.lifted_objects = set()
        self.contact_since = {}
        self.physics_thread = threading.Thread(target=self.physics_loop, daemon=True)
        self.render_thread = threading.Thread(target=self.render_loop, daemon=True)
        self.physics_thread.start()
        self.render_thread.start()

    @staticmethod
    def stamp(seconds):
        from builtin_interfaces.msg import Time
        ns = max(1, round(seconds*1e9))
        sec, nsec = divmod(ns, 1_000_000_000)
        return Time(sec=sec, nanosec=nsec)

    def move(self, msg):
        try:
            with self.lock:
                if self.world.stopped:
                    return
            if msg.pose.header.frame_id != 'sim_base':
                raise ValueError('Cartesian command must use sim_base coordinates')
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            duration = msg.time_from_start.sec+msg.time_from_start.nanosec/1e9
            with self.lock:
                self.world.plan([p.x,p.y,p.z], [q.w,q.x,q.y,q.z], duration)
        except Exception as exc:
            with self.lock:
                self.world.fault = str(exc)
                self.world.stop()
            self.get_logger().error(f'Simulation command rejected: {exc}')

    def grip(self, msg):
        with self.lock:
            if self.world.fault or self.world.stopped:
                return
            if (msg.joint_names != ['gripper_l_joint1'] or len(msg.points) != 1
                    or len(msg.points[0].positions) != 1):
                self.world.fault = 'Invalid simulated gripper command'
                self.world.stop()
                return
            target = msg.points[0].positions[0]
            if not np.isfinite(target) or not 0 <= target <= 1.1:
                self.world.fault = 'Gripper target outside model range'
                self.world.stop()
                return
            self.world.data.ctrl[self.world.grip_actuator] = target

    def stop(self, request, response):
        with self.lock:
            self.world.stop()
        response.success, response.message = True, 'Pending path discarded; measured joint hold set'
        return response

    def check_grasp(self, request, response):
        with self.lock:
            evaluation = self.world.evaluation()
            response.success = any(v['lifted'] and v['bilateral_contact'] and
                self.world.data.time-self.contact_since.get(name,self.world.data.time) >= .15
                for name,v in evaluation.items())
        response.message = 'Bilateral contact and sustained object lift confirmed' if response.success else 'No stable physical grasp'
        return response

    def evaluate(self, request, response):
        with self.lock:
            response.success = not bool(self.world.fault)
            response.message = json.dumps({'fault': self.world.fault, 'objects': self.world.evaluation()})
        return response

    def transform(self, name, position, quaternion, stamp):
        msg = TransformStamped()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, 'sim_base', name
        t, q = msg.transform.translation, msg.transform.rotation
        t.x,t.y,t.z = map(float, position)
        q.w,q.x,q.y,q.z = map(float, quaternion)
        return msg

    def publish_state(self):
        m,d = self.world.model,self.world.data
        stamp = self.stamp(d.time)
        self.clock_pub.publish(Clock(clock=stamp))
        joints = JointState()
        joints.header.stamp = stamp
        for j in range(m.njnt):
            if m.jnt_type[j] not in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                continue
            joints.name.append(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j))
            joints.position.append(float(d.qpos[m.jnt_qposadr[j]]))
            joints.velocity.append(float(d.qvel[m.jnt_dofadr[j]]))
        self.joint_pub.publish(joints)
        position,q = self.world.pose()
        transforms = [self.transform('sim_tcp', position, q, stamp)]
        cam = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,'work_camera')
        rotation = d.cam_xmat[cam].reshape(3,3)@np.diag([1.,-1.,-1.])
        optical_q = np.empty(4)
        mujoco.mju_mat2Quat(optical_q, rotation.reshape(-1))
        transforms.append(self.transform('sim_camera_optical', d.cam_xpos[cam], optical_q, stamp))
        # All body transforms come from one source, not from a different URDF state.
        for b in range(1, m.nbody):
            name = mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_BODY,b)
            if name and not name.startswith('item_'):
                transforms.append(self.transform('sim_'+name,d.xpos[b],d.xquat[b],stamp))
        self.tf.sendTransform(transforms)

    def physics_loop(self):
        while not self.shutdown_event.is_set():
            start = time.monotonic()
            try:
                with self.lock:
                    self.world.step(10)
                    for name,state in self.world.evaluation().items():
                        if state['lifted'] and state['bilateral_contact']:
                            self.contact_since.setdefault(name,self.world.data.time)
                            if self.world.data.time-self.contact_since[name] >= .15:
                                self.lifted_objects.add(name)
                        else:
                            self.contact_since.pop(name,None)
                    self.publish_state()
                    self.evaluation_pub.publish(String(data=json.dumps({'time': self.world.data.time,
                        'fault': self.world.fault, 'objects': self.world.evaluation()})))
            except Exception as exc:
                with self.lock:
                    self.world.fault = str(exc)
                    self.world.stop()
                self.get_logger().error(f'Physics fault: {exc}', throttle_duration_sec=5.)
            self.shutdown_event.wait(max(0, .02-(time.monotonic()-start)))

    def render_loop(self):
        try:
            model = self.world.model
            snapshot = mujoco.MjData(model)
            with mujoco.Renderer(model, height=480, width=640) as renderer:
                while not self.shutdown_event.is_set():
                    start = time.monotonic()
                    with self.lock:
                        snapshot.qpos[:] = self.world.data.qpos
                        stamp = self.stamp(self.world.data.time)
                    mujoco.mj_forward(model, snapshot)
                    renderer.disable_depth_rendering()
                    renderer.update_scene(snapshot, camera='work_camera')
                    rgb = renderer.render().copy()
                    renderer.enable_depth_rendering()
                    renderer.update_scene(snapshot, camera='work_camera')
                    depth = renderer.render().copy()
                    fx,fy,cx,cy = intrinsics(model)
                    info = CameraInfo()
                    info.header.stamp, info.header.frame_id = stamp, 'sim_camera_optical'
                    info.width, info.height = 640, 480
                    info.distortion_model, info.d = 'plumb_bob', [0.]*5
                    info.k = [fx,0.,cx,0.,fy,cy,0.,0.,1.]
                    info.r = np.eye(3).reshape(-1).tolist()
                    info.p = [fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
                    color = self.camera.cv2_to_imgmsg(rgb, encoding='rgb8')
                    metric = self.camera.cv2_to_imgmsg(depth, encoding='32FC1')
                    color.header = metric.header = info.header
                    self.info_pub.publish(info)
                    self.depth_pub.publish(metric)
                    self.color_pub.publish(color)
                    overview_camera = mujoco.MjvCamera()
                    mujoco.mjv_defaultCamera(overview_camera)
                    overview_camera.lookat[:] = [.2,.25,.75]
                    overview_camera.distance,overview_camera.azimuth,overview_camera.elevation = 2.1,140.,-22.
                    renderer.disable_depth_rendering()
                    renderer.update_scene(snapshot,camera=overview_camera)
                    overview = self.camera.cv2_to_imgmsg(renderer.render().copy(),encoding='rgb8')
                    overview.header = info.header
                    self.overview_pub.publish(overview)
                    self.shutdown_event.wait(max(0, .15-(time.monotonic()-start)))
        except Exception as exc:
            with self.lock:
                self.world.fault = f'Camera rendering failed: {exc}'
                self.world.stop()
            self.get_logger().error(self.world.fault)

    def destroy_node(self):
        self.shutdown_event.set()
        self.physics_thread.join(timeout=5)
        self.render_thread.join(timeout=5)
        return super().destroy_node()
