"""ROBOTIS Cyclo MoveL + raw gripper trajectory adapter with measured feedback.

    Contract: robotis_interfaces/MoveL, Cyclo AIWorkerMoveLController.
    No sleep-only completion and no assumed gripper gap/metre conversion.
"""
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotis_interfaces.msg import MoveL
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger

from .core import Point, in_workspace
from .locator import transform_point


def orientation_error(a, b):
    dot = abs(sum(x*y for x,y in zip(a,b)))
    return 2*math.acos(min(1., max(0., dot)))


def compose_quaternion(a,b):
    x,y,z,w = a
    u,v,t,s = b
    result = [w*u+x*s+y*t-z*v,w*v-x*t+y*s+z*u,w*t+x*v-y*u+z*s,w*s-x*u-y*v-z*t]
    norm = math.sqrt(sum(c*c for c in result))
    if not math.isfinite(norm) or norm<1e-8:
        raise ValueError('Invalid controller-frame orientation')
    return [c/norm for c in result]


class RobotisMoveLRobot:
    def __init__(self, node, cfg, check, grasp_verifier=None):
        self.node, self.cfg, self.check = node,cfg,check
        if not cfg['hardware_configured'] or not cfg['tool_link']:
            raise ValueError('Robot hardware configuration and measured TCP link are required')
        q = cfg['tool_quaternion']
        if len(q)!=4 or not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1)>1e-3:
            raise ValueError('TCP orientation must be a normalized XYZW quaternion')
        for name in ('move_duration','cartesian_speed','angular_speed','feedback_timeout','position_tolerance',
                     'orientation_tolerance','gripper_duration','gripper_tolerance','action_timeout'):
            if not math.isfinite(cfg[name]) or cfg[name]<=0:
                raise ValueError(f'{name} must be positive')
        if cfg['cartesian_speed']>.15 or cfg['gripper_open']==cfg['gripper_closed']:
            raise ValueError('Invalid Cartesian speed or gripper endpoints')
        for name in ('gripper_open','gripper_closed'):
            if not cfg['gripper_joint_min']<=cfg[name]<=cfg['gripper_joint_max']:
                raise ValueError(f'{name} is outside configured joint limits')
        if grasp_verifier is None and not cfg['grasp_check_service']:
            raise ValueError('Measured grasp verification is required')
        self.verifier = grasp_verifier
        self.lock = threading.Lock()
        self.joints, self.joint_received = {},0.
        self.joint_stamp = 0
        self.contract_verified = False
        self.stopped = threading.Event()
        self.active = False
        self.failed = False
        self.tf = Buffer()
        self.listener = TransformListener(self.tf,node)
        self.movel = node.create_publisher(MoveL,cfg['movel_topic'],10)
        self.grip = node.create_publisher(JointTrajectory,cfg['gripper_trajectory_topic'],10)
        self.sub = node.create_subscription(JointState,cfg['joint_states_topic'],self.on_joints,qos_profile_sensor_data)
        self.verify_client = node.create_client(Trigger,cfg['grasp_check_service']) if grasp_verifier is None else None

    def on_joints(self,msg):
        joint = self.cfg['gripper_joint']
        if len(msg.name)!=len(msg.position) or joint not in msg.name:
            return
        position = msg.position[msg.name.index(joint)]
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
        if stamp<=0 or not 0<=age<=self.cfg['feedback_timeout'] or not math.isfinite(position):
            return
        with self.lock:
            self.joints = {name:float(p) for name,p in zip(msg.name,msg.position)}
            self.joint_received = time.monotonic()
            self.joint_stamp = stamp

    def current_pose(self):
        tf = self.tf.lookup_transform(self.cfg['base_frame'],self.cfg['tool_link'],Time())
        stamp = tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
        if stamp<=0 or not 0<=age<=self.cfg['feedback_timeout']:
            raise RuntimeError('TCP TF feedback is stale')
        t,q = tf.transform.translation,tf.transform.rotation
        if not all(math.isfinite(x) for x in (t.x,t.y,t.z,q.x,q.y,q.z,q.w)):
            raise RuntimeError('TCP TF feedback is invalid')
        norm = math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        if norm<1e-8:
            raise RuntimeError('TCP TF quaternion is invalid')
        return Point(t.x,t.y,t.z),(q.x/norm,q.y/norm,q.z/norm,q.w/norm),stamp

    def joint_position(self):
        with self.lock:
            age = (self.node.get_clock().now().nanoseconds-self.joint_stamp)/1e9
            if not 0<=age<=self.cfg['feedback_timeout'] or time.monotonic()-self.joint_received>self.cfg['feedback_timeout']:
                raise RuntimeError('Joint feedback is stale')
            return self.joints[self.cfg['gripper_joint']]

    def wait_ready(self,publisher):
        if not self.contract_verified:
            from .controller_contract import verify_controller
            verify_controller(self.node,self.cfg)
            self.contract_verified = True
        deadline = time.monotonic()+self.cfg['action_timeout']
        while time.monotonic()<deadline:
            self.check()
            if self.stopped.is_set():
                raise RuntimeError('Robot adapter stopped')
            if publisher.get_subscription_count():
                try:
                    self.current_pose()
                    self.joint_position()
                    return
                except (RuntimeError,KeyError):
                    pass
                except Exception:
                    pass  # TF listener may still be discovering the robot.
            time.sleep(.02)
        raise TimeoutError('Controller subscriber, fresh TCP TF or joint feedback unavailable')

    def command_pose(self,point,q,duration):
        if not self.contract_verified:
            raise RuntimeError('Controller contract has not been verified')
        controller_frame = self.cfg.get('controller_frame',self.cfg['base_frame'])
        if controller_frame!=self.cfg['base_frame']:
            # Cyclo consumes numeric poses in its model root; it does not
            # transform PoseStamped.header.frame_id on our behalf.
            measured = self.tf.lookup_transform(controller_frame,self.cfg['base_frame'],Time())
            stamp = measured.header.stamp.sec*1_000_000_000+measured.header.stamp.nanosec
            age = (self.node.get_clock().now().nanoseconds-stamp)/1e9
            if stamp and not 0<=age<=self.cfg['feedback_timeout']:
                raise RuntimeError('Controller-frame TF is stale')
            transform = measured.transform
            t,r = transform.translation,transform.rotation
            if not all(math.isfinite(x) for x in (t.x,t.y,t.z,r.x,r.y,r.z,r.w)) or abs(r.x*r.x+r.y*r.y+r.z*r.z+r.w*r.w-1)>1e-3:
                raise RuntimeError('Controller-frame TF is invalid')
            p = transform_point([point.x,point.y,point.z],(t.x,t.y,t.z),(r.x,r.y,r.z,r.w))
            point = Point(*map(float,p))
            q = compose_quaternion((r.x,r.y,r.z,r.w),q)
        msg = MoveL()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = controller_frame
        msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z = point.x,point.y,point.z
        orientation = msg.pose.pose.orientation
        orientation.x,orientation.y,orientation.z,orientation.w = q
        msg.time_from_start.sec = int(duration)
        msg.time_from_start.nanosec = int((duration-int(duration))*1e9)
        self.movel.publish(msg)

    def move(self,point):
        self.move_pose(point,self.cfg['tool_quaternion'])

    def capture_home(self):
        self.wait_ready(self.movel)
        point,orientation,_ = self.current_pose()
        self.home_pose = (point,orientation)
        return point

    def return_home(self,point):
        orientation = self.home_pose[1] if hasattr(self,'home_pose') else self.cfg['tool_quaternion']
        current,_,_ = self.current_pose()
        self.move_pose(current,orientation)
        self.move_pose(point,orientation)

    def raise_clear(self,point):
        _,orientation,_ = self.current_pose()
        self.move_pose(point,orientation)

    def move_pose(self,point,target_q):
        if not in_workspace(point,self.cfg['workspace']):
            raise ValueError('MoveL target outside configured workspace')
        self.wait_ready(self.movel)
        current,q,_ = self.current_pose()
        distance = math.dist((current.x,current.y,current.z),(point.x,point.y,point.z))
        duration = max(self.cfg['move_duration'],distance/self.cfg['cartesian_speed'],
                       orientation_error(q,target_q)/self.cfg['angular_speed'])
        if duration+1>=self.cfg['action_timeout']:
            raise ValueError('Move duration exceeds configured action_timeout')
        began = self.node.get_clock().now().nanoseconds
        self.active = True
        try:
            self.command_pose(point,target_q,duration)
            deadline,stable,last_stamp = time.monotonic()+self.cfg['action_timeout'],0,0
            while time.monotonic()<deadline:
                self.check()
                measured,orientation,stamp = self.current_pose()
                self.joint_position()
                converged = (stamp>began and math.dist((point.x,point.y,point.z),
                    (measured.x,measured.y,measured.z))<=self.cfg['position_tolerance']
                    and orientation_error(orientation,target_q)<=self.cfg['orientation_tolerance'])
                if stamp!=last_stamp:
                    stable = stable+1 if converged else 0
                    last_stamp = stamp
                if stable>=3:
                    self.active = False
                    return
                time.sleep(.03)
            raise TimeoutError('MoveL did not converge to the measured TCP target')
        except Exception:
            self.failed = True
            self.cancel()
            raise

    def gripper(self,opened):
        self.wait_ready(self.grip)
        initial = self.joint_position()
        target = self.cfg['gripper_open' if opened else 'gripper_closed']
        began,deadline = time.monotonic(),time.monotonic()+self.cfg['action_timeout']
        observed = []
        self.active = True
        try:
            while time.monotonic()<deadline:
                self.check()
                command = JointTrajectory()
                command.joint_names = [self.cfg['gripper_joint']]
                point = JointTrajectoryPoint()
                point.positions = [target]
                point.time_from_start.nanosec = 100_000_000
                command.points = [point]
                # Cyclo's raw input updates its gripper setpoint; it remains the
                # only publisher of full arm trajectories to the robot hardware.
                self.grip.publish(command)
                position = self.joint_position()
                observed = (observed+[position])[-8:]
                if time.monotonic()-began>=self.cfg['gripper_duration']:
                    reached = abs(position-target)<=self.cfg['gripper_tolerance']
                    blocked = (not opened and len(observed)==8 and max(observed)-min(observed)<.01
                               and abs(position-initial)>.03 and (position-initial)*(target-initial)>0)
                    if reached or blocked:
                        self.active = False
                        return
                time.sleep(.02)
            raise TimeoutError('Gripper command not confirmed by joint feedback')
        except Exception:
            self.failed = True
            self.cancel()
            raise

    def verify_grasp(self):
        self.check()
        if self.verifier:
            self.verifier()
            return
        if not self.verify_client.wait_for_service(timeout_sec=2):
            raise RuntimeError('Grasp verification service unavailable')
        future = self.verify_client.call_async(Trigger.Request())
        deadline = time.monotonic()+self.cfg['action_timeout']
        while not future.done():
            self.check()
            if time.monotonic()>deadline:
                raise TimeoutError('Grasp verification timed out')
            time.sleep(.02)
        if not future.result().success:
            raise RuntimeError('Grasp not verified')

    def cancel(self):
        if self.stopped.is_set():
            return
        self.stopped.set()
        if not self.contract_verified:
            return
        try:
            point,q,_ = self.current_pose()
            self.command_pose(point,q,.2)
        except Exception as exc:
            self.node.get_logger().error(f'Cannot send measured hold target: {exc}')
        try:
            command = JointTrajectory()
            command.joint_names = [self.cfg['gripper_joint']]
            point = JointTrajectoryPoint()
            point.positions = [self.joint_position()]
            point.time_from_start.nanosec = 100_000_000
            command.points = [point]
            self.grip.publish(command)
        except Exception as exc:
            self.node.get_logger().error(f'Cannot hold the measured gripper position: {exc}')

    def destroy(self):
        if self.active or self.failed:
            self.cancel()
            # Keep the reliable hold publisher alive on the fault-latched node.
            retained = getattr(self.node,'_failed_robot_adapters',None)
            if retained is None:
                retained = self.node._failed_robot_adapters = []
            if self not in retained:
                retained.append(self)
            return
        self.node.destroy_subscription(self.sub)
        self.listener.unregister()
        self.node.destroy_publisher(self.movel)
        self.node.destroy_publisher(self.grip)
        if self.verify_client:
            self.node.destroy_client(self.verify_client)
