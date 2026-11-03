import threading
import math

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .core import Point, DryRobot, pick_and_place


class TaskManager(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('task_manager', parameter_overrides=parameter_overrides)
        defaults = dict(dry_run=True, hardware_configured=False, base_frame='base_link',
                        target_max_age=1.0, workspace=[0.1, 0.7, -0.4, 0.4, 0.05, 0.7],
                        bin_position=[0.35, -0.2, 0.2], home_position=[0.3, 0.0, 0.4],
                        clearance=0.1, grasp_offset=0.0, tool_quaternion=[1.0, 0.0, 0.0, 0.0],
                        planning_group='', tool_link='', move_action='/move_action',
                        gripper_action='', gripper_open=0.04, gripper_closed=0.0,
                        gripper_max_effort=5.0, velocity_scale=0.05,
                        acceleration_scale=0.05, action_timeout=30.0, grasp_check_service='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if not math.isfinite(self.cfg['target_max_age']) or self.cfg['target_max_age'] <= 0:
            raise ValueError('target_max_age must be positive')
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.target = None
        self.worker = None
        self.busy = False
        self.faulted = False
        self.status = self.create_publisher(String, '/sorting/status', 10)
        self.create_subscription(PointStamped, '/sorting/target', self.on_target, 1)
        self.create_service(Trigger, '/sorting/start', self.start)
        self.create_service(Trigger, '/sorting/stop', self.stop)
        if self.cfg['dry_run']:
            self.robot = DryRobot(self.get_logger().info)
        else:
            from .robot_interface import MoveItRobot
            self.robot = MoveItRobot(self, self.cfg, self.check)
        self.get_logger().info(f"Ready; dry_run={self.cfg['dry_run']}. Call /sorting/start once per object.")

    def on_target(self, msg):
        with self.lock:
            if not self.busy:
                self.target = msg

    def check(self):
        if self.stop_event.is_set() or not rclpy.ok():
            raise RuntimeError('Stop requested')

    def report(self, state):
        self.status.publish(String(data=state))
        self.get_logger().info(state)

    def start(self, request, response):
        with self.lock:
            if self.busy or self.faulted:
                response.message = 'Busy or fault latched; inspect robot and restart node after a fault'
                return response
            msg = self.target
            if msg is None or msg.header.frame_id != self.cfg['base_frame']:
                response.message = 'No target in configured base frame'
                return response
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now()-stamp).nanoseconds / 1e9
            if stamp.nanoseconds == 0 or not 0 <= age <= self.cfg['target_max_age']:
                response.message = 'Target is stale or has invalid timestamp'
                return response
            target = Point(msg.point.x, msg.point.y, msg.point.z)
            self.target = None
            self.busy = True
            self.stop_event.clear()
            self.worker = threading.Thread(target=self.run_task, args=(target,), daemon=True)
            self.worker.start()
        response.success = True
        response.message = 'Started one cycle; monitor /sorting/status'
        return response

    def run_task(self, target):
        try:
            pick_and_place(self.robot, target, Point(*self.cfg['bin_position']),
                           Point(*self.cfg['home_position']), self.cfg['workspace'],
                           self.cfg['clearance'], self.cfg['grasp_offset'],
                           self.check, self.report)
        except Exception as exc:
            if hasattr(self.robot, 'cancel'):
                self.robot.cancel()
            with self.lock:
                self.faulted = True
            self.report(f'ERROR: {exc}')
        finally:
            with self.lock:
                self.busy = False
                self.target = None

    def stop(self, request, response):
        # Latch until node restart, including stop received while idle.
        with self.lock:
            self.faulted = True
            self.stop_event.set()
        if hasattr(self.robot, 'cancel'):
            self.robot.cancel()
        response.success = True
        response.message = 'Cancellation requested; use physical emergency stop if required'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = TaskManager()
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None:
            node.stop_event.set()
            if hasattr(node.robot, 'cancel'):
                node.robot.cancel()
            if node.worker is not None:
                node.worker.join(timeout=3.0)
        executor.shutdown()
        if node is not None:
            if hasattr(node.robot, 'destroy'):
                node.robot.destroy()
            node.destroy_node()
        rclpy.try_shutdown()
