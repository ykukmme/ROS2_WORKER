"""Asynchronous sorting jobs behind ROS2 request/status/cancel services."""
import json
import math
import re
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from ai_worker_interfaces.srv import SortOnce, TaskStatus, CancelTask, DetectObjects

from .core import Point, DryRobot, in_workspace, pick_and_place
from .tasks import TaskStore, TaskCanceled


class DelayedDryRobot(DryRobot):
    def __init__(self, log, delay, check, cancel):
        super().__init__(log)
        self.delay, self.check, self.stop = delay, check, cancel

    def pause(self):
        self.stop.wait(self.delay)
        self.check()

    def move(self, point):
        self.pause()
        super().move(point)

    def gripper(self, opened):
        self.pause()
        super().gripper(opened)

    def verify_grasp(self):
        self.pause()
        super().verify_grasp()


class WorkerService(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('worker_service', parameter_overrides=parameter_overrides)
        defaults = dict(dry_run=True, hardware_configured=False, base_frame='base_link',
                        target_max_age=1.0, min_confidence=0.8, service_timeout=3.0,
                        task_timeout=120.0, dry_step_delay=0.15, history_capacity=100,
                        supported_classes=['box', 'bottle'],
                        workspace=[0.1, 0.7, -0.4, 0.4, 0.05, 0.7],
                        home_position=[0.3, 0.0, 0.4], clearance=0.1,
                        tool_quaternion=[1.0, 0.0, 0.0, 0.0],
                        planning_group='', tool_link='', move_action='/move_action',
                        gripper_action='', gripper_open=0.04, gripper_closed=0.0,
                        gripper_max_effort=5.0, velocity_scale=0.05,
                        acceleration_scale=0.05, action_timeout=30.0,
                        grasp_check_service='', stop_file='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key in ('service_timeout', 'task_timeout', 'target_max_age'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if not 0 <= self.cfg['dry_step_delay'] <= 5 or not 0 < self.cfg['min_confidence'] <= 1:
            raise ValueError('Invalid delay or confidence')
        self.classes = {}
        for index, name in enumerate(self.cfg['supported_classes']):
            if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', name):
                raise ValueError(f'Invalid configured class name: {name}')
            settings = dict(bin_position=[0.35, -0.2 if index % 2 == 0 else 0.2, 0.2],
                            grasp_offset=0.0, gripper_open=self.cfg['gripper_open'],
                            gripper_closed=self.cfg['gripper_closed'])
            for key, value in settings.items():
                param = f'objects.{name}.{key}'
                self.declare_parameter(param, value)
                settings[key] = self.get_parameter(param).value
            self.classes[name] = settings
        self.store = TaskStore(self.cfg['history_capacity'])
        self.shutdown_event = threading.Event()
        self.thread = None
        self.robot = None
        self.vision = self.create_client(DetectObjects, '/vision/detect_objects')
        self.events = self.create_publisher(String, '/worker/status', 10)
        self.create_service(SortOnce, '/worker/sort_once', self.sort_once)
        self.create_service(TaskStatus, '/worker/task_status', self.task_status)
        self.create_service(CancelTask, '/worker/cancel_task', self.cancel_task)
        if not self.cfg['dry_run'] and not self.cfg['hardware_configured']:
            raise ValueError('Real motion requires verified hardware configuration')
        self.get_logger().info(f"Worker services ready; dry_run={self.cfg['dry_run']}")

    def publish(self, task):
        self.events.publish(String(data=json.dumps(self.store.snapshot(task.task_id))))

    def sort_once(self, request, response):
        if request.target_class not in self.classes:
            response.message = 'Unsupported class; configure class-specific bin and grasp settings'
            return response
        try:
            task = self.store.admit(request.target_class)
        except RuntimeError as exc:
            response.message = str(exc)
            return response
        self.publish(task)
        self.thread = threading.Thread(target=self.run_task, args=(task,), daemon=True)
        self.thread.start()
        response.accepted, response.task_id = True, task.task_id
        response.message = 'Queued; poll task_status for actual result'
        return response

    def task_status(self, request, response):
        snapshot = self.store.snapshot(request.task_id)
        if snapshot is None:
            response.message = 'Unknown or expired task ID'
            return response
        response.found = True
        response.state, response.stage, response.message = snapshot['state'], snapshot['stage'], snapshot['message']
        return response

    def cancel_task(self, request, response):
        response.accepted = self.store.request_cancel(request.task_id)
        response.message = ('Cancellation requested; poll status for outcome' if response.accepted
                            else 'Task unknown or already terminal')
        return response

    def check(self, task, deadline):
        if task.cancel.is_set() or self.shutdown_event.is_set() or not rclpy.ok():
            raise TaskCanceled('Task cancellation requested')
        if time.monotonic() >= deadline:
            raise TimeoutError('Task time limit exceeded')

    def detect(self, task, check):
        limit = time.monotonic()+self.cfg['service_timeout']
        while not self.vision.wait_for_service(timeout_sec=0.05):
            check()
            if time.monotonic() >= limit:
                raise TimeoutError('Vision service unavailable')
        request = DetectObjects.Request(target_class=task.target_class)
        future = self.vision.call_async(request)
        try:
            while not future.done():
                check()
                if time.monotonic() >= limit:
                    raise TimeoutError('Vision service response timeout')
                task.cancel.wait(.02)
            check()
            result = future.result()
            if not result.success:
                raise RuntimeError(f'Vision failed: {result.message}')
            return result.objects
        finally:
            if not future.done():
                self.vision.remove_pending_request(future)
                future.cancel()

    def select_target(self, objects, target_class):
        candidates = []
        for obj in objects:
            p, header = obj.surface_pose.pose.position, obj.surface_pose.header
            stamp = header.stamp.sec*1_000_000_000+header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            point = Point(p.x, p.y, p.z)
            if (obj.class_name == target_class and obj.localized
                    and (self.cfg['dry_run'] or not obj.simulated)
                    and math.isfinite(obj.confidence) and self.cfg['min_confidence'] <= obj.confidence <= 1
                    and header.frame_id == self.cfg['base_frame'] and stamp > 0
                    and 0 <= age <= self.cfg['target_max_age']
                    and in_workspace(point, self.cfg['workspace'])):
                candidates.append((obj.confidence, point))
        if not candidates:
            raise ValueError('No fresh localized target in workspace; 2D-only/synthetic-live targets are rejected')
        return max(candidates, key=lambda c: c[0])[1]

    def run_task(self, task):
        deadline = time.monotonic()+self.cfg['task_timeout']
        check = lambda: self.check(task, deadline)
        live_robot = None
        outcome, message, fault = 'FAILED', 'Task did not complete', False
        try:
            def report(stage):
                self.store.stage(task, stage)
                self.publish(task)
            report('DETECT')
            check()
            objects = self.detect(task, check)
            report('LOCATE')
            target = self.select_target(objects, task.target_class)
            settings = self.classes[task.target_class]
            cfg = dict(self.cfg, **{k: settings[k] for k in ('gripper_open', 'gripper_closed')})
            if cfg['dry_run']:
                robot = DelayedDryRobot(self.get_logger().info, cfg['dry_step_delay'], check, task.cancel)
            else:
                from .robot_interface import MoveItRobot
                robot = live_robot = MoveItRobot(self, cfg, check)
            self.robot = robot
            pick_and_place(robot, target, Point(*settings['bin_position']), Point(*cfg['home_position']),
                           cfg['workspace'], cfg['clearance'], settings['grasp_offset'], check, report)
            # Serialize completion and cancel acceptance to avoid falsely reporting success.
            check()
            outcome, message = 'SUCCEEDED', ('Object sorted (simulated)' if cfg['dry_run'] else 'Object sorted')
        except TaskCanceled as exc:
            if live_robot is not None:
                live_robot.cancel()
                outcome, message, fault = 'FAILED', 'Cancellation requested; controller stop unconfirmed; inspect hardware', True
            else:
                outcome, message = 'CANCELED', str(exc)
        except Exception as exc:
            if live_robot is not None:
                live_robot.cancel()
            outcome, message, fault = 'FAILED', str(exc), live_robot is not None
        finally:
            # Destroy clients before allowing a later task to use shared robot resources.
            if live_robot is not None:
                try:
                    live_robot.destroy()
                except Exception as exc:
                    outcome, message, fault = 'FAILED', f'Robot cleanup failed: {exc}', True
            self.robot = None
            if outcome == 'SUCCEEDED' and task.cancel.is_set() and live_robot is not None:
                outcome, message, fault = 'FAILED', 'Late cancellation; inspect controller state', True
            self.store.finish(task, outcome, message, fault=fault)
            self.publish(task)


def main(args=None):
    from pathlib import Path
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = WorkerService()
        stop_file = node.cfg['stop_file']
        if stop_file and Path(stop_file).exists():
            raise ValueError('Worker stop file already exists')
        executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=.2)
            if stop_file and Path(stop_file).exists():
                break
    finally:
        if node is not None:
            node.shutdown_event.set()
            if node.thread is not None:
                node.thread.join(timeout=3)
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
