"""Generic MoveGroup + GripperCommand adapter, not model-specific bringup."""
import math
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger


class MoveItRobot:
    def __init__(self, node, cfg, check):
        self.node, self.cfg, self.check = node, cfg, check
        for key in ('planning_group', 'tool_link', 'gripper_action', 'grasp_check_service'):
            if not cfg[key]:
                raise ValueError(f'Live mode requires {key}')
        if not cfg['hardware_configured']:
            raise ValueError('Live mode requires hardware_configured: true')
        q = cfg['tool_quaternion']
        if len(q) != 4 or not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1) > 1e-3:
            raise ValueError('tool_quaternion must be normalized')
        for key in ('velocity_scale', 'acceleration_scale'):
            if not math.isfinite(cfg[key]) or not 0 < cfg[key] <= 0.1:
                raise ValueError(f'{key} must be in (0, 0.1]')
        if not math.isfinite(cfg['action_timeout']) or cfg['action_timeout'] <= 0:
            raise ValueError('action_timeout must be positive')
        if not all(math.isfinite(cfg[k]) for k in ('gripper_open', 'gripper_closed', 'gripper_max_effort')):
            raise ValueError('Invalid gripper settings')
        if cfg['gripper_max_effort'] <= 0:
            raise ValueError('gripper_max_effort must be positive')
        self.move_client = ActionClient(node, MoveGroup, cfg['move_action'])
        self.grip_client = ActionClient(node, GripperCommand, cfg['gripper_action'])
        self.verify_client = node.create_client(Trigger, cfg['grasp_check_service'])
        self._handles = []
        self._lock = threading.Lock()
        self._failed_action = False

    def wait(self, future, interruptible=True):
        deadline = time.monotonic() + self.cfg['action_timeout']
        ready = threading.Event()
        future.add_done_callback(lambda _: ready.set())
        while not ready.wait(0.05):
            if interruptible:
                self.check()
            if time.monotonic() >= deadline:
                raise TimeoutError('Robot request timed out; execution state is uncertain')
        if interruptible:
            self.check()
        return future.result()

    def cancel(self):
        with self._lock:
            handles = list(self._handles)
        for handle in handles:
            handle.cancel_goal_async()

    def action(self, client, goal):
        self.check()
        if not client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError('Required action server is unavailable')
        pending = client.send_goal_async(goal)

        # A stop/timeout before acceptance must also cancel a LATE accepted goal.
        abandoned = threading.Event()

        def accepted(future):
            try:
                handle = future.result()
                if handle.accepted:
                    with self._lock:
                        self._handles.append(handle)
                    if abandoned.is_set():
                        handle.cancel_goal_async()
            except Exception as exc:
                self.node.get_logger().error(f'Goal response error: {exc}')

        pending.add_done_callback(accepted)
        try:
            handle = self.wait(pending)
            if not handle.accepted:
                raise RuntimeError('Robot rejected goal')
            result = self.wait(handle.get_result_async())
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Robot action failed: status={result.status}')
            return result.result
        except Exception:
            self._failed_action = True
            abandoned.set()
            self.cancel()
            raise

    def move(self, point):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.cfg['planning_group']
        req.start_state.is_diff = True
        req.num_planning_attempts = 3
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.cfg['velocity_scale']
        req.max_acceleration_scaling_factor = self.cfg['acceleration_scale']
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.cfg['base_frame']
        position.link_name = self.cfg['tool_link']
        position.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.003]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point.x, point.y, point.z
        pose.orientation.w = 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [pose]
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.cfg['base_frame']
        orientation.link_name = self.cfg['tool_link']
        q = self.cfg['tool_quaternion']
        orientation.orientation.x, orientation.orientation.y = q[0], q[1]
        orientation.orientation.z, orientation.orientation.w = q[2], q[3]
        orientation.absolute_x_axis_tolerance = 0.05
        orientation.absolute_y_axis_tolerance = 0.05
        orientation.absolute_z_axis_tolerance = 0.05
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        req.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        result = self.action(self.move_client, goal)
        if result.error_code.val != 1:
            raise RuntimeError(f'MoveIt error {result.error_code.val}')

    def gripper(self, opened):
        goal = GripperCommand.Goal()
        goal.command.position = self.cfg['gripper_open' if opened else 'gripper_closed']
        goal.command.max_effort = self.cfg['gripper_max_effort']
        result = self.action(self.grip_client, goal)
        if opened and not result.reached_goal:
            raise RuntimeError('Gripper did not reach open position')
        if not opened and not (result.reached_goal or result.stalled):
            raise RuntimeError('Gripper close failed')

    def verify_grasp(self):
        self.check()
        if not self.verify_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Grasp verification service unavailable')
        result = self.wait(self.verify_client.call_async(Trigger.Request()))
        if not result.success:
            raise RuntimeError(f'Grasp not verified: {result.message}')

    def destroy(self):
        self.cancel()
        if self._failed_action:
            # Keep late acceptance/cancel callbacks serviced by the fault-latched
            # node. A node restart is allowed only after checking the hardware.
            retained = getattr(self.node, '_failed_robot_adapters', None)
            if retained is None:
                retained = self.node._failed_robot_adapters = []
            if self not in retained:
                retained.append(self)
                self.node.get_logger().warning('Controller clients retained after failed action; inspect hardware before node restart')
            return
        self.move_client.destroy()
        self.grip_client.destroy()
        self.node.destroy_client(self.verify_client)
