"""Measured MuJoCo motion using the shared feedback adapter, on isolated topics."""
import threading
import time

from std_srvs.srv import Trigger

from .controller_contract import verify_controller
from .robotis_robot import RobotisMoveLRobot


class MujocoRobot(RobotisMoveLRobot):
    def __init__(self, node, cfg, check, grasp_verifier=None):
        super().__init__(node, cfg, check, grasp_verifier)
        self.stop_client = node.create_client(Trigger, '/sim/stop')
        self.stop_confirmed = False
        self.physics_grasp_client = node.create_client(Trigger, '/sim/check_grasp')

    def verify_grasp(self):
        super().verify_grasp()
        if not self.physics_grasp_client.wait_for_service(timeout_sec=1):
            raise RuntimeError('Physics contact verification unavailable')
        future = self.physics_grasp_client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _:event.set())
        if not event.wait(3) or not future.result() or not future.result().success:
            raise RuntimeError('Physical contact/lift verification failed')

    def validate_runtime(self, cfg):
        if cfg.get('runtime_mode') != 'mujoco' or cfg['hardware_configured'] or not cfg['tool_link'].startswith('sim_'):
            raise ValueError('MuJoCo requires an isolated simulation profile')
        for name in ('movel_topic', 'gripper_trajectory_topic', 'joint_states_topic', 'controller_node'):
            if not cfg[name].startswith('/sim/'):
                raise ValueError(f'Simulation requires /sim/ topic/node: {name}')

    def verify_contract(self):
        verify_controller(self.node, self.cfg, expected_kind='mujoco')

    def current_pose(self):
        # /clock and /tf use different DDS writers. A just-arrived TF can be
        # one clock callback ahead; wait for that clock, never alter the stamp.
        deadline = time.monotonic()+.5
        while True:
            try:
                return super().current_pose()
            except RuntimeError as exc:
                if str(exc)!='TCP TF feedback is stale' or time.monotonic()>=deadline:
                    raise
                from rclpy.time import Time
                tf = self.tf.lookup_transform(self.cfg['base_frame'],self.cfg['tool_link'],Time())
                stamp = tf.header.stamp.sec*1_000_000_000+tf.header.stamp.nanosec
                if stamp<=self.node.get_clock().now().nanoseconds:
                    raise
                time.sleep(.004)

    def cancel(self):
        if self.stopped.is_set():
            return
        self.stopped.set()
        if not self.stop_client.wait_for_service(timeout_sec=1):
            return
        future = self.stop_client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if event.wait(2) and future.result() and future.result().success:
            deadline, stable = time.monotonic()+2, 0
            while time.monotonic() < deadline:
                try:
                    first, _, stamp = self.current_pose()
                    time.sleep(.06)
                    second, _, newer = self.current_pose()
                    distance = sum((getattr(first,k)-getattr(second,k))**2 for k in ('x','y','z'))**.5
                    stable = stable+1 if newer > stamp and distance < .002 else 0
                    if stable >= 3:
                        self.stop_confirmed = True
                        self.active = False
                        return
                except Exception:
                    break

    def destroy(self):
        super().destroy()
        if not self.active and not self.failed:
            self.node.destroy_client(self.stop_client)
            self.node.destroy_client(self.physics_grasp_client)
