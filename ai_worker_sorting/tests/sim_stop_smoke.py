"""Cancel an executing physical trajectory and reject later queued commands."""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from robotis_interfaces.msg import MoveL

from ai_worker_sorting.core import Point
from ai_worker_sorting.mujoco_robot import MujocoRobot
from ai_worker_sorting.sim_bridge import SimulationBridge
from ai_worker_sorting.sim_profile import profile


def main():
    rclpy.init()
    bridge = SimulationBridge()
    node = Node('simulation_stop_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(bridge)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin,daemon=True)
    spin.start()
    stop_requested = threading.Event()
    def check():
        if stop_requested.is_set():
            raise RuntimeError('Test cancellation requested')
    robot = MujocoRobot(node,profile('/sim-data/simulation_yolo.pt')['worker_service'],check,lambda:None)
    errors = []
    def motion():
        try:
            robot.move(Point(.34,.18,.93))
        except Exception as exc:
            errors.append(str(exc))
    move_thread = threading.Thread(target=motion)
    try:
        move_thread.start()
        deadline = time.monotonic()+15
        while time.monotonic()<deadline:
            with bridge.lock:
                moving = bridge.world.path is not None and bridge.world.data.time-bridge.world.path_started>.3
            if moving:
                break
            time.sleep(.02)
        assert moving,'Motion never began'
        stop_requested.set()
        move_thread.join(timeout=8)
        assert not move_thread.is_alive() and robot.stop_confirmed,errors
        with bridge.lock:
            before = bridge.world.pose()[0]
            assert bridge.world.stopped
        queued = MoveL()
        queued.pose.header.frame_id = 'sim_base'
        queued.pose.pose.position.x,queued.pose.pose.position.y,queued.pose.pose.position.z = .5,.25,.9
        queued.pose.pose.orientation.w = 1.
        queued.time_from_start.sec = 2
        bridge.move(queued)
        time.sleep(.6)
        with bridge.lock:
            after = bridge.world.pose()[0]
            assert bridge.world.path is None
        assert np.linalg.norm(after-before)<.003,(before,after)
        print('SIMULATION_STOP_PASS',flush=True)
    finally:
        stop_requested.set()
        move_thread.join(timeout=8)
        executor.shutdown(timeout_sec=5)
        spin.join(timeout=5)
        robot.destroy()
        node.destroy_node()
        bridge.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
