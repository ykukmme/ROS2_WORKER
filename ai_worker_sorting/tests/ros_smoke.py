"""Run inside a sourced ROS2 environment; never connects to robot actions."""
import threading
import time

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ai_worker_sorting.task_manager import TaskManager


def until(predicate, message, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(message)


def main():
    rclpy.init()
    task = TaskManager()
    assert task.cfg['dry_run'] is True
    probe = Node('sorting_smoke_probe')
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(task)
    executor.add_node(probe)
    states = []
    subscription = probe.create_subscription(String, '/sorting/status', lambda msg: states.append(msg.data), 20)
    publisher = probe.create_publisher(PointStamped, '/sorting/target', 1)
    start = probe.create_client(Trigger, '/sorting/start')
    stop = probe.create_client(Trigger, '/sorting/stop')
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def call(client):
        assert client.wait_for_service(timeout_sec=5.0)
        future = client.call_async(Trigger.Request())
        until(future.done, 'Service response timeout')
        return future.result()

    def publish(frame, stale=False):
        msg = PointStamped()
        msg.header.frame_id = frame
        msg.header.stamp = probe.get_clock().now().to_msg()
        if stale:
            msg.header.stamp.sec -= 10
        msg.point.x, msg.point.y, msg.point.z = 0.4, 0.1, 0.15
        publisher.publish(msg)
        until(lambda: task.target is not None and task.target.header == msg.header,
              'Target subscription timeout')

    try:
        until(lambda: publisher.get_subscription_count() > 0, 'No target subscription')
        until(lambda: task.status.get_subscription_count() > 0, 'No status subscription')
        assert not call(start).success, 'Must reject missing target'
        publish('wrong_frame')
        assert not call(start).success, 'Must reject wrong frame'
        publish('base_link', stale=True)
        assert not call(start).success, 'Must reject stale target'
        publish('base_link')
        assert call(start).success, 'Fresh target should start'
        until(lambda: 'DONE' in states, f'Cycle failed: {states}')
        assert states.index('VERIFY_GRASP') < states.index('RELEASE')
        assert call(stop).success
        assert not call(start).success, 'Stop must latch until restart'
        print('PASS: missing/wrong-frame/stale targets rejected; dry cycle DONE; stop latched')
    finally:
        task.stop_event.set()
        if task.worker:
            task.worker.join(timeout=3.0)
        executor.shutdown(timeout_sec=3.0)
        thread.join(timeout=3.0)
        probe.destroy_node()
        task.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
