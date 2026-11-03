"""Synthetic camera/TF -> actual YOLO -> target -> DRY pick cycle.

Use ROS_DOMAIN_ID=77. Projected coordinates have no physical meaning.
"""
import argparse
import json
from pathlib import Path
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from ai_worker_sorting.perception import Perception
from ai_worker_sorting.task_manager import TaskManager


def until(predicate, message, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', required=True)
    parser.add_argument('--image', required=True)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise ValueError('Cannot decode image')
    rclpy.init(args=[])
    perception = Perception(parameter_overrides=[
        Parameter('weights', value=args.weights), Parameter('target_class', value='bus'),
        Parameter('calibrated', value=True), Parameter('max_image_age', value=10.0),
        Parameter('image_topic', value='/sorting_test/image_rect'),
        Parameter('camera_info_topic', value='/sorting_test/camera_info'),
    ])
    task = TaskManager(parameter_overrides=[Parameter('dry_run', value=True),
                                           Parameter('target_max_age', value=10.0)])
    assert task.cfg['dry_run'] is True
    probe = Node('synthetic_camera_probe')
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (perception, task, probe):
        executor.add_node(node)
    states, targets = [], []
    status_sub = probe.create_subscription(String, '/sorting/status', lambda m: states.append(m.data), 20)
    target_sub = probe.create_subscription(PointStamped, '/sorting/target', targets.append, 10)
    image_pub = probe.create_publisher(Image, '/sorting_test/image_rect', qos_profile_sensor_data)
    info_pub = probe.create_publisher(CameraInfo, '/sorting_test/camera_info', qos_profile_sensor_data)
    start = probe.create_client(Trigger, '/sorting/start')
    broadcaster = StaticTransformBroadcaster(probe)
    transform = TransformStamped()
    transform.header.stamp = probe.get_clock().now().to_msg()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'synthetic_camera_optical'
    transform.transform.translation.x = 0.4
    transform.transform.translation.z = 1.0
    transform.transform.rotation.x = 1.0
    transform.transform.rotation.w = 0.0
    broadcaster.sendTransform(transform)
    bridge = CvBridge()
    height, width = image.shape[:2]

    def publish_frame():
        msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = probe.get_clock().now().to_msg()
        msg.header.frame_id = 'synthetic_camera_optical'
        info = CameraInfo()
        info.header = msg.header
        info.width, info.height = width, height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0]*5
        info.k = [500.0, 0.0, width/2, 0.0, 500.0, height/2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [500.0, 0.0, width/2, 0.0, 0.0, 500.0, height/2, 0.0, 0.0, 0.0, 1.0, 0.0]
        image_pub.publish(msg)
        info_pub.publish(info)

    timer = probe.create_timer(0.5, publish_frame)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        until(lambda: targets and task.target is not None, 'No target from YOLO/TF pipeline')
        print(f'Synthetic target: {targets[-1].point}', flush=True)
        assert start.wait_for_service(timeout_sec=5.0)
        future = start.call_async(Trigger.Request())
        until(future.done, 'No start response')
        assert future.result().success, future.result().message
        until(lambda: 'DONE' in states or any(s.startswith('ERROR:') for s in states),
              'No terminal task status')
        assert 'DONE' in states, f'Dry cycle failed: {states}'
        timer.cancel()
        point = targets[-1].point
        report = {
            'mode': 'synthetic_camera_and_tf_dry_robot',
            'physical_calibration_verified': False,
            'detected_class': 'bus',
            'synthetic_target': {'x': point.x, 'y': point.y, 'z': point.z},
            'states': list(states), 'result': 'PASS',
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
    finally:
        timer.cancel()
        task.stop_event.set()
        if task.worker:
            task.worker.join(timeout=3.0)
        executor.shutdown(timeout_sec=5.0)
        thread.join(timeout=5.0)
        for node in (probe, task, perception):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
