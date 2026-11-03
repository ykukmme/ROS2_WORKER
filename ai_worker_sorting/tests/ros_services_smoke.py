"""Real ROS2 service integration; synthetic observations, DRY robot only."""
import argparse
import json
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from ai_worker_interfaces.srv import SortOnce, TaskStatus, CancelTask, DetectObjects

from ai_worker_sorting.vision_service import VisionService
from ai_worker_sorting.worker_service import WorkerService


def until(predicate, message, timeout=6):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        result = predicate()
        if result:
            return result
        time.sleep(.02)
    raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(args=[])
    vision = VisionService(parameter_overrides=[
        Parameter('mock_mode', value=True), Parameter('enable_depth', value=True),
        Parameter('calibrated', value=True),
        Parameter('detections_topic', value='/service_test/detections'),
        Parameter('depth_topic', value='/service_test/depth'),
        Parameter('camera_info_topic', value='/service_test/info'),
    ])
    worker = WorkerService(parameter_overrides=[Parameter('dry_step_delay', value=.05),
                                                Parameter('service_timeout', value=.5)])
    probe = Node('worker_services_probe')
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (vision, worker, probe):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    sort = probe.create_client(SortOnce, '/worker/sort_once')
    status = probe.create_client(TaskStatus, '/worker/task_status')
    cancel = probe.create_client(CancelTask, '/worker/cancel_task')
    detect = probe.create_client(DetectObjects, '/vision/detect_objects')
    cases = []
    timers = []

    def call(client, request):
        assert client.wait_for_service(timeout_sec=3)
        future = client.call_async(request)
        until(future.done, 'Service callback stalled')
        return future.result()

    def state(task_id):
        return call(status, TaskStatus.Request(task_id=task_id))

    def submit(name='box'):
        result = call(sort, SortOnce.Request(target_class=name))
        assert result.accepted, result.message
        return result.task_id

    def terminal(task_id, expected):
        result = until(lambda: (r if (r := state(task_id)).state in
                                ('SUCCEEDED', 'FAILED', 'CANCELED') else None),
                       'Task did not terminate')
        assert result.state == expected, (result.state, result.message)
        return result

    try:
        assert not call(sort, SortOnce.Request(target_class='unsupported')).accepted
        assert not state('missing').found
        assert not call(cancel, CancelTask.Request(task_id='missing')).accepted
        cases.append('unknown class/task rejected')

        first = submit()
        assert not call(sort, SortOnce.Request(target_class='box')).accepted
        terminal(first, 'SUCCEEDED')
        assert not call(cancel, CancelTask.Request(task_id=first)).accepted
        terminal(submit('bottle'), 'SUCCEEDED')
        assert worker.classes['box']['bin_position'] != worker.classes['bottle']['bin_position']
        cases.append('asynchronous success, class bins, busy and terminal-cancel rejection')

        worker.cfg['dry_step_delay'] = .3
        task_id = submit()
        until(lambda: state(task_id).stage == 'APPROACH', 'Task did not approach')
        assert call(cancel, CancelTask.Request(task_id=task_id)).accepted
        terminal(task_id, 'CANCELED')
        worker.cfg['dry_step_delay'] = .05
        cases.append('cancel during motion stage stops dry sequence')

        vision.cfg['mock_age'] = 10.0
        terminal(submit(), 'FAILED')
        vision.cfg['mock_age'] = 0.0
        vision.cfg['mock_position'] = [10.0, 0.0, .2]
        terminal(submit(), 'FAILED')
        vision.cfg['mock_position'] = [.4, .1, .15]
        cases.append('stale and out-of-workspace observations rejected')

        # Exercise simulated-observation rejection before any real adapter exists.
        worker.cfg['dry_run'] = False
        terminal(submit(), 'FAILED')
        assert worker.robot is None
        worker.cfg['dry_run'] = True
        cases.append('mock target cannot trigger hardware mode')

        vision.destroy_service(vision.service)
        terminal(submit(), 'FAILED')
        vision.service = vision.create_service(DetectObjects, '/vision/detect_objects', vision.detect)
        cases.append('vision service timeout becomes FAILED')

        vision.cfg['mock_mode'] = False
        vision.cfg['enable_depth'] = False
        detection_pub = probe.create_publisher(Detection2DArray, '/service_test/detections', 10)
        depth_pub = probe.create_publisher(Image, '/service_test/depth', qos_profile_sensor_data)
        info_pub = probe.create_publisher(CameraInfo, '/service_test/info', qos_profile_sensor_data)
        bridge = CvBridge()
        broadcaster = StaticTransformBroadcaster(probe)
        tf = TransformStamped()
        tf.header.stamp = probe.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = 'base_link', 'service_test_optical'
        tf.transform.translation.x, tf.transform.translation.z = .4, 1.0
        tf.transform.rotation.x, tf.transform.rotation.w = 1.0, 0.0
        broadcaster.sendTransform(tf)
        until(lambda: vision.tf.can_transform('base_link', 'service_test_optical', Time()), 'TF missing')
        invalid_depth = [False]

        def publish():
            detections = Detection2DArray()
            detections.header.stamp = probe.get_clock().now().to_msg()
            detections.header.frame_id = 'service_test_optical'
            d = Detection2D()
            d.header = detections.header
            d.bbox.center.position.x, d.bbox.center.position.y = 320., 240.
            d.bbox.size_x, d.bbox.size_y = 100., 80.
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id, h.hypothesis.score = 'box', .95
            d.results = [h]
            detections.detections = [d]
            depth = bridge.cv2_to_imgmsg(np.full((480, 640), 0 if invalid_depth[0] else 800,
                                                 dtype=np.uint16), encoding='16UC1')
            depth.header = detections.header
            info = CameraInfo()
            info.header = detections.header
            info.width, info.height = 640, 480
            info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            info.p = [500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.]
            depth_pub.publish(depth)
            info_pub.publish(info)
            detection_pub.publish(detections)

        until(lambda: depth_pub.get_subscription_count() and detection_pub.get_subscription_count(),
              'Sensor subscriptions missing')
        timer = probe.create_timer(.1, publish)
        timers.append(timer)
        until(lambda: vision.latest is not None, 'Detections missing')
        two_d = call(detect, DetectObjects.Request(target_class='box'))
        assert two_d.success and two_d.objects and not two_d.objects[0].localized
        terminal(submit(), 'FAILED')
        cases.append('fresh 2D detection served but cannot start a grasp')

        vision.cfg['enable_depth'] = True
        def localized():
            r = call(detect, DetectObjects.Request(target_class='box'))
            return r.objects[0] if r.success and r.objects and r.objects[0].localized else None
        obj = until(localized, 'No localized RGB-D observation')
        p = obj.surface_pose.pose.position
        np.testing.assert_allclose([p.x, p.y, p.z], [.4, 0., .2], atol=1e-8)
        terminal(submit(), 'SUCCEEDED')
        cases.append('synthetic RGB-D + TF -> service -> complete dry pick/place')

        invalid_depth[0] = True
        until(lambda: vision.depths and np.max(bridge.imgmsg_to_cv2(vision.depths[-1])) == 0,
              'Invalid depth not received')
        terminal(submit(), 'FAILED')
        cases.append('invalid depth cannot become a grasp target')

        report = {'result': 'PASS', 'mode': 'synthetic_observations_dry_robot_only', 'cases': cases}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
    finally:
        for timer in timers:
            timer.cancel()
        worker.shutdown_event.set()
        if worker.thread:
            worker.thread.join(timeout=3)
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=5)
        for node in (probe, worker, vision):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
