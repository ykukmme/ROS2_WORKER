"""Inspect live webcam topics without writing images or commanding a robot."""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from ai_worker_sorting.ros_camera import video_qos


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Node('laptop_camera_probe')
    stamps = {'raw': set(), 'annotated': set(), 'detections': set()}
    classes = set()
    sizes = {}

    def received(kind, msg):
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        assert msg.header.frame_id == 'laptop_camera_optical_frame'
        stamps[kind].add(stamp)
        if kind == 'detections':
            for detection in msg.detections:
                assert detection.bbox.size_x > 0 and detection.bbox.size_y > 0
                assert 0 <= detection.results[0].hypothesis.score <= 1
                classes.add(detection.results[0].hypothesis.class_id)
        else:
            assert msg.encoding == 'bgr8' and msg.width > 0 and msg.height > 0
            sizes[kind] = [msg.width, msg.height]

    subscriptions = [
        node.create_subscription(Image, '/laptop_camera/image_raw', lambda m: received('raw', m), video_qos()),
        node.create_subscription(Image, '/laptop_camera/image_annotated', lambda m: received('annotated', m), video_qos()),
        node.create_subscription(Detection2DArray, '/laptop_camera/detections', lambda m: received('detections', m), 10),
    ]
    try:
        deadline = time.monotonic()+args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        shared = stamps['raw'] & stamps['annotated'] & stamps['detections']
        report = {
            'mode': 'live_laptop_webcam_ros2_no_robot',
            'frame_counts': {k: len(v) for k, v in stamps.items()},
            'matching_capture_timestamps': len(shared),
            'classes_seen': sorted(classes), 'image_sizes': sizes,
            'result': 'PASS' if len(shared) >= 3 else 'FAIL',
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        assert report['result'] == 'PASS', 'Missing or mismatched live topic output'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
