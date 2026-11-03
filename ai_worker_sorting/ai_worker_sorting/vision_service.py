"""Serve cached 2D detections; optional calibrated RGB-D surface localization."""
from collections import deque
from pathlib import Path
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from ai_worker_interfaces.msg import DetectedObject
from ai_worker_interfaces.srv import DetectObjects

from .locator import depth_surface, transform_point


def stamp_ns(header):
    return header.stamp.sec*1_000_000_000 + header.stamp.nanosec


class VisionService(Node):
    def __init__(self, parameter_overrides=None, node_name='vision_service', expose_service=True):
        super().__init__(node_name, parameter_overrides=parameter_overrides)
        defaults = dict(mock_mode=False, mock_classes=['box', 'bottle'],
                        mock_position=[0.4, 0.1, 0.15], mock_age=0.0,
                        detections_topic='/laptop_camera/detections',
                        enable_depth=False, calibrated=False, base_frame='base_link',
                        depth_topic='/camera/aligned_depth_to_color/image_raw',
                        camera_info_topic='/camera/color/camera_info',
                        max_age=1.0, sync_slop=0.06, scale_16u=0.001,
                        min_depth=0.1, max_depth=2.0, stop_file='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if not 0 < self.cfg['max_age'] <= 10 or not 0 <= self.cfg['sync_slop'] <= .2:
            raise ValueError('Invalid observation age or synchronization tolerance')
        self.lock = threading.Lock()
        self.latest = None
        self.depths, self.infos = deque(maxlen=60), deque(maxlen=60)
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.detection_sub = self.create_subscription(
            Detection2DArray, self.cfg['detections_topic'], self.on_detections, qos_profile_sensor_data)
        if self.cfg['enable_depth']:
            if not self.cfg['calibrated']:
                raise ValueError('Depth localization requires calibrated: true')
            self.depth_sub = self.create_subscription(Image, self.cfg['depth_topic'],
                                                     self.on_depth, qos_profile_sensor_data)
            self.info_sub = self.create_subscription(CameraInfo, self.cfg['camera_info_topic'],
                                                    self.on_info, qos_profile_sensor_data)
        if expose_service:
            self.service = self.create_service(DetectObjects, '/vision/detect_objects', self.detect)
        self.get_logger().info(f"Vision ready; mock_mode={self.cfg['mock_mode']}, depth={self.cfg['enable_depth']}")

    def on_detections(self, msg):
        with self.lock:
            self.latest = msg

    def on_depth(self, msg):
        with self.lock:
            self.depths.append(msg)

    def on_info(self, msg):
        with self.lock:
            self.infos.append(msg)

    def mock(self, target_class):
        if not target_class:
            objects = []
            for index, name in enumerate(self.cfg['mock_classes']):
                for copy in range(2):
                    obj = self.mock(name)[0]
                    obj.target_id = f'mock-{name}-{copy}'
                    obj.surface_pose.pose.position.x = .2 + .07*copy
                    obj.surface_pose.pose.position.y = -.05 + .1*index
                    obj.bbox.center.position.x = 100. + 80.*copy
                    obj.bbox.center.position.y = 100. + 90.*index
                    obj.bbox.size_x, obj.bbox.size_y = 40., 40.
                    objects.append(obj)
            return objects
        if target_class not in self.cfg['mock_classes']:
            return []
        obj = DetectedObject()
        obj.class_name, obj.confidence = target_class, 1.0
        obj.localized, obj.simulated = True, True
        obj.surface_pose.header.frame_id = self.cfg['base_frame']
        stamp = self.get_clock().now()-Duration(seconds=self.cfg['mock_age'])
        obj.surface_pose.header.stamp = stamp.to_msg()
        obj.target_id = f'mock-{stamp.nanoseconds}'
        p = obj.surface_pose.pose.position
        p.x, p.y, p.z = self.cfg['mock_position']
        obj.surface_pose.pose.orientation.w = 1.0
        obj.localization_message = 'Synthetic surface point, NOT a camera observation'
        return [obj]

    def localize(self, detection, header, depths, infos):
        if not self.cfg['enable_depth']:
            raise ValueError('2D only; calibrated RGB-D localization is disabled')
        stamp = stamp_ns(header)
        candidates = [m for m in depths if m.header.frame_id == header.frame_id
                      and abs(stamp_ns(m.header)-stamp) <= self.cfg['sync_slop']*1e9]
        if not candidates:
            raise ValueError('No time-aligned depth in the detection optical frame')
        depth_msg = min(candidates, key=lambda m: abs(stamp_ns(m.header)-stamp))
        infos = [m for m in infos if m.header.frame_id == header.frame_id
                 and abs(stamp_ns(m.header)-stamp) <= 1e9
                 and (m.width, m.height) == (depth_msg.width, depth_msg.height)]
        if not infos:
            raise ValueError('No matching CameraInfo')
        info = min(infos, key=lambda m: abs(stamp_ns(m.header)-stamp))
        if not np.allclose(np.array(info.r).reshape(3, 3), np.eye(3), atol=1e-6):
            raise ValueError('Only rectified monocular optical frames are supported')
        if abs(info.p[3]) > 1e-8 or abs(info.p[7]) > 1e-8:
            raise ValueError('Stereo projection shift is unsupported')
        b = detection.bbox
        point = depth_surface(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'), depth_msg.encoding,
            (b.center.position.x, b.center.position.y, b.size_x, b.size_y),
            (info.p[0], info.p[5], info.p[2], info.p[6]),
            self.cfg['scale_16u'], self.cfg['min_depth'], self.cfg['max_depth'])
        transform = self.tf.lookup_transform(self.cfg['base_frame'], header.frame_id,
                                             Time.from_msg(header.stamp)).transform
        t, q = transform.translation, transform.rotation
        point = transform_point(point, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = self.cfg['base_frame'], header.stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, point)
        pose.pose.orientation.w = 1.0  # No object rotation estimate.
        return pose

    def detect(self, request, response):
        if self.cfg['mock_mode']:
            response.objects = self.mock(request.target_class)
            response.header.stamp = self.get_clock().now().to_msg()
            response.header.frame_id = self.cfg['base_frame']
            response.success, response.message = True, 'MOCK observations only'
            return response
        with self.lock:
            msg, depths, infos = self.latest, list(self.depths), list(self.infos)
        if msg is None:
            response.message = 'No detections received'
            return response
        response.header = msg.header
        age = (self.get_clock().now().nanoseconds-stamp_ns(msg.header))/1e9
        if stamp_ns(msg.header) <= 0 or not 0 <= age <= self.cfg['max_age']:
            response.message = 'Detection snapshot stale or future-dated'
            return response
        for index, detection in enumerate(msg.detections):
            matches = [h.hypothesis for h in detection.results
                       if not request.target_class or h.hypothesis.class_id == request.target_class]
            if not matches:
                continue
            obj = DetectedObject()
            obj.target_id = f'{stamp_ns(msg.header)}-{index}'
            best = max(matches, key=lambda h: h.score)
            obj.class_name, obj.confidence = best.class_id, best.score
            obj.bbox = detection.bbox
            # Retain acquisition stamp even when no 3D point is available.
            obj.surface_pose.header = msg.header
            try:
                obj.surface_pose = self.localize(detection, msg.header, depths, infos)
                obj.localized = True
                obj.localization_message = 'Calibrated depth surface point; orientation not estimated'
            except Exception as exc:
                obj.localization_message = str(exc)
            response.objects.append(obj)
        response.success, response.message = True, 'Fresh detections; check localized for usable 3D points'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionService()
        stop_file = node.cfg['stop_file']
        if stop_file and Path(stop_file).exists():
            raise ValueError('Vision stop file already exists')
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
            if stop_file and Path(stop_file).exists():
                break
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
