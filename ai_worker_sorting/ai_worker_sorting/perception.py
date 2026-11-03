"""YOLO + calibrated fixed-height plane. No robot commands are sent here."""
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener

from .core import pixel_on_plane


def rotation_matrix(q):
    a = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(a)
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = a / norm
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


class Perception(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('perception', parameter_overrides=parameter_overrides)
        defaults = dict(image_topic='/central_camera/image_rect',
                        camera_info_topic='/central_camera/camera_info',
                        base_frame='base_link', weights='', target_class='box',
                        confidence=0.8, device='cpu', calibrated=False,
                        object_top_z=0.15, max_image_age=1.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        if not self.cfg['calibrated']:
            raise ValueError('Calibrate camera-to-base TF and object plane; set calibrated: true')
        weights = Path(self.cfg['weights'])
        if not weights.is_file():
            raise ValueError('weights must point to an existing local YOLO model')
        if not 0 < self.cfg['confidence'] <= 1 or self.cfg['max_image_age'] <= 0:
            raise ValueError('Invalid confidence or max_image_age')
        from ultralytics import YOLO
        self.model = YOLO(str(weights))
        if self.cfg['target_class'] not in self.model.names.values():
            raise ValueError(f"Model has no class: {self.cfg['target_class']}")
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(PointStamped, '/sorting/target', 1)
        self.rgb = Subscriber(self, Image, self.cfg['image_topic'], qos_profile=qos_profile_sensor_data)
        self.info = Subscriber(self, CameraInfo, self.cfg['camera_info_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.rgb, self.info], queue_size=3, slop=0.05)
        self.sync.registerCallback(self.detect)

    def detect(self, image, info):
        try:
            stamp = Time.from_msg(image.header.stamp)
            age = (self.get_clock().now()-stamp).nanoseconds / 1e9
            if stamp.nanoseconds == 0 or not 0 <= age <= self.cfg['max_image_age']:
                return
            if not image.header.frame_id or image.header.frame_id != info.header.frame_id:
                raise ValueError('Image and CameraInfo optical frames must match')
            if (image.width, image.height) != (info.width, info.height):
                raise ValueError('Image and CameraInfo sizes must match')
            # This prototype supports monocular rectified images, no stereo shift.
            if not np.allclose(np.array(info.r).reshape(3, 3), np.eye(3), atol=1e-6):
                raise ValueError('Nonidentity rectification is unsupported')
            if abs(info.p[3]) > 1e-8 or abs(info.p[7]) > 1e-8:
                raise ValueError('Stereo projection offset is unsupported')
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            result = self.model.predict(frame, conf=self.cfg['confidence'],
                                        device=self.cfg['device'], verbose=False)[0]
            candidates = [b for b in result.boxes if
                          result.names[int(b.cls.item())] == self.cfg['target_class']]
            if not candidates:
                return
            box = max(candidates, key=lambda b: float(b.conf.item()))
            x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
            u, v = (x1+x2)/2, (y1+y2)/2
            if not (0 <= u < image.width and 0 <= v < image.height):
                raise ValueError('Detection outside image')
            # Exact acquisition time, never a latest-time transform for old pixels.
            transform = self.tf.lookup_transform(self.cfg['base_frame'], image.header.frame_id,
                                                 stamp, timeout=Duration(seconds=0.0)).transform
            t = transform.translation
            point = pixel_on_plane(u, v, info.p[0], info.p[5], info.p[2], info.p[6],
                                   (t.x, t.y, t.z), rotation_matrix(transform.rotation),
                                   self.cfg['object_top_z'])
            # Inference itself may have made the observation stale.
            age = (self.get_clock().now()-stamp).nanoseconds / 1e9
            if not 0 <= age <= self.cfg['max_image_age']:
                return
            msg = PointStamped()
            msg.header.stamp = image.header.stamp
            msg.header.frame_id = self.cfg['base_frame']
            msg.point.x, msg.point.y, msg.point.z = point.x, point.y, point.z
            self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Perception rejected frame: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Perception()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
