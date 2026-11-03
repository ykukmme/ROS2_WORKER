"""Host JPEG -> ROS Image -> YOLO Detection2DArray + annotated Image."""
from pathlib import Path
from collections import OrderedDict
import threading
import time
from urllib.request import Request, build_opener, ProxyHandler

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose


def video_qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE)


class BridgeHTTP:
    def __init__(self, url, token_file):
        self.url = url.rstrip('/')
        self.token = Path(token_file).read_text(encoding='utf-8').strip()
        if len(self.token) < 32:
            raise ValueError('Invalid bridge token')
        self.opener = build_opener(ProxyHandler({}))

    def request(self, endpoint, data=None, sequence=None):
        headers = {'X-Bridge-Token': self.token}
        if data is not None:
            headers['Content-Type'] = 'image/jpeg'
            headers['X-Frame-Sequence'] = str(sequence)
        return self.opener.open(Request(self.url+endpoint, data=data, headers=headers), timeout=1.0)


class HttpCamera(Node):
    def __init__(self):
        super().__init__('laptop_camera_bridge')
        for key, value in dict(bridge_url='http://host.docker.internal:8765',
                               token_file='', stop_file='', fps=10.0, max_frame_age=2.0).items():
            self.declare_parameter(key, value)
        self.http = BridgeHTTP(self.get_parameter('bridge_url').value, self.get_parameter('token_file').value)
        fps = self.get_parameter('fps').value
        self.max_age = self.get_parameter('max_frame_age').value
        if not 0 < fps <= 30 or not 0 < self.max_age <= 10:
            raise ValueError('Invalid FPS or max_frame_age')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/laptop_camera/image_raw', video_qos())
        self.last_sequence = 0
        self.sequence_by_stamp = OrderedDict()
        self.sequence_lock = threading.Lock()
        self.timer = self.create_timer(1/fps, self.fetch)

    def fetch(self):
        try:
            with self.http.request('/frame.jpg') as response:
                sequence = int(response.headers['X-Frame-Sequence'])
                stamp = int(response.headers['X-Capture-Nanoseconds'])
                data = response.read(2_000_001)
            if sequence == self.last_sequence:
                return
            if len(data) > 2_000_000 or not 0 <= (time.time_ns()-stamp)/1e9 <= self.max_age:
                raise ValueError('Oversize or stale frame (check host/container clocks)')
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError('Invalid JPEG')
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.header.frame_id = 'laptop_camera_optical_frame'
            with self.sequence_lock:
                self.sequence_by_stamp[stamp] = sequence
                while len(self.sequence_by_stamp) > 64:
                    self.sequence_by_stamp.popitem(last=False)
            self.last_sequence = sequence
            self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().warning(f'Camera fetch failed: {exc}', throttle_duration_sec=5.0)


class YoloDetector(Node):
    def __init__(self, camera):
        super().__init__('laptop_yolo_detector')
        for key, value in dict(weights='', confidence=0.5, imgsz=416, device='cpu').items():
            self.declare_parameter(key, value)
        weights = Path(self.get_parameter('weights').value)
        if not weights.is_file():
            raise ValueError('weights must be a local file')
        self.confidence = self.get_parameter('confidence').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value
        if not 0 < self.confidence <= 1 or self.imgsz <= 0:
            raise ValueError('Invalid detector settings')
        from ultralytics import YOLO
        self.model = YOLO(str(weights))
        self.camera = camera
        self.bridge = CvBridge()
        self.http = BridgeHTTP(camera.get_parameter('bridge_url').value, camera.get_parameter('token_file').value)
        self.detections = self.create_publisher(Detection2DArray, '/laptop_camera/detections', 10)
        self.annotated = self.create_publisher(Image, '/laptop_camera/image_annotated', video_qos())
        self.sub = self.create_subscription(Image, '/laptop_camera/image_raw', self.detect, video_qos())
        self.count = 0
        self.last_log = 0.0

    def detect(self, msg):
        try:
            capture_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            with self.camera.sequence_lock:
                sequence = self.camera.sequence_by_stamp.get(capture_ns)
            if sequence is None:
                return
            if not 0 <= (time.time_ns()-capture_ns)/1e9 <= 2:
                return
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.model.predict(frame, conf=self.confidence, imgsz=self.imgsz,
                                        device=self.device, verbose=False)[0]
            if not 0 <= (time.time_ns()-capture_ns)/1e9 <= 2:
                return
            output = Detection2DArray()
            output.header = msg.header
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().tolist()
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.position.x = (x1+x2)/2
                detection.bbox.center.position.y = (y1+y2)/2
                detection.bbox.size_x, detection.bbox.size_y = x2-x1, y2-y1
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = result.names[int(box.cls.item())]
                hypothesis.hypothesis.score = float(box.conf.item())
                detection.results = [hypothesis]
                output.detections.append(detection)
            self.detections.publish(output)
            image = result.plot()
            annotated = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            annotated.header = msg.header
            self.annotated.publish(annotated)
            self.count += 1
            if time.monotonic()-self.last_log >= 5:
                names = [d.results[0].hypothesis.class_id for d in output.detections]
                self.get_logger().info(f'YOLO frames={self.count} classes={names}')
                self.last_log = time.monotonic()
            ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.http.request('/annotated.jpg', jpeg.tobytes(), sequence) as response:
                    response.read()
        except Exception as exc:
            self.get_logger().warning(f'Detection/preview failed: {exc}', throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=3)
    nodes = []
    try:
        camera = HttpCamera()
        nodes.append(camera)
        stop_path = camera.get_parameter('stop_file').value
        if stop_path and Path(stop_path).exists():
            raise ValueError('ROS camera stop file already exists')
        detector = YoloDetector(camera)
        nodes.append(detector)
        for node in nodes:
            executor.add_node(node)
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.2)
            if stop_path and Path(stop_path).exists():
                break
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
