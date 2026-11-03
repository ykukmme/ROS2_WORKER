"""YOLO on a real ROS2 camera topic. No Windows HTTP bridge or mock detections."""
from pathlib import Path
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge


class RGBDDetector(Node):
    def __init__(self,parameter_overrides=None,model=None):
        super().__init__('rgbd_detector',parameter_overrides=parameter_overrides)
        defaults = dict(image_topic='/worker_camera/image_rect',weights='',confidence=.8,
                        imgsz=640,device='0',max_age=1.,inference_hz=10.,excluded_classes=['person'])
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.cfg = {key:self.get_parameter(key).value for key in defaults}
        if not 0<self.cfg['confidence']<=1 or not 0<self.cfg['inference_hz']<=60:
            raise ValueError('Invalid detection confidence or inference rate')
        if model is None:
            if not Path(self.cfg['weights']).is_file():
                raise ValueError('weights must be an existing local YOLO model')
            from ultralytics import YOLO
            model = YOLO(self.cfg['weights'])
        self.model,self.bridge = model,CvBridge()
        self.lock,self.latest,self.processed = threading.Lock(),None,None
        self.sub = self.create_subscription(Image,self.cfg['image_topic'],self.on_image,qos_profile_sensor_data)
        self.pub = self.create_publisher(Detection2DArray,'/worker_camera/detections',10)
        self.annotated = self.create_publisher(Image,'/worker_camera/image_annotated',1)
        self.inference_group = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(1/self.cfg['inference_hz'],self.infer,callback_group=self.inference_group)

    def on_image(self,msg):
        with self.lock:
            self.latest = msg

    def fresh(self,msg):
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        return stamp>0 and 0<=(self.get_clock().now().nanoseconds-stamp)/1e9<=self.cfg['max_age']

    def infer(self):
        with self.lock:
            msg = self.latest
        if msg is None or msg is self.processed or not self.fresh(msg):
            return
        self.processed = msg
        try:
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            result = self.model.predict(image,conf=self.cfg['confidence'],imgsz=self.cfg['imgsz'],
                                        device=self.cfg['device'],verbose=False)[0]
            if not self.fresh(msg):
                return
            detections = Detection2DArray()
            detections.header = msg.header
            for box in result.boxes:
                label = result.names[int(box.cls.item())]
                if label in self.cfg['excluded_classes']:
                    continue
                x1,y1,x2,y2 = box.xyxy[0].cpu().tolist()
                detection = Detection2D()
                detection.header = msg.header
                detection.bbox.center.position.x,detection.bbox.center.position.y = (x1+x2)/2,(y1+y2)/2
                detection.bbox.size_x,detection.bbox.size_y = x2-x1,y2-y1
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = label
                hypothesis.hypothesis.score = float(box.conf.item())
                detection.results = [hypothesis]
                detections.detections.append(detection)
            self.pub.publish(detections)
            output = self.bridge.cv2_to_imgmsg(result.plot(),encoding='bgr8')
            output.header = msg.header
            self.annotated.publish(output)
        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}',throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=2)
    try:
        node = RGBDDetector()
        executor.add_node(node)
        executor.spin()
    finally:
        executor.shutdown()
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
