"""Rectify RGB and already color-aligned depth using exactly the same pixel map."""
import copy
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer


class AlignedRGBDInput(Node):
    def __init__(self,parameter_overrides=None):
        super().__init__('rgbd_input',parameter_overrides=parameter_overrides)
        defaults = dict(color_topic='/camera/color/image_raw',
                        depth_topic='/camera/aligned_depth_to_color/image_raw',
                        camera_info_topic='/camera/color/camera_info',
                        input_is_rectified=False, sync_slop=.03, max_age=1.0)
        for key,value in defaults.items():
            self.declare_parameter(key,value)
        self.cfg = {key:self.get_parameter(key).value for key in defaults}
        self.bridge,self.lock,self.info = CvBridge(),threading.Lock(),None
        self.maps,self.signature = None,None
        self.color_pub = self.create_publisher(Image,'/worker_camera/image_rect',1)
        self.depth_pub = self.create_publisher(Image,'/worker_camera/depth_rect',1)
        self.info_pub = self.create_publisher(CameraInfo,'/worker_camera/camera_info',1)
        self.info_sub = self.create_subscription(CameraInfo,self.cfg['camera_info_topic'],self.on_info,qos_profile_sensor_data)
        self.color_sub = Subscriber(self,Image,self.cfg['color_topic'],qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self,Image,self.cfg['depth_topic'],qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.color_sub,self.depth_sub],5,self.cfg['sync_slop'])
        self.sync.registerCallback(self.process)

    def on_info(self,msg):
        with self.lock:
            self.info = msg

    def process(self,color,depth):
        try:
            with self.lock:
                info = self.info
            if info is None:
                return
            frame_id = color.header.frame_id
            if not frame_id or depth.header.frame_id!=frame_id or info.header.frame_id!=frame_id:
                raise ValueError('Depth must already be aligned to the RGB optical frame')
            if (color.width,color.height)!=(depth.width,depth.height) or (color.width,color.height)!=(info.width,info.height):
                raise ValueError('RGB, aligned depth and CameraInfo dimensions must match')
            stamp = color.header.stamp.sec*1_000_000_000+color.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if stamp<=0 or not 0<=age<=self.cfg['max_age']:
                return
            if depth.encoding not in ('16UC1','32FC1'):
                raise ValueError('Depth must use 16UC1 or 32FC1')
            rgb = self.bridge.imgmsg_to_cv2(color,desired_encoding='bgr8')
            values = self.bridge.imgmsg_to_cv2(depth,desired_encoding='passthrough')
            matrix = np.asarray(info.k,dtype=float).reshape(3,3)
            if not np.isfinite(matrix).all() or matrix[0,0]<=0 or matrix[1,1]<=0:
                raise ValueError('Invalid camera intrinsics')
            if self.cfg['input_is_rectified']:
                matrix = np.array([[info.p[0],0.,info.p[2]],[0.,info.p[5],info.p[6]],[0.,0.,1.]])
                if matrix[0,0]<=0 or matrix[1,1]<=0 or abs(info.p[3])+abs(info.p[7])>1e-8:
                    raise ValueError('Invalid rectified monocular projection')
                if not np.allclose(np.asarray(info.r).reshape(3,3),np.eye(3)):
                    raise ValueError('Rectified input needs a matching rectified optical frame')
            else:
                if info.distortion_model not in ('','plumb_bob','rational_polynomial'):
                    raise ValueError('Unsupported camera distortion model')
                signature = (color.width,color.height,tuple(info.k),tuple(info.d))
                if signature!=self.signature:
                    distortion = np.asarray(info.d,dtype=float) if info.d else np.zeros(5)
                    self.maps = cv2.initUndistortRectifyMap(matrix,distortion,np.eye(3),matrix,
                                                          (color.width,color.height),cv2.CV_32FC1)
                    self.signature = signature
                rgb = cv2.remap(rgb,*self.maps,cv2.INTER_LINEAR)
                values = cv2.remap(values,*self.maps,cv2.INTER_NEAREST)
            output_info = copy.deepcopy(info)
            output_info.header = color.header
            output_info.k = matrix.reshape(-1).tolist()
            output_info.d = [0.]*5
            output_info.r = np.eye(3).reshape(-1).tolist()
            output_info.p = [float(matrix[0,0]),0.,float(matrix[0,2]),0.,0.,float(matrix[1,1]),
                             float(matrix[1,2]),0.,0.,0.,1.,0.]
            output_info.distortion_model = 'plumb_bob'
            rgb_msg = self.bridge.cv2_to_imgmsg(rgb,encoding='bgr8')
            depth_msg = self.bridge.cv2_to_imgmsg(values,encoding=depth.encoding)
            rgb_msg.header = depth_msg.header = color.header
            self.info_pub.publish(output_info)
            self.depth_pub.publish(depth_msg)
            self.color_pub.publish(rgb_msg)
        except Exception as exc:
            self.get_logger().warning(f'RGB-D frame rejected: {exc}',throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AlignedRGBDInput()
        rclpy.spin(node)
    finally:
        if node:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
