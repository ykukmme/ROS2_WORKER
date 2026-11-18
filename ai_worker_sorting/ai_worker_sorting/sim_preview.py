"""Desktop-only image monitor; it has no robot command publishers."""
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
    rclpy.init()
    node = Node('simulation_preview')
    bridge = CvBridge()
    images = {}
    def receive(name,msg):
        images[name] = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    for name,topic in [('AI WORKER simulation','/sim/image_overview'),
                       ('YOLO camera','/worker_camera/image_annotated'),
                       ('Automatic tape zones','/workspace/image_zones')]:
        node.create_subscription(Image,topic,lambda msg,n=name:receive(n,msg),qos_profile_sensor_data)
    try:
        while rclpy.ok():
            rclpy.spin_once(node,timeout_sec=.02)
            for name,frame in images.items():
                cv2.imshow(name,frame)
            if cv2.waitKey(1)&0xff==27:
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
