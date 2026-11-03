"""Register two closed tape rectangles; localize with aligned depth when available."""
import uuid
import numpy as np
import cv2
import rclpy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from vision_msgs.msg import Detection2D
from ai_worker_interfaces.msg import PlacementZone
from ai_worker_interfaces.srv import ScanZones

from .vision_service import VisionService, stamp_ns
from .zones import tape_zones, validate_quad


class ZoneService(VisionService):
    def __init__(self, parameter_overrides=None):
        super().__init__(parameter_overrides, node_name='zone_service', expose_service=False)
        for key, value in dict(image_topic='/laptop_camera/image_raw',
                               zone_colors=['red', 'blue'], min_zone_area=1500.).items():
            self.declare_parameter(key, value)
            self.cfg[key] = self.get_parameter(key).value
        if len(set(self.cfg['zone_colors'])) != 2:
            raise ValueError('Configure exactly two different tape colors')
        self.image = None
        self.image_sub = self.create_subscription(Image, self.cfg['image_topic'], self.on_image,
                                                  qos_profile_sensor_data)
        self.overlay = self.create_publisher(Image, '/workspace/image_zones', 1)
        self.service = self.create_service(ScanZones, '/workspace/scan_zones', self.scan)

    def on_image(self, msg):
        with self.lock:
            self.image = msg

    def mock_zones(self):
        zones = []
        for index, color in enumerate(sorted(self.cfg['zone_colors'])):
            zone = PlacementZone()
            zone.zone_id, zone.color = color+'_zone', color
            zone.localized, zone.simulated = True, True
            zone.image_header.stamp = self.get_clock().now().to_msg()
            zone.image_header.frame_id = 'mock_camera_optical'
            zone.boundary.header.stamp = zone.image_header.stamp
            zone.boundary.header.frame_id = self.cfg['base_frame']
            y0 = -.35 if index == 0 else .1
            zone.boundary.polygon.points = [Point32(x=float(x), y=float(y), z=.1)
                                            for x, y in ((.4,y0),(.65,y0),(.65,y0+.25),(.4,y0+.25))]
            v0 = 20. if index == 0 else 200.
            zone.pixel_boundary.points = [Point32(x=x, y=y, z=0.)
                                           for x, y in ((400.,v0),(620.,v0),(620.,v0+140.),(400.,v0+140.))]
            zone.message = 'MOCK boundary; not measured from tape'
            zones.append(zone)
        return zones

    def scan(self, request, response):
        try:
            if self.cfg['mock_mode']:
                response.zones = self.mock_zones()
            else:
                with self.lock:
                    msg, depths, infos = self.image, list(self.depths), list(self.infos)
                if msg is None:
                    raise ValueError('No camera image')
                age = (self.get_clock().now().nanoseconds-stamp_ns(msg.header))/1e9
                if stamp_ns(msg.header) <= 0 or not 0 <= age <= self.cfg['max_age']:
                    raise ValueError('Zone image is stale')
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
                polygons = tape_zones(frame, self.cfg['zone_colors'], self.cfg['min_zone_area'])
                for color, polygon in sorted(polygons.items()):
                    zone = PlacementZone()
                    zone.zone_id, zone.color, zone.image_header = color+'_zone', color, msg.header
                    zone.pixel_boundary.points = [Point32(x=x, y=y, z=0.) for x, y in polygon]
                    try:
                        # Inset corners to sample the table inside the tape, not its edge.
                        center = np.mean(polygon, axis=0)
                        for corner in polygon:
                            u, v = .95*np.array(corner)+.05*center
                            detection = Detection2D()
                            detection.bbox.center.position.x, detection.bbox.center.position.y = float(u), float(v)
                            detection.bbox.size_x, detection.bbox.size_y = 24., 24.
                            pose = self.localize(detection, msg.header, depths, infos)
                            p = pose.pose.position
                            zone.boundary.polygon.points.append(Point32(x=p.x, y=p.y, z=p.z))
                            zone.boundary.header = pose.header
                        points = [(p.x,p.y,p.z) for p in zone.boundary.polygon.points]
                        validate_quad(points)
                        if max(p[2] for p in points)-min(p[2] for p in points) > .015:
                            raise ValueError('Zone corners are not on a horizontal table')
                        zone.localized, zone.message = True, 'Calibrated tabletop corners'
                    except Exception as exc:
                        zone.boundary.polygon.points = []
                        zone.message = str(exc)
                    response.zones.append(zone)
                    contour = np.array(polygon, dtype=np.int32)
                    cv2.polylines(frame, [contour], True, (0,255,0), 2)
                    cv2.putText(frame, color, tuple(contour[0]), cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 2)
                overlay = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                overlay.header = msg.header
                self.overlay.publish(overlay)
            response.workspace_id = uuid.uuid4().hex
            response.success, response.message = True, 'Zones scanned; localized=false permits preview only'
        except Exception as exc:
            response.zones = []
            response.message = str(exc)
        return response


def main(args=None):
    from pathlib import Path
    rclpy.init(args=args)
    node = None
    try:
        node = ZoneService()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
            if node.cfg['stop_file'] and Path(node.cfg['stop_file']).exists():
                break
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
