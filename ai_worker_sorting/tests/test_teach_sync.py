"""Calibration must replace cached observations without accepting mismatched depth."""
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

try:
    from ai_worker_sorting import configure_auto as calibration
    from builtin_interfaces.msg import Time
except ImportError:
    calibration = None


@unittest.skipIf(calibration is None,'ROS2 calibration dependencies are required')
class TeachingSyncTests(unittest.TestCase):
    def setUp(self):
        self.header = NS(stamp=Time(sec=10),frame_id='camera')
        self.node = Mock()
        self.node.get_clock.return_value.now.return_value.nanoseconds = 10_000_000_000
        self.info = NS(header=self.header,width=640,height=480)
        self.depth = NS(header=self.header,encoding='16UC1')
        self.buffer = Mock()
        self.buffer.can_transform.return_value = True
        zone = NS(simulated=False,localized=True,
                  boundary=NS(header=NS(stamp=Time(sec=10),frame_id='base'),
                              polygon=NS(points=[NS(z=.1)]*4)),
                  pixel_boundary=NS(points=[]))
        self.zones = NS(zones=[zone,zone])
        self.args = NS(base_frame='base')

    def observe(self,stamp,frame='camera'):
        return NS(header=NS(stamp=stamp,frame_id=frame),objects=[])

    def run_scan(self,responses,message):
        with patch.object(calibration,'service',side_effect=[self.zones,*responses]) as service, \
             patch.object(calibration.rclpy,'spin_once') as spin, \
             patch.object(calibration,'CvBridge') as bridge:
            bridge.return_value.imgmsg_to_cv2.return_value.shape = (480,640)
            with self.assertRaisesRegex(ValueError,message):
                calibration.scan(self.node,self.buffer,self.args,self.info,[self.depth])
        return service,spin

    def test_cached_detection_is_replaced_before_geometry(self):
        old = self.observe(Time(sec=9,nanosec=700_000_000))
        current = self.observe(Time(sec=10))
        service,spin = self.run_scan([old,current],'Show exactly two')
        self.assertEqual(service.call_count,3)
        spin.assert_called_once()
        self.buffer.lookup_transform.assert_called_once()

    def test_unmatched_depth_is_still_rejected_at_deadline(self):
        with patch.object(calibration.time,'monotonic',side_effect=[0.,2.]):
            self.run_scan([self.observe(Time(sec=9,nanosec=700_000_000))],
                          'No synchronized complete depth')
        self.buffer.lookup_transform.assert_not_called()

    def test_stale_observation_is_still_rejected(self):
        self.run_scan([self.observe(Time(sec=8))],'Observation/TF is stale')
        self.buffer.lookup_transform.assert_not_called()

    def test_camera_frame_mismatch_is_still_rejected(self):
        self.run_scan([self.observe(Time(sec=10),'other_camera')],'Matching rectified camera')
        self.buffer.lookup_transform.assert_not_called()
