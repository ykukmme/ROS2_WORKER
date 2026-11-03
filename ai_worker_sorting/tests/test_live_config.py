from pathlib import Path
import tempfile
import unittest
import yaml
from ai_worker_sorting.live_config import validate_live_config


class LiveConfigTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        root = Path(self.directory.name)
        self.weights = root/'model.pt'
        self.weights.write_bytes(b'test-placeholder-not-a-model')
        self.path = root/'live.yaml'
        self.config = {name:{'ros__parameters':{}} for name in
                       ('rgbd_input','rgbd_detector','vision_service','zone_service','worker_service')}
        self.config['rgbd_detector']['ros__parameters']['weights'] = str(self.weights)
        for name in ('vision_service','zone_service'):
            self.config[name]['ros__parameters'] = dict(mock_mode=False,enable_depth=True,calibrated=True,base_frame='base_link')
        self.config['worker_service']['ros__parameters'] = dict(dry_run=False,robot_backend='robotis_movel',
            hardware_configured=True,shared_grasp_verified=True,base_frame='base_link',
            workspace=[.1,.7,-.4,.4,.05,.8],tool_quaternion=[1.,0.,0.,0.],
            source_roi=[0.,0.,640.,480.],object_size=.06,tool_link='end_effector_l_link',
            home_mode='current_tcp',placement_height_mode='object_surface',transport_above_objects=True,
            auto_grasp_offset=-.02,controller_frame='base_link',tool_below_tcp=.02,
            controller_node='/ai_worker_movel_controller')

    def tearDown(self):
        self.directory.cleanup()

    def validate(self):
        self.path.write_text(yaml.safe_dump(self.config),encoding='utf-8')
        return validate_live_config(self.path)

    def test_explicit_real_profile_passes(self):
        self.assertFalse(self.validate()['worker_service']['ros__parameters']['dry_run'])

    def test_mock_or_dry_profile_rejected(self):
        self.config['worker_service']['ros__parameters']['dry_run'] = True
        with self.assertRaises(ValueError):
            self.validate()
        self.config['worker_service']['ros__parameters']['dry_run'] = False
        self.config['vision_service']['ros__parameters']['mock_mode'] = True
        with self.assertRaises(ValueError):
            self.validate()

    def test_missing_weights_and_bad_workspace_rejected(self):
        self.weights.unlink()
        with self.assertRaises(ValueError):
            self.validate()
        self.weights.write_bytes(b'fixture')
        self.config['worker_service']['ros__parameters']['workspace'] = [0.,1.]
        with self.assertRaises(ValueError):
            self.validate()


if __name__=='__main__':
    unittest.main()
