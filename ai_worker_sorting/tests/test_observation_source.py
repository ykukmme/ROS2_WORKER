import unittest
from types import SimpleNamespace
from ai_worker_sorting.observation_source import usable_observation


class ObservationSourceTests(unittest.TestCase):
    def test_physics_observation_cannot_drive_hardware(self):
        obj = SimpleNamespace(simulated=True, source_kind='mujoco')
        self.assertFalse(usable_observation(obj,dict(dry_run=False,runtime_mode='hardware')))

    def test_mock_and_hardware_are_not_valid_physics_feedback(self):
        cfg = dict(dry_run=False,runtime_mode='mujoco')
        for kind, simulated in [('mock',True),('hardware',False),('',False),('mujoco',False)]:
            self.assertFalse(usable_observation(SimpleNamespace(source_kind=kind,simulated=simulated),cfg))
        self.assertTrue(usable_observation(SimpleNamespace(source_kind='mujoco',simulated=True),cfg))
