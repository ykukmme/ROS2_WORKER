import math
import unittest

from ai_worker_sorting.core import Point, DryRobot, pixel_on_plane, pick_and_place


class GeometryTests(unittest.TestCase):
    def project(self, u=320, v=240, rotation=None, plane=0.2, fx=500):
        return pixel_on_plane(u, v, fx, 500, 320, 240, (0, 0, 1),
                              rotation or [[1, 0, 0], [0, -1, 0], [0, 0, -1]], plane)

    def test_camera_looking_down_center(self):
        p = self.project()
        self.assertAlmostEqual(p.x, 0)
        self.assertAlmostEqual(p.y, 0)
        self.assertAlmostEqual(p.z, 0.2)

    def test_off_center_and_rotation(self):
        p = self.project(u=420, v=340)
        self.assertAlmostEqual(p.x, 0.16)
        self.assertAlmostEqual(p.y, -0.16)

    def test_parallel_ray_rejected(self):
        with self.assertRaises(ValueError):
            self.project(rotation=[[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

    def test_plane_behind_camera_rejected(self):
        with self.assertRaises(ValueError):
            self.project(plane=2)

    def test_invalid_intrinsics_rejected(self):
        for fx in (0, -1, math.nan, math.inf):
            with self.subTest(fx=fx), self.assertRaises(ValueError):
                self.project(fx=fx)


class RecordingRobot(DryRobot):
    def __init__(self, fail=None):
        self.events = []
        self.fail = fail

    def record(self, event):
        self.events.append(event)
        if event == self.fail:
            raise RuntimeError('Injected failure')

    def move(self, point):
        self.record(('move', point))

    def gripper(self, opened):
        self.record(('gripper', opened))

    def verify_grasp(self):
        self.record(('verify',))


class TaskTests(unittest.TestCase):
    def run_cycle(self, robot, target=None, check=lambda: None, limits=None):
        states = []
        pick_and_place(robot, target or Point(0.4, 0.1, 0.2),
                       Point(0.35, -0.2, 0.2), Point(0.3, 0, 0.4),
                       limits or [0.1, 0.7, -0.4, 0.4, 0.05, 0.7],
                       0.1, 0.0, check, states.append)
        return states

    def test_complete_cycle_and_lift_before_verification(self):
        robot = RecordingRobot()
        states = self.run_cycle(robot)
        self.assertEqual(states[-1], 'DONE')
        verify = robot.events.index(('verify',))
        self.assertEqual(robot.events[verify-1][0], 'move')
        self.assertAlmostEqual(robot.events[verify-1][1].z, 0.3)
        self.assertEqual(robot.events[-1], ('move', Point(0.3, 0, 0.4)))
        self.assertEqual(sum(e == ('gripper', True) for e in robot.events), 2)

    def test_outside_target_sends_nothing(self):
        robot = RecordingRobot()
        with self.assertRaises(ValueError):
            self.run_cycle(robot, Point(10, 0, 0.2))
        self.assertEqual(robot.events, [])

    def test_nan_target_sends_nothing(self):
        robot = RecordingRobot()
        with self.assertRaises(ValueError):
            self.run_cycle(robot, Point(math.nan, 0, 0.2))
        self.assertEqual(robot.events, [])

    def test_approach_outside_workspace_sends_nothing(self):
        robot = RecordingRobot()
        with self.assertRaises(ValueError):
            self.run_cycle(robot, Point(0.4, 0.1, 0.65))
        self.assertEqual(robot.events, [])

    def test_bin_outside_workspace_sends_nothing(self):
        robot = RecordingRobot()
        with self.assertRaises(ValueError):
            self.run_cycle(robot, limits=[0.1, 0.7, 0, 0.4, 0.05, 0.7])
        self.assertEqual(robot.events, [])

    def test_grasp_failure_never_transfers_or_releases(self):
        robot = RecordingRobot(fail=('verify',))
        with self.assertRaises(RuntimeError):
            self.run_cycle(robot)
        self.assertEqual(robot.events[-1], ('verify',))
        self.assertEqual(sum(e == ('gripper', True) for e in robot.events), 1)

    def test_close_failure_never_lifts(self):
        robot = RecordingRobot(fail=('gripper', False))
        with self.assertRaises(RuntimeError):
            self.run_cycle(robot)
        self.assertEqual(robot.events[-1], ('gripper', False))

    def test_stop_before_start_sends_nothing(self):
        robot = RecordingRobot()
        def stop():
            raise RuntimeError('stopped')
        with self.assertRaises(RuntimeError):
            self.run_cycle(robot, check=stop)
        self.assertEqual(robot.events, [])

    def test_stop_after_close_never_lifts_or_releases(self):
        robot = RecordingRobot()
        def stop():
            if ('gripper', False) in robot.events:
                raise RuntimeError('stopped')
        with self.assertRaises(RuntimeError):
            self.run_cycle(robot, check=stop)
        self.assertEqual(robot.events[-1], ('gripper', False))


if __name__ == '__main__':
    unittest.main()
