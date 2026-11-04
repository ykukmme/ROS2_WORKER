import math
import unittest
import numpy as np
from ai_worker_sorting.auto_geometry import footprint,teaching_offset,depth_top


class AutoGeometryTests(unittest.TestCase):
    def test_high_edge_is_used_instead_of_lower_central_surface(self):
        depth = np.full((60,60),800,dtype=np.uint16)
        depth[10,10] = 600
        top = depth_top(depth,'16UC1',(30.,30.,40.,40.),(100.,100.,30.,30.),
                        (0.,0.,1.),(1.,0.,0.,0.),.1)
        self.assertAlmostEqual(top,.4)

    def test_missing_edge_depth_is_rejected(self):
        depth = np.full((60,60),800,dtype=np.uint16)
        depth[10,10] = 0
        with self.assertRaises(ValueError):
            depth_top(depth,'16UC1',(30.,30.,40.,40.),(100.,100.,30.,30.),
                      (0.,0.,1.),(1.,0.,0.,0.),.1)

    def test_complete_box_bounds_include_table_and_surface(self):
        size = footprint((320.,240.,50.,40.),(500.,500.,320.,240.),(0.,0.,1.),
                         (1.,0.,0.,0.),.1,.2,(640,480))
        self.assertAlmostEqual(size,.09)

    def test_truncated_object_is_rejected(self):
        with self.assertRaises(ValueError):
            footprint((10.,240.,50.,40.),(500.,500.,320.,240.),(0.,0.,1.),(1.,0.,0.,0.),.1,.2,(640,480))

    def test_camera_parallel_to_table_is_rejected(self):
        with self.assertRaises(ValueError):
            footprint((320.,240.,50.,40.),(500.,500.,320.,240.),(0.,0.,1.),
                      (0.,math.sqrt(.5),0.,math.sqrt(.5)),.1,.2,(640,480))

    def test_teaching_converts_pose_to_surface_offset(self):
        self.assertAlmostEqual(teaching_offset((.2,0.,.2),(.2,0.,.23),.1,.05),.03)

    def test_tool_table_contact_and_wrong_object_are_rejected(self):
        for tcp in ((.2,0.,.15),(.5,0.,.23)):
            with self.assertRaises(ValueError):
                teaching_offset((.2,0.,.2),tcp,.1,.05)


if __name__=='__main__':
    unittest.main()
