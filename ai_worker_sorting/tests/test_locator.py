import unittest
import numpy as np
from ai_worker_sorting.locator import depth_surface, transform_point


class DepthTests(unittest.TestCase):
    def locate(self, depth, encoding='16UC1', bbox=(50., 50., 40., 40.)):
        return depth_surface(depth, encoding, bbox, (100., 100., 50., 50.))

    def test_mm_depth_and_camera_to_base(self):
        point = self.locate(np.full((100, 100), 800, dtype=np.uint16))
        np.testing.assert_allclose(point, [0, 0, .8])
        np.testing.assert_allclose(transform_point(point, [.4, 0, 1], [1, 0, 0, 0]), [.4, 0, .2])

    def test_float_metres_and_noncenter_pixel(self):
        point = self.locate(np.full((100, 100), .8, dtype=np.float32), '32FC1', (60., 40., 20., 20.))
        np.testing.assert_allclose(point, [.08, -.08, .8], atol=1e-7)

    def test_zero_nan_and_sparse_depth_rejected(self):
        for value in (0, np.nan, np.inf, 5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.locate(np.full((100, 100), value), '32FC1')
        depth = np.zeros((100, 100), dtype=np.uint16)
        depth[50, 50] = 800
        with self.assertRaises(ValueError):
            self.locate(depth)

    def test_discontinuous_depth_rejected(self):
        depth = np.full((100, 100), 800, dtype=np.uint16)
        depth[42:51, 42:59] = 1200
        with self.assertRaises(ValueError):
            self.locate(depth)

    def test_out_of_image_and_unknown_encoding(self):
        depth = np.ones((100, 100), dtype=np.uint16)
        with self.assertRaises(ValueError):
            self.locate(depth, bbox=(1000., 20., 20., 20.))
        with self.assertRaises(ValueError):
            self.locate(depth, encoding='8UC1')


if __name__ == '__main__':
    unittest.main()
