import unittest
import cv2
import numpy as np
from ai_worker_sorting.zones import tape_zones, grid_slots, AutoSession, inside, overlap


def tape_image():
    image = np.full((360, 640, 3), 220, dtype=np.uint8)
    cv2.rectangle(image, (400,20), (620,160), (0,0,255), 8)
    cv2.rectangle(image, (400,200), (620,340), (255,0,0), 8)
    return image


class ZoneTests(unittest.TestCase):
    def test_closed_color_tape_interiors_detected(self):
        zones = tape_zones(tape_image())
        self.assertEqual(set(zones), {'red','blue'})
        self.assertTrue(inside((500,90), zones['red']))
        self.assertFalse(inside((100,90), zones['red']))

    def test_missing_open_and_duplicate_colors_rejected(self):
        image = tape_image()
        image[10:35,470:530] = 220
        with self.assertRaises(ValueError):
            tape_zones(image)
        image = tape_image()
        cv2.rectangle(image, (20,20), (200,160), (0,0,255), 8)
        with self.assertRaises(ValueError):
            tape_zones(image)

    def test_solid_color_patch_is_not_a_zone(self):
        image = tape_image()
        cv2.rectangle(image, (400,20), (620,160), (0,0,255), -1)
        with self.assertRaises(ValueError):
            tape_zones(image)

    def test_polygon_overlap_detected(self):
        a = [(0,0),(1,0),(1,1),(0,1)]
        self.assertTrue(overlap(a,[(.5,.5),(2,.5),(2,2),(.5,2)]))
        self.assertFalse(overlap(a,[(2,0),(3,0),(3,1),(2,1)]))

    def test_slots_preserve_boundary_margin(self):
        polygon = [(.4,.1,.1),(.65,.1,.1),(.65,.35,.1),(.4,.35,.1)]
        slots = grid_slots('red', polygon)
        self.assertEqual(len(slots), 9)
        for slot in slots:
            self.assertTrue(inside((slot.position.x-.04,slot.position.y-.04),polygon))
            self.assertTrue(inside((slot.position.x+.04,slot.position.y+.04),polygon))
            self.assertAlmostEqual(slot.position.z,.15)

    def test_small_sloping_and_degenerate_zones_rejected(self):
        for polygon in [
            [(0,0,0),(.01,0,0),(.01,.01,0),(0,.01,0)],
            [(0,0,0),(1,0,.1),(1,1,.1),(0,1,0)],
            [(0,0,0),(1,0,0),(1,0,0),(0,0,0)],
        ]:
            with self.subTest(polygon=polygon), self.assertRaises(ValueError):
                grid_slots('red',polygon)

    def test_reserved_and_unknown_slots_are_never_reused(self):
        slots = grid_slots('red',[(0,0,0),(.1,0,0),(.1,.1,0),(0,.1,0)])
        session = AutoSession('workspace',{'cup':'red'},[],{'red':slots})
        slot = session.reserve('cup')
        self.assertEqual(slot.state,'RESERVED')
        slot.state = 'UNKNOWN'
        with self.assertRaisesRegex(ValueError,'ZONE_FULL'):
            session.reserve('cup')


if __name__ == '__main__':
    unittest.main()
