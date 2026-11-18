import unittest
import cv2
import numpy as np
from ai_worker_sorting.zones import tape_zones, find_tape_regions, grid_slots, AutoSession, inside, overlap


def tape_image(colors=((0,0,255),(255,0,0)),background=220):
    image = np.full((360, 640, 3), background, dtype=np.uint8)
    cv2.rectangle(image, (400,20), (620,160), colors[0], 8)
    cv2.rectangle(image, (400,200), (620,340), colors[1], 8)
    return image


class ZoneTests(unittest.TestCase):
    def test_closed_color_tape_interiors_detected(self):
        zones = tape_zones(tape_image())
        self.assertEqual(set(zones), {'zone_1','zone_2'})
        self.assertTrue(inside((500,90), zones['zone_1']))
        self.assertFalse(inside((100,90), zones['zone_1']))

    def test_colors_outside_old_palette_are_discovered(self):
        regions = find_tape_regions(tape_image(((0,128,255),(200,0,180))))
        self.assertEqual([r.color for r in regions],['#ff8000','#b400c8'])

    def test_colors_across_hue_range_need_no_configuration(self):
        for hues in ((8,95),(25,145),(43,165),(60,110),(175,80)):
            hsv = np.array([[[h,220,230] for h in hues]],dtype=np.uint8)
            colors = [tuple(map(int,p)) for p in cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)[0]]
            with self.subTest(hues=hues):
                self.assertEqual(len(tape_zones(tape_image(colors))),2)

    def test_neutral_tape_with_background_contrast_is_detected(self):
        for colors in (((0,0,0),(255,255,255)),((90,90,90),(255,255,255))):
            with self.subTest(colors=colors):
                self.assertEqual(len(tape_zones(tape_image(colors))),2)

    def test_two_regions_of_same_color_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'colors are too similar'):
            tape_zones(tape_image(((0,0,255),(0,0,255))))

    def test_hue_wrap_does_not_create_two_different_reds(self):
        hsv = np.array([[[179,240,230],[1,240,230]]],dtype=np.uint8)
        colors = [tuple(map(int,p)) for p in cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)[0]]
        with self.assertRaisesRegex(ValueError,'colors are too similar'):
            tape_zones(tape_image(colors))

    def test_three_different_rectangles_are_not_silently_reduced_to_two(self):
        image = tape_image()
        cv2.rectangle(image,(20,20),(200,160),(0,255,0),8)
        with self.assertRaisesRegex(ValueError,'found 3'):
            tape_zones(image)

    def test_unrelated_color_patches_do_not_become_zones(self):
        image = tape_image()
        cv2.rectangle(image,(20,20),(200,160),(0,255,255),-1)
        cv2.circle(image,(150,260),40,(200,0,180),-1)
        self.assertEqual(len(tape_zones(image)),2)

    def test_clipped_tape_border_is_rejected(self):
        image = tape_image()
        image[:185] = 220
        cv2.rectangle(image,(400,0),(620,160),(0,0,255),8)
        with self.assertRaises(ValueError):
            tape_zones(image)

    def test_low_background_contrast_is_rejected(self):
        with self.assertRaises(ValueError):
            tape_zones(tape_image(((220,220,220),(0,0,0))))

    def test_region_numbers_follow_previous_positions_not_color_brightness(self):
        image = tape_image(((0,128,255),(200,0,180)))
        initial = find_tape_regions(image)
        previous = [np.mean(r.polygon,axis=0).tolist() for r in reversed(initial)]
        regions = find_tape_regions(image,previous_centers=previous)
        self.assertTrue(inside((500,270),regions[0].polygon))
        self.assertTrue(inside((500,90),regions[1].polygon))

    def test_mild_brightness_and_pixel_noise_keep_two_regions(self):
        image = tape_image(((0,128,255),(200,0,180)))
        noise = np.random.default_rng(7).integers(-3,4,size=image.shape)
        image = np.clip(image.astype(float)*.85+noise,0,255).astype(np.uint8)
        regions = find_tape_regions(image)
        self.assertEqual(len(regions),2)
        self.assertTrue(inside((500,90),regions[0].polygon))

    def test_invalid_image_and_area_rejected(self):
        for image,area in ((np.empty((0,0,3),np.uint8),1500),(tape_image().astype(float),1500),
                           (tape_image(),0),(tape_image(),float('nan'))):
            with self.subTest(area=area),self.assertRaises(ValueError):
                tape_zones(image,area)

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
