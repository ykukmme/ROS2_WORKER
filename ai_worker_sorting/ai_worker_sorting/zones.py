"""Color-tape boundaries and deterministic, non-overlapping placement slots."""
from dataclasses import dataclass, field
import math
import uuid

from .core import Point


def inside(point, polygon):
    x, y = point
    sign = None
    for a, b in zip(polygon, polygon[1:]+polygon[:1]):
        cross = (b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0])
        if abs(cross) < 1e-9:
            continue
        current = cross > 0
        if sign is not None and sign != current:
            return False
        sign = current
    return sign is not None


def validate_quad(polygon):
    if len(polygon) != 4 or not all(math.isfinite(v) for p in polygon for v in p[:2]):
        raise ValueError('Zone must be a finite convex quadrilateral')
    crosses = []
    for i in range(4):
        a, b, c = polygon[i], polygon[(i+1)%4], polygon[(i+2)%4]
        crosses.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if not (all(c > 1e-10 for c in crosses) or all(c < -1e-10 for c in crosses)):
        raise ValueError('Zone is degenerate or non-convex')


def overlap(a, b):
    # Separating axis theorem for convex polygons. Touching counts as overlap.
    for polygon in (a, b):
        for p, q in zip(polygon, polygon[1:]+polygon[:1]):
            axis = (-(q[1]-p[1]), q[0]-p[0])
            pa = [x*axis[0]+y*axis[1] for x, y, *_ in a]
            pb = [x*axis[0]+y*axis[1] for x, y, *_ in b]
            if max(pa) < min(pb)-1e-9 or max(pb) < min(pa)-1e-9:
                return False
    return True


def tape_zones(bgr, colors=('red', 'blue'), min_area=1500):
    import cv2
    import numpy as np
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    ranges = {'red': [(0, 10), (170, 179)], 'blue': [(100, 130)],
              'green': [(40, 85)], 'yellow': [(20, 35)]}
    zones = {}
    for color in colors:
        if color not in ranges:
            raise ValueError(f'Unsupported tape color: {color}')
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for low, high in ranges[color]:
            mask |= cv2.inRange(hsv, (low, 100, 70), (high, 255, 255))
        # Do not bridge missing tape gaps: require an actual closed contour.
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        found = []
        if hierarchy is not None:
            for index, contour in enumerate(contours):
                if hierarchy[0][index][3] < 0 or cv2.contourArea(contour) < min_area:
                    continue
                corners = cv2.approxPolyDP(contour, .025*cv2.arcLength(contour, True), True)
                if len(corners) == 4 and cv2.isContourConvex(corners):
                    found.append([(float(p[0][0]), float(p[0][1])) for p in corners])
        if len(found) != 1:
            raise ValueError(f'{color}: expected one closed tape rectangle, found {len(found)}')
        validate_quad(found[0])
        zones[color] = found[0]
    values = list(zones.values())
    if len(values) != 2 or overlap(values[0], values[1]):
        raise ValueError('Exactly two separate colored zones are required')
    return zones


@dataclass
class Slot:
    slot_id: str
    position: Point
    state: str = 'EMPTY'


def grid_slots(zone_id, polygon, object_size=.06, margin=.01, place_offset=.05):
    validate_quad(polygon)
    if not all(math.isfinite(v) for p in polygon for v in p) or any(len(p) != 3 for p in polygon):
        raise ValueError('Zone requires finite XYZ corners')
    if max(p[2] for p in polygon)-min(p[2] for p in polygon) > .015:
        raise ValueError('Only horizontal tabletop zones are supported')
    if not all(math.isfinite(v) for v in (object_size, margin, place_offset)) or object_size <= 0 or margin < 0:
        raise ValueError('Invalid footprint, clearance or placement offset')
    radius = object_size/2+margin
    pitch = 2*radius
    x0, x1 = min(p[0] for p in polygon), max(p[0] for p in polygon)
    y0, y1 = min(p[1] for p in polygon), max(p[1] for p in polygon)
    if max(x1-x0, y1-y0) > 3:
        raise ValueError('Zone dimensions exceed prototype limits')
    slots = []
    z = sum(p[2] for p in polygon)/4+place_offset
    for ix in range(max(0, int((x1-x0)/pitch)+1)):
        for iy in range(max(0, int((y1-y0)/pitch)+1)):
            x, y = x0+radius+ix*pitch, y0+radius+iy*pitch
            if all(inside((x+dx, y+dy), polygon)
                   for dx, dy in ((-radius,-radius), (radius,-radius), (radius,radius), (-radius,radius))):
                slots.append(Slot(f'{zone_id}-{len(slots)}', Point(x, y, z)))
    if not slots:
        raise ValueError('ZONE_TOO_SMALL: no placement slot fits')
    return slots


@dataclass
class AutoSession:
    workspace_id: str
    mapping: dict
    zones: list
    slots: dict
    session_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    completed_mock_ids: set = field(default_factory=set)
    placed: int = 0
    started: bool = False

    def reserve(self, class_name):
        zone_id = self.mapping[class_name]
        for slot in self.slots.get(zone_id, []):
            if slot.state == 'EMPTY':
                slot.state = 'RESERVED'
                return slot
        raise ValueError(f'ZONE_FULL: {zone_id}')
