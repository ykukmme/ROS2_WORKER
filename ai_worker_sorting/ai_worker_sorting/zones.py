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


@dataclass
class TapeRegion:
    polygon: list
    color: str
    lab: tuple


def _color_peaks(histogram, radius, minimum, circular=False):
    """Discover supported pixel values, without a named color palette."""
    import numpy as np
    remaining = np.asarray(histogram,dtype=float).copy()
    peaks = []
    while remaining.max(initial=0)>=minimum:
        index = int(remaining.argmax())
        peaks.append(index)
        distance = np.abs(np.arange(len(remaining))-index)
        if circular:
            distance = np.minimum(distance,len(remaining)-distance)
        remaining[distance<=radius] = 0
    return peaks


def _regions_in_mask(bgr,mask,min_area):
    import cv2
    import numpy as np
    contours,hierarchy = cv2.findContours(mask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    regions = []
    for index,contour in enumerate(contours):
        parent = int(hierarchy[0][index][3])
        inner_area = cv2.contourArea(contour)
        if parent<0 or inner_area<min_area:
            continue
        outer = contours[parent]
        x,y,width,height = cv2.boundingRect(outer)
        # A table/background touching the image boundary is not tape.
        if x<=0 or y<=0 or x+width>=mask.shape[1] or y+height>=mask.shape[0]:
            continue
        corners = cv2.approxPolyDP(contour,.025*cv2.arcLength(contour,True),True)
        outer_corners = cv2.approxPolyDP(outer,.025*cv2.arcLength(outer,True),True)
        if (len(corners)!=4 or not cv2.isContourConvex(corners) or
                len(outer_corners)!=4 or not cv2.isContourConvex(outer_corners)):
            continue
        if inner_area<.35*cv2.contourArea(outer):
            continue
        ring = np.zeros_like(mask)
        cv2.drawContours(ring,[outer],-1,255,-1)
        cv2.drawContours(ring,[contour],-1,0,-1)
        pixels = bgr[(ring!=0)&(mask!=0)]
        if len(pixels)<max(40,min_area*.02):
            continue
        lab_pixels = cv2.cvtColor(pixels.astype(np.float32).reshape(-1,1,3)/255.,cv2.COLOR_BGR2LAB).reshape(-1,3)
        lab = np.median(lab_pixels,axis=0)
        if np.percentile(np.linalg.norm(lab_pixels-lab,axis=1),90)>18:
            continue
        color = np.median(pixels,axis=0).astype(np.uint8)
        polygon = [(float(p[0][0]),float(p[0][1])) for p in corners]
        validate_quad(polygon)
        regions.append(TapeRegion(polygon,'#'+''.join(f'{int(v):02x}' for v in color[::-1]),tuple(lab)))
    return regions


def find_tape_regions(bgr,min_area=1500,previous_centers=None):
    """Find exactly two distinct, closed tape colors in the current image.

    Hue peaks cover chromatic tape; value peaks cover neutral tape with visible
    contrast. No closing/dilation bridges broken borders. Ambiguous scenes fail.
    """
    import cv2
    import numpy as np
    if (not isinstance(bgr,np.ndarray) or bgr.dtype!=np.uint8 or
            bgr.ndim!=3 or bgr.shape[2]!=3 or not bgr.size):
        raise ValueError('Tape detection requires a nonempty uint8 BGR image')
    if not math.isfinite(min_area) or min_area<=0:
        raise ValueError('Invalid minimum tape zone area')
    hsv = cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
    hue,saturation,value = cv2.split(hsv)
    chromatic = (saturation>=70)&(value>=50)
    neutral = (saturation<70)|(value<50)
    minimum = max(40,int(min_area*.02))
    hue_hist = np.bincount(hue[chromatic],minlength=180)
    value_hist = np.bincount(value[neutral]//8,minlength=32)
    masks = []
    for peak in _color_peaks(hue_hist,8,minimum,circular=True):
        distance = np.abs(hue.astype(np.int16)-peak)
        distance = np.minimum(distance,180-distance)
        masks.append(((distance<=10)&chromatic).astype(np.uint8)*255)
    for peak in _color_peaks(value_hist,2,minimum):
        masks.append(((np.abs(value.astype(np.int16)-(peak*8+4))<=20)&neutral).astype(np.uint8)*255)
    regions = []
    for mask in masks:
        for candidate in _regions_in_mask(bgr,mask,min_area):
            duplicate = False
            for known in regions:
                a,b = np.asarray(candidate.polygon,np.float32),np.asarray(known.polygon,np.float32)
                intersection,_ = cv2.intersectConvexConvex(a,b)
                union = cv2.contourArea(a)+cv2.contourArea(b)-intersection
                if intersection/union>.85 and np.linalg.norm(np.array(candidate.lab)-known.lab)<12:
                    duplicate = True
                    break
            if not duplicate:
                regions.append(candidate)
    if len(regions)!=2:
        raise ValueError(f'Exactly two closed tape rectangles required; found {len(regions)}')
    if np.linalg.norm(np.array(regions[0].lab)-regions[1].lab)<20:
        raise ValueError('Tape colors are too similar; use two clearly different colors')
    if overlap(regions[0].polygon,regions[1].polygon):
        raise ValueError('Exactly two separate colored zones are required')
    centers = np.array([np.mean(region.polygon,axis=0) for region in regions])
    if previous_centers is not None:
        previous = np.asarray(previous_centers,dtype=float)
        if previous.shape!=(2,2) or not np.isfinite(previous).all():
            raise ValueError('Invalid previous tape centers')
        reverse = np.sum((centers[::-1]-previous)**2)<np.sum((centers-previous)**2)
    else:
        axis = int(abs(centers[0,1]-centers[1,1])>abs(centers[0,0]-centers[1,0]))
        reverse = centers[0,axis]>centers[1,axis]
    return regions[::-1] if reverse else regions


def tape_zones(bgr,min_area=1500):
    return {f'zone_{index+1}':region.polygon
            for index,region in enumerate(find_tape_regions(bgr,min_area))}


@dataclass
class Slot:
    slot_id: str
    position: Point
    state: str = 'EMPTY'


def grid_slots(zone_id, polygon, object_size=.06, margin=.01, place_offset=.05, pitch_extra=0.):
    validate_quad(polygon)
    if not all(math.isfinite(v) for p in polygon for v in p) or any(len(p) != 3 for p in polygon):
        raise ValueError('Zone requires finite XYZ corners')
    if max(p[2] for p in polygon)-min(p[2] for p in polygon) > .015:
        raise ValueError('Only horizontal tabletop zones are supported')
    if not all(math.isfinite(v) for v in (object_size, margin, place_offset, pitch_extra)) or object_size <= 0 or margin < 0 or not 0<=pitch_extra<=.05:
        raise ValueError('Invalid footprint, clearance or placement offset')
    radius = object_size/2+margin
    pitch = 2*radius+pitch_extra
    x0, x1 = min(p[0] for p in polygon), max(p[0] for p in polygon)
    y0, y1 = min(p[1] for p in polygon), max(p[1] for p in polygon)
    if max(x1-x0, y1-y0) > 3:
        raise ValueError('Zone dimensions exceed prototype limits')
    slots = []
    z = sum(p[2] for p in polygon)/4+place_offset
    def count(span):
        return max(0, 1+math.floor((span-2*radius+1e-10)/pitch))
    nx, ny = count(x1-x0), count(y1-y0)
    # Center the grid: anchoring at the bounding-box corner can reject every
    # slot in a slightly skewed measured rectangle, despite ample room inside.
    for ix in range(nx):
        for iy in range(ny):
            x = (x0+x1)/2+(ix-(nx-1)/2)*pitch
            y = (y0+y1)/2+(iy-(ny-1)/2)*pitch
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
