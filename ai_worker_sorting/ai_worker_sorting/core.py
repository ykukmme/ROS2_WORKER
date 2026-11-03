"""ROS-independent geometry and task sequence; distances are metres."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float

    def above(self, distance):
        return Point(self.x, self.y, self.z + distance)


def finite(values):
    return all(math.isfinite(float(v)) for v in values)


def in_workspace(point, limits):
    values = (point.x, point.y, point.z)
    return (len(limits) == 6 and finite(values + tuple(limits))
            and all(limits[2*i] <= values[i] <= limits[2*i+1] for i in range(3)))


def pixel_on_plane(u, v, fx, fy, cx, cy, origin, rotation, plane_z):
    """Intersect a rectified optical-camera ray with a base-frame Z plane.

    rotation: 3x3 camera-to-base rotation; origin: camera in base frame.
    plane_z is the visible object's TOP surface, not the tabletop.
    """
    values = [u, v, fx, fy, cx, cy, plane_z, *origin]
    values += [x for row in rotation for x in row]
    if not finite(values) or fx <= 0 or fy <= 0:
        raise ValueError('Invalid camera calibration')
    ray = ((u-cx)/fx, (v-cy)/fy, 1.0)
    direction = [sum(rotation[i][j]*ray[j] for j in range(3)) for i in range(3)]
    if abs(direction[2]) < 1e-8:
        raise ValueError('Camera ray is parallel to the object plane')
    scale = (plane_z-origin[2])/direction[2]
    if scale <= 0:
        raise ValueError('Object plane is behind the camera')
    return Point(*(origin[i]+scale*direction[i] for i in range(3)))


def pick_and_place(robot, target, bin_point, home, limits, clearance,
                   grasp_offset, check, report, transport_z=None):
    """Stop at the first failure. Never auto-open/recover after an error."""
    if not finite([clearance, grasp_offset]) or clearance <= 0:
        raise ValueError('Invalid clearance or grasp offset')
    grasp = target.above(grasp_offset)
    approach = grasp.above(clearance)
    bin_above = bin_point.above(clearance)
    raise_from_home = None
    if transport_z is not None:
        if not finite([transport_z]) or transport_z<max(grasp.z,bin_point.z):
            raise ValueError('Transport height must clear grasp and placement endpoints')
        approach = Point(grasp.x,grasp.y,max(approach.z,transport_z))
        bin_above = Point(bin_point.x,bin_point.y,max(bin_above.z,transport_z))
        raise_from_home = Point(home.x,home.y,max(home.z,transport_z))
    # Validate ALL endpoints before sending even the first command.
    endpoints = [grasp,approach,bin_point,bin_above,home]+([raise_from_home] if raise_from_home else [])
    if not all(in_workspace(p, limits) for p in endpoints):
        raise ValueError('Target, bin, home or approach outside workspace')
    steps = [
        ('OPEN', lambda: robot.gripper(True)),
        *([('RAISE_CLEAR',lambda:robot.raise_clear(raise_from_home) if hasattr(robot,'raise_clear') else robot.move(raise_from_home))] if raise_from_home else []),
        ('APPROACH', lambda: robot.move(approach)),
        ('DESCEND', lambda: robot.move(grasp)),
        ('CLOSE', lambda: robot.gripper(False)),
        ('LIFT', lambda: robot.move(approach)),
        ('VERIFY_GRASP', robot.verify_grasp),
        ('TRANSFER', lambda: robot.move(bin_above)),
        ('LOWER_TO_BIN', lambda: robot.move(bin_point)),
        ('RELEASE', lambda: robot.gripper(True)),
        ('RETREAT', lambda: robot.move(bin_above)),
        *([('RETURN_CLEAR',lambda:robot.move(raise_from_home))] if raise_from_home else []),
        ('HOME', lambda: robot.return_home(home) if hasattr(robot,'return_home') else robot.move(home)),
    ]
    for state, action in steps:
        check()
        report(state)
        action()
    check()
    report('DONE')


class DryRobot:
    def __init__(self, log=print):
        self.log = log

    def move(self, point):
        self.log(f'[DRY] move {point}')

    def gripper(self, opened):
        self.log('[DRY] gripper ' + ('open' if opened else 'close'))

    def verify_grasp(self):
        self.log('[DRY] simulated grasp verification (no sensor check)')
