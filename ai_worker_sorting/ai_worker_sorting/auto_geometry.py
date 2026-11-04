"""Conservative image-frustum bounds for fully visible objects on a level table."""
import math
import numpy as np
from .core import pixel_on_plane
from .locator import transform_point


def depth_top(depth, encoding, bbox, intrinsics, origin, quaternion, table_z):
    """Use every valid depth pixel in the box, including high rims/edges.

    Missing depth is rejected instead of assuming an unseen upper bound.
    Raw maxima are deliberately conservative: outliers can reject setup.
    """
    u,v,w,h = bbox
    fx,fy,cx,cy = intrinsics
    if fx<=0 or fy<=0 or not all(math.isfinite(x) for x in (*bbox,*intrinsics,*origin,table_z)):
        raise ValueError('Invalid depth intrinsics or bounding box')
    x0,y0 = int(math.floor(u-w/2)),int(math.floor(v-h/2))
    x1,y1 = int(math.ceil(u+w/2))+1,int(math.ceil(v+h/2))+1
    if depth.ndim!=2 or x0<0 or y0<0 or x1>depth.shape[1] or y1>depth.shape[0] or x0>=x1 or y0>=y1:
        raise ValueError('Object depth box is incomplete')
    if encoding not in ('16UC1','32FC1'):
        raise ValueError('Unsupported depth encoding')
    metric = depth[y0:y1,x0:x1].astype(float)*(.001 if encoding=='16UC1' else 1.)
    valid = np.isfinite(metric)&(metric>=.1)&(metric<=2.)
    # Even a narrow missing stripe can hide a tall edge; require full coverage.
    if not valid.all():
        raise ValueError('Incomplete object depth; cannot bound its highest point')
    yy,xx = np.mgrid[y0:y1,x0:x1]
    rotation = np.column_stack([transform_point(np.eye(3)[i],(0.,0.,0.),quaternion) for i in range(3)])
    heights = origin[2]+rotation[2,0]*(xx-cx)*metric/fx+rotation[2,1]*(yy-cy)*metric/fy+rotation[2,2]*metric
    top = float(np.max(heights))
    if not math.isfinite(top) or not .01<=top-table_z<=.4:
        raise ValueError('Unsupported or unobserved maximum object height')
    return top


def footprint(bbox, intrinsics, origin, quaternion, table_z, top_z, image_size):
    u,v,w,h = bbox
    width,height = image_size
    if not all(math.isfinite(x) for x in (*bbox,table_z,top_z)) or w<=0 or h<=0 or top_z-table_z<.01:
        raise ValueError('Invalid object geometry or object too thin for depth estimation')
    if u-w/2<=1 or v-h/2<=1 or u+w/2>=width-1 or v+h/2>=height-1:
        raise ValueError('Object touches image edge; show its complete outline')
    rotation = np.column_stack([transform_point(np.eye(3)[i],(0.,0.,0.),quaternion) for i in range(3)])
    points = [pixel_on_plane(x,y,*intrinsics,origin,rotation,z)
              for x in (u-w/2,u+w/2) for y in (v-h/2,v+h/2) for z in (table_z,top_z+.01)]
    size = max(max(getattr(p,a) for p in points)-min(getattr(p,a) for p in points) for a in ('x','y'))
    if not .005<size<=.25:
        raise ValueError('Observed object footprint is outside supported bounds')
    return float(size)


def teaching_offset(surface, tcp, table_z, tool_extent):
    if not all(math.isfinite(x) for x in (*surface,*tcp,table_z,tool_extent)):
        raise ValueError('Invalid teaching geometry')
    if math.hypot(tcp[0]-surface[0],tcp[1]-surface[1])>.04:
        raise ValueError('Place the controlled link over the selected object center')
    if tcp[2]-tool_extent<=table_z+.005:
        raise ValueError('Taught grasp leaves insufficient tool/table clearance')
    offset = tcp[2]-surface[2]
    if not -.5<offset<.5:
        raise ValueError('Taught offset outside supported bounds')
    return offset
