"""Metric surface localization from an aligned, rectified depth image."""
import math
import numpy as np


def depth_surface(depth, encoding, bbox, projection, scale_16u=0.001,
                  min_depth=0.1, max_depth=2.0, min_samples=20, max_mad=0.02):
    """Central ROI median, in metres. Returns an optical-frame XYZ surface point.

    Assumes an isolated object whose central box region lies on its surface.
    No object rotation or object-center offset is estimated here.
    """
    u, v, width, height = bbox
    fx, fy, cx, cy = projection
    if not all(math.isfinite(x) for x in (*bbox, *projection, scale_16u,
                                         min_depth, max_depth, max_mad)):
        raise ValueError('Non-finite depth/calibration input')
    if depth.ndim != 2 or fx <= 0 or fy <= 0 or width <= 0 or height <= 0:
        raise ValueError('Invalid depth image, box or intrinsics')
    if not (0 < scale_16u and 0 < min_depth < max_depth and min_samples > 0 and max_mad >= 0):
        raise ValueError('Invalid depth settings')
    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
        raise ValueError('Detection center outside depth image')
    if encoding == '16UC1':
        metric = depth.astype(float)*scale_16u
    elif encoding == '32FC1':
        metric = depth.astype(float)
    else:
        raise ValueError(f'Unsupported depth encoding: {encoding}')
    x1, x2 = max(0, int(u-width*.2)), min(depth.shape[1], int(u+width*.2)+1)
    y1, y2 = max(0, int(v-height*.2)), min(depth.shape[0], int(v+height*.2)+1)
    roi = metric[y1:y2, x1:x2]
    valid = roi[np.isfinite(roi) & (roi >= min_depth) & (roi <= max_depth)]
    if valid.size < min_samples or valid.size < roi.size*.5:
        raise ValueError('Insufficient valid depth')
    z = float(np.median(valid))
    spread = float(np.percentile(valid, 90)-np.percentile(valid, 10))
    if float(np.median(np.abs(valid-z))) > max_mad or spread > 2*max_mad:
        raise ValueError('Depth region is not a consistent surface')
    return np.array([(u-cx)*z/fx, (v-cy)*z/fy, z])


def transform_point(point, translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid TF rotation')
    x, y, z, w = q/np.linalg.norm(q)
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    result = rotation@point + np.asarray(translation)
    if not np.isfinite(result).all():
        raise ValueError('Invalid transformed position')
    return result
