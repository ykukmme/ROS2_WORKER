"""Conservative full-range tool geometry from expanded URDF and local STL assets."""
import itertools
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from .locator import transform_point


def vector(text,default):
    result = np.array([float(x) for x in text.split()] if text else default,dtype=float)
    if result.shape!=(3,) or not np.isfinite(result).all():
        raise ValueError('Invalid URDF vector')
    return result


def origin(element):
    item = element.find('origin')
    xyz = vector(item.get('xyz') if item is not None else None,[0,0,0])
    r,p,y = vector(item.get('rpy') if item is not None else None,[0,0,0])
    rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    result = np.eye(4)
    result[:3,:3],result[:3,3] = rz@ry@rx,xyz
    return result


def multiply(a,b):
    lo,hi = np.zeros((4,4)),np.zeros((4,4))
    for i,j,k in itertools.product(range(4),repeat=3):
        values = [a[x][i,k]*b[y][k,j] for x,y in itertools.product((0,1),repeat=2)]
        lo[i,j] += min(values)
        hi[i,j] += max(values)
    return lo,hi


def trig_bounds(lo,hi,cosine=False):
    if hi-lo>=2*math.pi:
        return -1.,1.
    fn = math.cos if cosine else math.sin
    values = [fn(lo),fn(hi)]
    shift = 0. if cosine else math.pi/2
    for n in range(math.ceil((lo-shift)/math.pi),math.floor((hi-shift)/math.pi)+1):
        values.append(fn(shift+n*math.pi))
    return min(values),max(values)


def joint_range(joint):
    kind = joint.get('type')
    exact = origin(joint)
    if kind=='fixed':
        return exact,exact
    axis_element = joint.find('axis')
    axis = vector(axis_element.get('xyz') if axis_element is not None else None,[1,0,0])
    if np.linalg.norm(axis)<1e-8:
        raise ValueError('Invalid URDF joint axis')
    axis /= np.linalg.norm(axis)
    limits = joint.find('limit')
    if kind=='continuous':
        lower,upper = -math.pi,math.pi
    elif kind in ('revolute','prismatic') and limits is not None:
        lower,upper = float(limits.get('lower')),float(limits.get('upper'))
    else:
        raise ValueError('Unsupported or unbounded tool joint')
    if not math.isfinite(lower+upper) or lower>upper:
        raise ValueError('Invalid tool joint limits')
    lo,hi = np.eye(4),np.eye(4)
    if kind=='prismatic':
        lo[:3,3],hi[:3,3] = np.minimum(axis*lower,axis*upper),np.maximum(axis*lower,axis*upper)
    else:
        outer = np.outer(axis,axis)
        x,y,z = axis
        skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        c,s = trig_bounds(lower,upper,True),trig_bounds(lower,upper)
        for i,j in itertools.product(range(3),repeat=2):
            values = [outer[i,j]+((1. if i==j else 0.)-outer[i,j])*cv+skew[i,j]*sv for cv in c for sv in s]
            lo[i,j],hi[i,j] = min(values),max(values)
    return multiply((exact,exact),(lo,hi))


def mesh_vertices(filename,urdf_path=None):
    if filename.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package,relative = filename[len('package://'):].split('/',1)
        path = Path(get_package_share_directory(package))/relative
    elif filename.startswith('file://'):
        path = Path(filename[7:])
    else:
        path = Path(filename)
        if not path.is_absolute():
            if urdf_path is None:
                raise ValueError('Relative mesh requires a local URDF path')
            path = Path(urdf_path).parent/path
    if path.suffix.lower()!='.stl' or path.stat().st_size>64*1024*1024:
        raise ValueError('Tool model requires local STL assets smaller than 64MB')
    data = path.read_bytes()
    if len(data)>=84 and 84+50*struct.unpack_from('<I',data,80)[0]==len(data):
        vertices = np.frombuffer(data,offset=84,dtype=np.dtype([('normal','<f4',3),('v','<f4',(3,3)),('attr','<u2')]))['v'].reshape(-1,3)
    else:
        vertices = np.array([[float(x) for x in line.split()[1:]] for line in data.decode('ascii').splitlines()
                             if line.strip().startswith('vertex ')])
    if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError('Invalid STL geometry')
    return vertices


def geometry_bounds(element,urdf_path):
    mesh,box,cylinder,sphere = (element.find(x) for x in ('mesh','box','cylinder','sphere'))
    if mesh is not None:
        scale = vector(mesh.get('scale'),[1,1,1])
        if (scale==0).any():
            raise ValueError('Zero mesh scale is unsupported')
        points = mesh_vertices(mesh.get('filename'),urdf_path)*scale
        return points.min(axis=0),points.max(axis=0)
    if box is not None:
        radius = vector(box.get('size'),[0,0,0])/2
    elif cylinder is not None:
        radius = np.array([float(cylinder.get('radius'))]*2+[float(cylinder.get('length'))/2])
    elif sphere is not None:
        radius = np.full(3,float(sphere.get('radius')))
    else:
        raise ValueError('Unsupported tool geometry')
    if not np.isfinite(radius).all() or (radius<=0).any():
        raise ValueError('Invalid tool dimensions')
    return -radius,radius


def tool_extent(xml,tool_link,gripper_joint,orientation,urdf_path=None):
    """Bound lowest attached geometry for all modeled finger joint positions.

    No current opening/closing pose is substituted for full travel limits.
    Mimic correlations are relaxed, which may overestimate the result.
    """
    robot = ET.fromstring(xml)
    links = {e.get('name'):e for e in robot.findall('link')}
    joints = {e.get('name'):e for e in robot.findall('joint')}
    parents = {j.find('child').get('link'):j for j in joints.values()}
    if tool_link not in links or gripper_joint not in joints:
        raise ValueError('Configured tool/gripper joint not found in expanded robot model')
    def ancestors(link):
        result = [link]
        while link in parents:
            link = parents[link].find('parent').get('link')
            if link in result:
                raise ValueError('Cyclic robot model')
            result.append(link)
        return result
    hand_parent = joints[gripper_joint].find('parent').get('link')
    chain = ancestors(tool_link)
    root = next((link for link in chain if link in ancestors(hand_parent)),None)
    if root is None:
        raise ValueError('Tool and gripper are in different model trees')
    root_to_tool = np.eye(4)
    link = tool_link
    while link!=root:
        joint = parents[link]
        if joint.get('type')!='fixed':
            raise ValueError('Controlled TCP must be fixed relative to the gripper mount')
        root_to_tool = origin(joint)@root_to_tool
        link = joint.find('parent').get('link')
    initial = np.eye(4)
    initial[:3,:3] = np.column_stack([transform_point(np.eye(3)[i],(0.,0.,0.),orientation) for i in range(3)])
    initial = initial@np.linalg.inv(root_to_tool)
    finger = joints[gripper_joint].find('child').get('link')
    pending,visited,minimum,count,finger_count = [(root,(initial,initial))],set(),0.,0,0
    while pending:
        link,transform = pending.pop()
        if link in visited or link not in links:
            raise ValueError('Invalid tool model tree')
        visited.add(link)
        for shape in list(links[link].findall('collision'))+list(links[link].findall('visual')):
            bounds = geometry_bounds(shape.find('geometry'),urdf_path)
            local = origin(shape)
            lo,hi = multiply(transform,(local,local))
            for corner in itertools.product(*zip(*bounds)):
                point = (*corner,1.)
                minimum = min(minimum,sum(min(lo[2,i]*v,hi[2,i]*v) for i,v in enumerate(point)))
            count += 1
            if finger in ancestors(link):
                finger_count += 1
        for joint in joints.values():
            if joint.find('parent').get('link')==link:
                pending.append((joint.find('child').get('link'),multiply(transform,joint_range(joint))))
    extent = -minimum+.003
    if not count or not finger_count or not 0<extent<=.5:
        raise ValueError('Tool geometry unavailable or conservative extent exceeds supported range')
    return float(extent)
