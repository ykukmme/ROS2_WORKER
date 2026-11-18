"""Build a deterministic tabletop world around the unmodified official SG2 assets."""
from pathlib import Path
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

TABLE_Z = .72
TCP_LOCAL = np.array([0., 0., -.18])
OBJECTS = [('item_a_0', 'type_a', (.34, .18)),
           ('item_b_0', 'type_b', (.34, .32)),
           ('item_a_1', 'type_a', (.28, .12)),
           ('item_b_1', 'type_b', (.28, .39))]
ZONES = [(.49, .13), (.49, .37)]
OBJECT_HEIGHT = .065


def scene_xml(seed=7, training=False):
    folder = Path(os.environ.get('WORKER_SIM_MODELS', '/opt/sim-models/robotis_ffw'))
    root = ET.parse(folder / 'ffw_sg2.xml').getroot()
    root.find('compiler').set('meshdir', str(folder / 'assets'))
    base = root.find(".//body[@name='base_link']")
    base.remove(base.find('freejoint'))
    for body in root.findall('.//body'):
        body.set('gravcomp', '1')
    wrist = root.find(".//body[@name='arm_l_link7']")
    ET.SubElement(wrist, 'site', name='sim_tcp', pos='0 0 -.18', size='.004', rgba='0 0 0 0')
    option = root.find('option')
    option.set('timestep', '.002')
    option.set('iterations', '100')
    option.set('cone', 'elliptic')
    visual = root.find('visual')
    if visual is None:
        visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='640', offheight='480')
    world = root.find('worldbody')
    ET.SubElement(world, 'light', pos='.4 .25 2.4', dir='0 0 -1', diffuse='.9 .9 .9', ambient='.4 .4 .4')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='3 3 .05', rgba='.18 .2 .22 1')
    ET.SubElement(world, 'geom', name='table', type='box', pos=f'.43 .25 {TABLE_Z-.025}',
                  size='.23 .3 .025', rgba='.72 .73 .74 1', friction='1 .01 .001')
    # ROS optical coordinates: x right, y down, z forward. MuJoCo cameras look -Z.
    ET.SubElement(world, 'camera', name='work_camera', pos='.43 .25 1.65',
                  quat='1 0 0 0', fovy='42')
    rng = np.random.default_rng(seed)
    colors = rng.permutation(np.array([[.8, .12, .65, 1], [.15, .7, .25, 1], [.1, .4, .9, 1], [.95, .55, .1, 1]]))[:2]
    for zi, ((x, y), color) in enumerate(zip(ZONES, colors)):
        rgba = ' '.join(map(str, color))
        ET.SubElement(root.find('asset'),'material',name=f'tape_matte_{zi}',rgba=rgba,specular='0',shininess='0')
        for k, (dx, dy, sx, sy) in enumerate(((0, -.105, .113, .008), (0, .105, .113, .008),
                                             (-.105, 0, .008, .097), (.105, 0, .008, .097))):
            ET.SubElement(world, 'geom', name=f'tape_{zi}_{k}', type='box',
                          pos=f'{x+dx} {y+dy} {TABLE_Z+.0003}', size=f'{sx} {sy} .0002',
                          material=f'tape_matte_{zi}', contype='0', conaffinity='0')
    for name, label, (x, y) in OBJECTS:
        if training:
            x = rng.uniform(.275, .585)
            y = rng.uniform(.04, .46)
        body = ET.SubElement(world, 'body', name=name, pos=f'{x} {y} {TABLE_Z+OBJECT_HEIGHT/2+.001}')
        ET.SubElement(body, 'freejoint', name=f'{name}_joint')
        # Different geometries; colors are neutral so tape recognition has no object palette.
        attrs = dict(name=f'{name}_geom', mass='.045', friction='1.5 .02 .002',
                     condim='4', solref='.004 1', rgba='.32 .33 .34 1')
        if label == 'type_a':
            attrs.update(type='box', size=f'.022 .020 {OBJECT_HEIGHT/2}')
        else:
            attrs.update(type='cylinder', size=f'.020 {OBJECT_HEIGHT/2}')
        ET.SubElement(body, 'geom', **attrs)
    return ET.tostring(root, encoding='unicode')


def load_scene(seed=7, training=False):
    return mujoco.MjModel.from_xml_string(scene_xml(seed, training))


def intrinsics(model, width=640, height=480):
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'work_camera')
    f = height / (2 * np.tan(np.deg2rad(model.cam_fovy[index]) / 2))
    return float(f), float(f), (width-1)/2, (height-1)/2
