"""Train two simulation-only YOLO classes from rendered images, not injected detections."""
import json
from pathlib import Path
import shutil

import mujoco
import numpy as np
from PIL import Image
import yaml

from .sim_physics import PhysicsWorld
from .sim_scene import OBJECTS, TABLE_Z, OBJECT_HEIGHT


def generate_dataset(folder, count=240):
    world = PhysicsWorld()
    m,d = world.model,world.data
    rng = np.random.default_rng(183)
    camera = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, 'work_camera')
    geom_ids = {name: mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,name+'_geom') for name,_,_ in OBJECTS}
    for split in ('train','val'):
        (folder/'images'/split).mkdir(parents=True,exist_ok=True)
        (folder/'labels'/split).mkdir(parents=True,exist_ok=True)
    with mujoco.Renderer(m,height=480,width=640) as renderer:
        for frame in range(count):
            split = 'val' if frame%5 == 0 else 'train'
            positions = []
            for name,_,_ in OBJECTS:
                for _ in range(100):
                    xy = rng.uniform([.27,.015],[.60,.49])
                    if all(np.linalg.norm(xy-other)>.085 for other in positions):
                        break
                positions.append(xy)
                joint = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,name+'_joint')
                adr = m.jnt_qposadr[joint]
                d.qpos[adr:adr+3] = [*xy,TABLE_Z+OBJECT_HEIGHT/2]
                # Keep upright; vary yaw, brightness and tape palette independently.
                angle = rng.uniform(-np.pi,np.pi)
                d.qpos[adr+3:adr+7] = [np.cos(angle/2),0,0,np.sin(angle/2)]
                shade = rng.uniform(.12,.5)
                m.geom_rgba[geom_ids[name],:3] = shade
            table_id = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'table')
            m.geom_rgba[table_id,:3] = rng.uniform(.35,.7)
            for g in range(m.ngeom):
                if (mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g) or '').startswith('tape_'):
                    m.geom_rgba[g,:3] = rng.uniform(.1,.9,3)
            mujoco.mj_forward(m,d)
            renderer.disable_segmentation_rendering()
            renderer.update_scene(d,camera='work_camera')
            rgb = renderer.render().copy()
            Image.fromarray(rgb).save(folder/'images'/split/f'{frame:04}.jpg')
            renderer.enable_segmentation_rendering()
            renderer.update_scene(d,camera='work_camera')
            segmentation = renderer.render().copy()
            labels = []
            for name,label_name,_ in OBJECTS:
                label = {'type_a':0,'type_b':1}[label_name]
                yy,xx = np.where((segmentation[:,:,0]==geom_ids[name]) &
                                 (segmentation[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM)))
                if len(xx)<30:
                    continue
                xmin,xmax,ymin,ymax = xx.min(),xx.max()+1,yy.min(),yy.max()+1
                labels.append(f'{label} {(xmin+xmax)/1280:.6f} {(ymin+ymax)/960:.6f} {(xmax-xmin)/640:.6f} {(ymax-ymin)/480:.6f}')
            (folder/'labels'/split/f'{frame:04}.txt').write_text('\n'.join(labels)+'\n')
            if frame%60==0:
                print(f'Rendered training images: {frame}/{count}',flush=True)
    config = dict(path=str(folder.resolve()),train='images/train',val='images/val',names={0:'type_a',1:'type_b'})
    (folder/'dataset.yaml').write_text(yaml.safe_dump(config))
    return folder/'dataset.yaml'


def train(output, epochs=35):
    import torch
    from ultralytics import YOLO
    if not 1<=epochs<=300:
        raise ValueError('epochs must be in [1,300]')
    torch.set_num_threads(4)
    dataset = generate_dataset(output/'dataset')
    # Pretrained initialization improves small synthetic dataset convergence.
    model = YOLO('yolo11n.pt')
    model.train(data=str(dataset),epochs=epochs,imgsz=416,batch=16,device='cpu',workers=0,
                project=str(output/'training'),name='two_types',exist_ok=True,
                seed=183,deterministic=True,plots=False,cache=False,
                mosaic=.4,close_mosaic=5,fliplr=.5,degrees=15.,hsv_h=.05,hsv_s=.15,hsv_v=.2)
    trained = output/'training'/'two_types'/'weights'/'best.pt'
    if not trained.is_file():
        raise RuntimeError('Training did not produce best.pt')
    shutil.copyfile(trained,output/'simulation_yolo.pt')
    manifest = dict(classes=['type_a','type_b'],source='rendered MuJoCo images',seed=183,epochs=epochs,
                    hardware_validated=False)
    (output/'simulation_yolo.json').write_text(json.dumps(manifest,indent=2))
    print('Simulation weights saved. Run inspect/test to validate the complete scene.',flush=True)
