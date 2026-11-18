"""Container-only simulation commands. Never load hardware launch files."""
import argparse
import json
import os
from pathlib import Path


def inspect_world(output):
    import mujoco
    from PIL import Image
    from .sim_physics import PhysicsWorld
    from .sim_scene import intrinsics
    from .zones import find_tape_regions
    world = PhysicsWorld()
    world.step(500)
    with mujoco.Renderer(world.model, height=480, width=640) as renderer:
        renderer.update_scene(world.data, camera='work_camera')
        rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        renderer.update_scene(world.data, camera='work_camera')
        depth = renderer.render().copy()
    Image.fromarray(rgb).save(output/'scene.png')
    regions = find_tape_regions(rgb[:,:,::-1].copy())
    report = dict(tcp=world.pose()[0].tolist(), zones=[r.color for r in regions],
                  depth_min=float(depth.min()), intrinsics=intrinsics(world.model),
                  objects=world.evaluation(), fault=world.fault)
    from .sim_profile import resolve_weights
    weights = resolve_weights(output)
    if weights.exists():
        from ultralytics import YOLO
        from .auto_geometry import footprint,depth_top
        from .sim_scene import TABLE_Z
        results = YOLO(str(weights)).predict(rgb[:,:,::-1].copy(),conf=.65,imgsz=416,device='cpu',verbose=False)[0]
        camera_index = mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_CAMERA,'work_camera')
        origin = world.data.cam_xpos[camera_index]
        report['detections'] = []
        for box in results.boxes:
            x,y,w,h = map(float,box.xywh[0])
            top = depth_top(depth,'32FC1',(x,y,w,h),intrinsics(world.model),origin,[1.,0.,0.,0.],TABLE_Z)
            size = footprint((x,y,w,h),intrinsics(world.model),origin,[1.,0.,0.,0.],TABLE_Z,top,(640,480))
            report['detections'].append(dict(label=results.names[int(box.cls.item())],confidence=float(box.conf.item()),footprint=size))
        Image.fromarray(results.plot()[:,:,::-1]).save(output/'scene_yolo.png')
    (output/'inspect.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('command', choices=('inspect','train','menu','test'), default='menu', nargs='?')
    parser.add_argument('--epochs', type=int, default=35)
    args = parser.parse_args()
    output = Path(os.environ.get('WORKER_SIM_DATA','/sim-data'))
    output.mkdir(parents=True, exist_ok=True)
    if args.command == 'inspect':
        inspect_world(output)
    elif args.command == 'train':
        from .sim_training import train
        train(output, args.epochs)
    else:
        from .sim_session import run
        run(output, interactive=args.command=='menu')


if __name__ == '__main__':
    main()
