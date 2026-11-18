"""Execute real contact dynamics; failed grasp must fail this check."""
import json
import os
from pathlib import Path

import mujoco
import numpy as np

from ai_worker_sorting.sim_physics import PhysicsWorld
from ai_worker_sorting.sim_scene import TABLE_Z, OBJECT_HEIGHT


def main():
    world = PhysicsWorld()
    world.step(500)
    log = []

    def move(target, duration=2.):
        world.plan(target,[1.,0.,0.,0.],duration)
        for _ in range(int((duration+1.)/.02)):
            world.step(10)
        position = world.pose()[0]
        error = float(np.linalg.norm(position-target))
        print('MOVE',target,position.tolist(),error,flush=True)
        assert error < .008
        log.append(dict(target=target,error=error,evaluation=world.evaluation()))

    move([.34,.18,.93])
    move([.34,.18,TABLE_Z+OBJECT_HEIGHT-.005])
    world.data.ctrl[world.grip_actuator] = 1.1
    world.step(1500)
    print('CLOSE',world.evaluation(),flush=True)
    move([.34,.18,.93],3.)
    state = world.evaluation()['item_a_0']
    print('GRASP',state,flush=True)
    output = Path(os.environ.get('WORKER_SIM_DATA','/sim-data'))
    (output/'physics_smoke.json').write_text(json.dumps(log,indent=2))
    assert state['lifted'] and state['bilateral_contact'], 'Contact grasp/lift not verified'
    move([.49,.13,.93])
    move([.49,.13,TABLE_Z+OBJECT_HEIGHT+.020])
    world.data.ctrl[world.grip_actuator] = 0.
    world.step(1000)
    move([.49,.13,.93])
    final = world.evaluation()['item_a_0']
    assert np.linalg.norm(np.array(final['position'][:2])-[.49,.13]) < .04
    assert abs(final['position'][2]-(TABLE_Z+OBJECT_HEIGHT/2)) < .015
    print('PHYSICS_PICK_PLACE_PASS',json.dumps(final),flush=True)


if __name__ == '__main__':
    main()
