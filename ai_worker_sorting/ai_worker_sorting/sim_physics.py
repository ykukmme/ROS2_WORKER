"""MuJoCo state, local 6D IK, bounded Cartesian paths and contact measurements."""
import math

import mujoco
import numpy as np

from .sim_scene import load_scene, TCP_LOCAL, TABLE_Z, OBJECTS


class PhysicsWorld:
    def __init__(self, seed=7):
        self.model = load_scene(seed)
        self.data = mujoco.MjData(self.model)
        self.scratch = mujoco.MjData(self.model)
        m = self.model
        self.arm_joints = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f'arm_l_joint{i}') for i in range(1, 8)])
        self.qadr, self.dofs = m.jnt_qposadr[self.arm_joints], m.jnt_dofadr[self.arm_joints]
        self.arm_actuators = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f'arm_l_joint{i}') for i in range(1, 8)])
        self.grip_actuator = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_l_joint1')
        self.tcp_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, 'sim_tcp')
        self.lower, self.upper = m.jnt_range[self.arm_joints].T
        self.path, self.path_started, self.path_duration = None, 0., 0.
        self.fault = ''
        self.stopped = False
        self.object_ids = {name: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name) for name, _, _ in OBJECTS}
        lift = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'lift_joint')
        self.data.qpos[m.jnt_qposadr[lift]] = -.35
        self.data.qpos[self.qadr] = np.clip([0., -.8, 0., -1., 0., .8, 0.], self.lower, self.upper)
        mujoco.mj_forward(m, self.data)
        # Initialize pose once. Motion after startup only uses actuators.
        q = self.solve_ik(np.array([.25, .52, .90]), np.array([1., 0., 0., 0.]), self.data.qpos[self.qadr])
        self.data.qpos[self.qadr] = q
        for i in range(m.nu):
            j = m.actuator_trnid[i, 0]
            if j >= 0 and m.actuator_biasprm[i, 1] != 0:
                self.data.ctrl[i] = self.data.qpos[m.jnt_qposadr[j]]
        self.data.ctrl[self.grip_actuator] = 0.
        mujoco.mj_forward(m, self.data)

    def pose(self, data=None):
        d = self.data if data is None else data
        q = np.empty(4)
        mujoco.mju_mat2Quat(q, d.site_xmat[self.tcp_id])
        return d.site_xpos[self.tcp_id].copy(), q

    def solve_ik(self, position, quaternion, seed):
        m, d = self.model, self.scratch
        d.qpos[:] = self.data.qpos
        d.qpos[self.qadr] = seed
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        target = np.array(quaternion, dtype=float)
        target /= np.linalg.norm(target)
        for _ in range(220):
            mujoco.mj_forward(m, d)
            measured, q = self.pose(d)
            # mju_subQuat gives a local rotational error; jacobian is in world axes.
            local = np.empty(3)
            mujoco.mju_subQuat(local, target, q)
            angular = d.site_xmat[self.tcp_id].reshape(3, 3) @ local
            error = np.r_[position-measured, angular]
            if np.linalg.norm(error[:3]) < .0008 and np.linalg.norm(error[3:]) < .015:
                return d.qpos[self.qadr].copy()
            mujoco.mj_jacSite(m, d, jp, jr, self.tcp_id)
            jac = np.vstack([jp[:, self.dofs], jr[:, self.dofs]])
            step = jac.T @ np.linalg.solve(jac@jac.T + .0002*np.eye(6), error)
            d.qpos[self.qadr] = np.clip(d.qpos[self.qadr]+np.clip(step, -.07, .07), self.lower+1e-5, self.upper-1e-5)
        raise ValueError(f'IK target unreachable: {np.round(position, 4).tolist()}')

    def plan(self, target, quaternion, duration):
        if self.stopped:
            raise RuntimeError('Simulation stop is latched; restart the session')
        if self.fault:
            raise RuntimeError(self.fault)
        start, start_q = self.pose()
        target = np.asarray(target, dtype=float)
        q = np.asarray(quaternion, dtype=float)
        if (not np.isfinite(np.r_[target, q, duration]).all() or duration <= 0
                or abs(np.linalg.norm(q)-1) > .001):
            raise ValueError('Invalid Cartesian command')
        if np.dot(start_q, q) < 0:
            q = -q
        count = max(3, int(np.linalg.norm(target-start)/.008)+1)
        count = min(160, count)
        joints = [self.data.qpos[self.qadr].copy()]
        held = [name for name, state in self.evaluation().items()
                if state['lifted'] and state['bilateral_contact']]
        rotation = self.data.site_xmat[self.tcp_id].reshape(3, 3).copy()
        for alpha in np.linspace(0, 1, count)[1:]:
            interp = (1-alpha)*start_q+alpha*q
            interp /= np.linalg.norm(interp)
            joints.append(self.solve_ik((1-alpha)*start+alpha*target, interp, joints[-1]))
            # Predict a held object's pose only in scratch collision data. The real
            # object's pose is always advanced by contact dynamics, never attached.
            planned_rotation = self.scratch.site_xmat[self.tcp_id].reshape(3, 3)
            for name in held:
                body = self.object_ids[name]
                joint = self.model.body_jntadr[body]
                address = self.model.jnt_qposadr[joint]
                relative = rotation.T@(self.data.xpos[body]-start)
                self.scratch.qpos[address:address+3] = self.scratch.site_xpos[self.tcp_id]+planned_rotation@relative
                orientation = planned_rotation@rotation.T@self.data.xmat[body].reshape(3,3)
                mujoco.mju_mat2Quat(self.scratch.qpos[address+3:address+7], orientation.reshape(-1))
            mujoco.mj_forward(self.model, self.scratch)
            self.check_collisions(self.scratch, planning=True)
        self.path = np.array(joints)
        self.path_started, self.path_duration = self.data.time, max(duration, .5)
        self.stopped = False

    def check_collisions(self, data, planning=False):
        m = self.model
        for contact in data.contact:
            if contact.dist >= -.001:
                continue
            names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ''
                     for g in (contact.geom1, contact.geom2)]
            active = any(n.startswith(('arm_l_', 'gripper_l_')) for n in names)
            if not active:
                continue
            geom_names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ''
                          for g in (contact.geom1, contact.geom2)]
            # Finger-to-object contact is necessary. Other robot/object and table hits fail.
            if (any(n.startswith('gripper_l_') for n in names)
                    and any(n in self.object_ids for n in names)):
                continue
            if all(n.startswith('gripper_l_') for n in names):
                continue
            raise ValueError(f'Forbidden collision: {names} / {geom_names}')

    def stop(self):
        self.path = None
        self.data.ctrl[self.arm_actuators] = self.data.qpos[self.qadr]
        joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, 'gripper_l_joint1')
        self.data.ctrl[self.grip_actuator] = self.data.qpos[self.model.jnt_qposadr[joint]]
        self.stopped = True

    def step(self, steps=1):
        for _ in range(steps):
            if self.path is not None:
                alpha = min(1., (self.data.time-self.path_started)/self.path_duration)
                smooth = alpha*alpha*(3-2*alpha)
                index = smooth*(len(self.path)-1)
                lower = min(int(index), len(self.path)-2)
                self.data.ctrl[self.arm_actuators] = ((1-(index-lower))*self.path[lower]
                                                     +(index-lower)*self.path[lower+1])
            mujoco.mj_step(self.model, self.data)
            if not np.isfinite(self.data.qpos).all() or any(w.number for w in self.data.warning):
                self.fault = 'Physics diverged or MuJoCo warning occurred'
                self.stop()
                raise RuntimeError(self.fault)
        self.check_collisions(self.data)

    def contact_objects(self):
        touched = {name: set() for name in self.object_ids}
        for i, contact in enumerate(self.data.contact):
            force = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, force)
            if force[0] < .01:
                continue
            bodies = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                       self.model.geom_bodyid[g]) or '' for g in (contact.geom1, contact.geom2)]
            for name in touched:
                if name in bodies:
                    for side in ('r', 'l'):
                        if any(b.startswith(f'gripper_l_{side}') for b in bodies):
                            touched[name].add(side)
        return touched

    def evaluation(self):
        contacts = self.contact_objects()
        return {name: {'position': self.data.xpos[index].tolist(),
                       'lifted': bool(self.data.xpos[index, 2] > TABLE_Z+.07),
                       'bilateral_contact': contacts[name] == {'l', 'r'}}
                for name, index in self.object_ids.items()}
