"""Turning study: what actually makes this 4WD skid-steer turn?

For each variant: spin-in-place yaw rate at 35/65/100 % duty, arc-turn yaw rate (inner 30 % / outer 90 %),
forward speed, and whether it still climbs the 15 deg ramp at 35 % (traction). Motor = straight
torque-speed line through (stall torque, free speed) at 11.6 V.
"""
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
KGCM = 0.0980665
VARIANTS = {
    'JGA25 78:1 (6.2 kg.cm, 77 rpm), all TPU  [current]': (6.2, 77, 0.9, 0.9),
    'JGA25 131:1 (~10 kg.cm, ~46 rpm), all TPU': (10.0, 46, 0.9, 0.9),
    'JGA25 171:1 (~12 kg.cm, ~35 rpm), all TPU': (12.0, 35, 0.9, 0.9),
    'JGA25 78:1, PETG front / TPU rear': (6.2, 77, 0.35, 0.9),
}


def sim(stall, rpm, mu_f, mu_r, dl, dr, secs, ramp=False):
    m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'thenar_walker.xml'))
    m.geom_friction[m.geom('tyre_FL').id][0] = m.geom_friction[m.geom('tyre_FR').id][0] = mu_f
    m.geom_friction[m.geom('tyre_RL').id][0] = m.geom_friction[m.geom('tyre_RR').id][0] = mu_r
    d = mujoco.MjData(m)
    a = m.jnt_qposadr[m.joint('chassis').id]
    x, y, yaw = (-1.2, -0.95, 90) if ramp else (0.3, -0.4, 0)
    d.qpos[a:a + 3] = [x, y, 0.0005]
    d.qpos[a + 3:a + 7] = [math.cos(math.radians(yaw) / 2), 0, 0, math.sin(math.radians(yaw) / 2)]
    for n, v in zip(['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'], [0, -25, 35, 0, 0, 20]):
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = math.radians(v)
        d.ctrl[m.actuator('s_' + n).id] = math.radians(v)
    Ts, wf, kv = stall * KGCM, rpm * 2 * math.pi / 60, 11.6 / 12
    wheels = [(m.jnt_dofadr[m.joint(f'wheel_{p}{q}').id], m.actuator(f'm_{p}{q}').id, 0 if q == 'L' else 1) for p in 'FR' for q in 'LR']
    yaws, pos = [], []
    for _ in range(int(secs * 1000)):
        for dof, aid, side in wheels:
            u = dl if side == 0 else dr
            d.ctrl[aid] = max(-Ts, min(Ts, Ts * (u * kv - d.qvel[dof] / wf)))
        mujoco.mj_step(m, d)
        R = d.body('chassis').xmat.reshape(3, 3)
        yaws.append(math.atan2(R[1, 0], R[0, 0]))
        pos.append(d.body('chassis').xpos.copy())
    yaws = np.unwrap(yaws)
    pos = np.array(pos)
    h = len(yaws) // 2
    rate = math.degrees((yaws[-1] - yaws[h]) / (secs / 2))
    speed = float(np.linalg.norm(pos[-1, :2] - pos[h, :2]) / (secs / 2))
    return rate, speed, float(pos[:, 2].max())


if __name__ == '__main__':
    out = {}
    for name, (st, rpm, mf, mr) in VARIANTS.items():
        row = {}
        for duty in (0.35, 0.65, 1.0):
            row[f'spin_{int(duty * 100)}%_deg_s'] = round(sim(st, rpm, mf, mr, -duty, duty, 3.0)[0], 1)
        rate, v, _ = sim(st, rpm, mf, mr, 0.3, 0.9, 3.0)
        row['arc_30/90%_deg_s'] = round(rate, 1)
        row['arc_speed_m_s'] = round(v, 2)
        row['straight_100%_m_s'] = round(sim(st, rpm, mf, mr, 1.0, 1.0, 2.0)[1], 2)
        row['climbs_ramp_at_35%'] = sim(st, rpm, mf, mr, 0.35, 0.35, 8.0, ramp=True)[2] > 0.075
        out[name] = row
        print(name, row, flush=True)
    json.dump(out, open(os.path.join(HERE, 'out', 'turn_study.json'), 'w'), indent=1)
