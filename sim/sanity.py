"""Model sanity checks: settling height, finger-box gap vs jaw-mesh gap, a still render."""
import json
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'thenar_walker.xml'))
d = mujoco.MjData(m)
STOW = [0, -80, 80, 50, 85, 0]
JN = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def set_robot(d, x, y, yaw_deg, q_deg):
    j = m.joint('chassis')
    a = m.jnt_qposadr[j.id]
    d.qpos[a:a + 3] = [x, y, 0.001]
    h = math.radians(yaw_deg) / 2
    d.qpos[a + 3:a + 7] = [math.cos(h), 0, 0, math.sin(h)]
    for n, v in zip(JN, q_deg):
        d.qpos[m.jnt_qposadr[m.joint(n).id]] = math.radians(v)
        d.ctrl[m.actuator('s_' + n).id] = math.radians(v)


set_robot(d, -1.2, -1.25, 90, STOW)
mujoco.mj_forward(m, d)
for _ in range(1500):
    mujoco.mj_step(m, d)
ch = d.body('chassis')
print('settled chassis origin z (mm):', round(d.qpos[m.jnt_qposadr[m.joint('chassis').id] + 2] * 1000, 2), ' should be ~0 (tyres on the floor)')
print('arm joints after settle (deg):', [round(math.degrees(d.qpos[m.jnt_qposadr[m.joint(n).id]]), 1) for n in JN])

gaps = {}
for g in (0, 10, 20, 28, 35, 40, 50, 60, 70):
    dd = mujoco.MjData(m)
    dd.qpos[m.jnt_qposadr[m.joint('gripper').id]] = math.radians(g)
    mujoco.mj_forward(m, dd)
    best = 1e9
    for i in range(m.ngeom):
        if not m.geom(i).name.startswith('finger_fixed'):
            continue
        for k in range(m.ngeom):
            if m.geom(k).name.startswith('finger_jaw'):
                dist = mujoco.mj_geomDistance(m, dd, i, k, 0.2, None)
                best = min(best, dist)
    gaps[g] = round(best * 1000, 1)
mesh = json.load(open(os.path.join(HERE, '..', 'analysis', 'gripper_gap.json')))
print('gripper deg : finger-box gap mm  vs  jaw-mesh gap mm')
for g, v in gaps.items():
    mv = mesh.get(str(g), {}).get('inner_gap_15mm_from_tip', '-')
    print(f'   {g:3d}     : {v:6.1f}            {mv}')

r = mujoco.Renderer(m, 720, 1280)
cam = mujoco.MjvCamera(); cam.lookat[:] = d.body('chassis').xpos + [0, 0, 0.12]; cam.distance = 0.9; cam.azimuth = 210; cam.elevation = -20
r.update_scene(d, cam)
import imageio
imageio.imwrite(os.path.join(HERE, 'out', 'sanity_stow.png'), r.render())
print('render ok')
