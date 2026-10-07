"""Generate sim/thenar_walker.xml — a MuJoCo model of Thenar Walker in the RoboReach arena.

Sources (nothing invented silently; every assumption is listed in ASSUMPTIONS below):
  * arm body tree + visual meshes: thenar-arms simulation/mujoco/so101.xml (URDF-validated kinematics)
  * link masses: SolidWorks volumes (cad/evidence/verification.json) + MG996R 55 g, horn 6 g;
    COM / inertia: integrated over the actual meshes with those masses (trimesh)
  * finger contact boxes: fitted to the fingertip zone of the jaw meshes (3 per finger)
  * chassis / wheels / arena obstacles: cad/tools/design.py, terrain.py, build_arena.py
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, 'cad', 'tools'))
import design as D       # noqa: E402
import terrain as T      # noqa: E402

SRC = os.path.join(HERE, 'arm_src')           # copy of thenar-arms simulation/mujoco
OUT = os.path.join(HERE, 'thenar_walker.xml')

ASSUMPTIONS = {
    'tyre_friction_TPU_on_ply': 0.9,        # front tyres (RR-06 printed in TPU 95A)
    'tyre_friction_PETG_on_ply': 0.35,      # PETG REAR tyres stall on the speed breakers (rejected); PETG FRONT tyres tested below
    'front_tyre_material': 'PETG',          # hard front tyres let the front slide sideways -> turns (turn_study.py)
    'rear_tyre_material': 'TPU',
    'finger_friction': 1.0,
    'object_friction_thermocol': 0.7,
    'cube50_mass_kg': 0.020,          # EPS cube is ~3 g; 20 g is deliberately pessimistic for the grip
    'cube60_mass_kg': 0.030,
    'servo_stall_Nm': 1.08,           # MG996R 11 kg.cm at 6 V
    'servo_kp_Nm_per_rad': 30.0,      # MG996R reaches stall within ~2 deg of error
    'gripper_kp_Nm_per_rad': 3.0,
    'servo_armature': 0.004,
    'wheel_armature': 0.004,          # 370 rotor ~7e-7 kg m2 x 78^2
    'motor': '25GA-370 / JGA25-370 12 V 103:1 (60 rpm free, rated 2.7 kg.cm @ 46 rpm, stall 8.2 kg.cm / 1.3 A)',
    'motor_stall_Nm': 8.2 * 0.0980665,
    'motor_free_rad_s': 60 * 2 * math.pi / 60,
    'battery_V_over_12': 11.6 / 12.0,
}
MESH_MASS_G = {'MG996R_Nominal_R3': 55.0, 'Metal_Horn_D20_PCD14_R3': 6.0, 'Base_MG996R_R3': 100.0,
               'Shoulder_MG996R_R3': 51.3, 'Upper_arm_MG996R_R3': 75.6, 'Forearm_MG996R_R3': 69.2,
               'Wrist_pitch_roll_MG996R_R3': 20.2, 'Gripper_body_MG996R_R3': 37.8, 'Moving_jaw_MG996R_R3': 13.6}


def quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def mat_to_quat(R):
    q = trimesh.transformations.quaternion_from_matrix(np.vstack([np.c_[R, [0, 0, 0]], [0, 0, 0, 1]]))
    return q  # (w, x, y, z)


def fmt(v):
    return ' '.join(f'{x:.6g}' for x in v)


def geom_tf(g):
    T4 = np.eye(4)
    T4[:3, 3] = [float(x) for x in g.get('pos', '0 0 0').split()]
    T4[:3, :3] = quat_to_mat([float(x) for x in g.get('quat', '1 0 0 0').split()])
    return T4


_mesh_cache = {}


def mesh(name):
    if name not in _mesh_cache:
        _mesh_cache[name] = trimesh.load(os.path.join(SRC, 'meshes', name + '.stl'), force='mesh')
    return _mesh_cache[name]


def body_inertial(body):
    """Mass, COM and inertia tensor of a body from its visual mesh geoms."""
    tot, mom, parts = 0.0, np.zeros(3), []
    for g in body.findall('geom'):
        n = g.get('mesh')
        m = mesh(n).copy(); m.apply_transform(geom_tf(g))
        mass = MESH_MASS_G[n] / 1000.0
        mp = m.mass_properties               # unit density
        scale = mass / mp['mass']
        I = np.array(mp['inertia']) * scale   # about this mesh's COM
        parts.append((mass, np.array(mp['center_mass']), I))
        tot += mass; mom += mass * np.array(mp['center_mass'])
    com = mom / tot
    I_tot = np.zeros((3, 3))
    for mass, c, I in parts:
        d = c - com
        I_tot += I + mass * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return tot, com, I_tot


def finger_boxes(verts, tool_R, tool_p, n_seg=3, depth=0.042):
    """Boxes (in the given frame) covering the fingertip zone of a finger mesh.

    verts: mesh vertices in that frame (m). tool_R/tool_p: tool frame in that frame.
    Returns list of (centre, quat, half_sizes) in metres."""
    loc = (verts - tool_p) @ tool_R          # coordinates in the tool frame (x, y, approach z)
    s = loc[:, 2]
    tip = s.max()
    out = []
    edges = np.linspace(tip - depth, tip, n_seg + 1)
    for a, b in zip(edges[:-1], edges[1:]):
        sel = loc[(s >= a) & (s <= b)]
        if len(sel) < 4:
            continue
        lo, hi = sel.min(0), sel.max(0)
        lo[2], hi[2] = a, b
        c_tool = (lo + hi) / 2
        half = np.maximum((hi - lo) / 2, 0.0015)
        out.append((tool_p + tool_R @ c_tool, mat_to_quat(tool_R), half))
    return out


def build_arm(arm_parent):
    tree = ET.parse(os.path.join(SRC, 'so101.xml'))
    base = tree.getroot().find('worldbody/body')
    bodies = {b.get('name'): b for b in base.iter('body')}
    for name, b in bodies.items():
        m, com, I = body_inertial(b)
        inert = ET.Element('inertial', {'pos': fmt(com), 'mass': f'{m:.5f}',
                                        'fullinertia': fmt([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]])})
        b.insert(0, inert)
        for g in b.findall('geom'):
            g.set('contype', '0'); g.set('conaffinity', '0'); g.set('group', '1'); g.set('material', 'arm')
            if 'MG996R_Nominal' in g.get('mesh') or 'Horn' in g.get('mesh'):
                g.set('material', 'servo')
        j = b.find('joint')
        if j is not None:
            j.set('armature', str(ASSUMPTIONS['servo_armature'])); j.set('damping', '0.05'); j.set('frictionloss', '0.01')
    # tool site (URDF gripper_tool_fixed: xyz -0.0079 -0.000218121 -0.0981274, rpy pi 0 pi)
    g_body = bodies['gripper_link']
    tool_p = np.array([-0.0079, -0.000218121, -0.0981274])
    tool_R = np.diag([-1.0, 1.0, -1.0])
    ET.SubElement(g_body, 'site', {'name': 'tool', 'pos': fmt(tool_p), 'quat': fmt(mat_to_quat(tool_R)), 'size': '0.004', 'rgba': '1 0 0 0.6'})
    # fixed finger boxes from the gripper body mesh
    gb = [g for g in g_body.findall('geom') if g.get('mesh') == 'Gripper_body_MG996R_R3'][0]
    v = trimesh.transform_points(mesh('Gripper_body_MG996R_R3').vertices, geom_tf(gb))
    boxes = {'fixed': finger_boxes(v, tool_R, tool_p)}
    # moving jaw boxes in the jaw frame (tool frame expressed there at gripper = 0)
    jb = bodies['moving_jaw_so101_v1_link']
    Tj = np.eye(4); Tj[:3, 3] = [float(x) for x in jb.get('pos').split()]; Tj[:3, :3] = quat_to_mat([float(x) for x in jb.get('quat').split()])
    Tt = np.eye(4); Tt[:3, :3] = tool_R; Tt[:3, 3] = tool_p
    Tjt = np.linalg.inv(Tj) @ Tt
    jg = [g for g in jb.findall('geom') if g.get('mesh') == 'Moving_jaw_MG996R_R3'][0]
    vj = trimesh.transform_points(mesh('Moving_jaw_MG996R_R3').vertices, geom_tf(jg))
    boxes['jaw'] = finger_boxes(vj, Tjt[:3, :3], Tjt[:3, 3])
    for key, body in (('fixed', g_body), ('jaw', jb)):
        for i, (c, q, h) in enumerate(boxes[key]):
            ET.SubElement(body, 'geom', {'name': f'finger_{key}_{i}', 'type': 'box', 'pos': fmt(c), 'quat': fmt(q), 'size': fmt(h),
                                         'class': 'finger'})
    base.set('pos', fmt([D.ARM_X / 1000, 0, (D.DECK_TOP + D.ARM_BASE_DROP) / 1000]))
    arm_parent.append(base)
    return [b for b in bodies]


def chassis_inertial():
    v = json.load(open(os.path.join(D.EVIDENCE, 'verification.json')))
    rows = v['configs']['STOW']['mass_rows']
    keep = [r for r in rows if 'SO101_Follower' not in r['component'] and 'Wheel' not in r['component']]
    m = sum(r['mass_g'] for r in keep) + D.MASS['wiring_misc'] + D.MASS['fasteners']
    com = (sum(np.array(r['centroid_mm']) * r['mass_g'] for r in keep) + np.array([0, 0, 100.0]) * 200) / m
    return m / 1000, com / 1000


def build():
    root = ET.Element('mujoco', {'model': 'thenar_walker_roboreach'})
    ET.SubElement(root, 'compiler', {'angle': 'radian', 'meshdir': '.', 'inertiafromgeom': 'auto', 'autolimits': 'true'})
    ET.SubElement(root, 'option', {'timestep': '0.001', 'integrator': 'implicitfast', 'cone': 'elliptic', 'impratio': '5'})
    vis = ET.SubElement(root, 'visual')
    ET.SubElement(vis, 'global', {'offwidth': '1280', 'offheight': '720'})
    ET.SubElement(vis, 'quality', {'shadowsize': '4096'})
    dft = ET.SubElement(root, 'default')
    f = ET.SubElement(dft, 'default', {'class': 'finger'})
    ET.SubElement(f, 'geom', {'friction': f"{ASSUMPTIONS['finger_friction']} 0.02 0.002", 'condim': '4', 'rgba': '0.1 0.8 0.3 0.35',
                              'solref': '0.004 1', 'solimp': '0.95 0.99 0.001', 'contype': '2', 'conaffinity': '2', 'group': '3'})
    ob = ET.SubElement(dft, 'default', {'class': 'object'})
    ET.SubElement(ob, 'geom', {'friction': f"{ASSUMPTIONS['object_friction_thermocol']} 0.02 0.002", 'condim': '4', 'contype': '3',
                               'conaffinity': '3', 'solref': '0.004 1', 'solimp': '0.95 0.99 0.001'})
    st = ET.SubElement(dft, 'default', {'class': 'static'})
    ET.SubElement(st, 'geom', {'contype': '1', 'conaffinity': '1', 'friction': f"{ASSUMPTIONS['tyre_friction_TPU_on_ply']} 0.01 0.001"})
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', {'name': 'sky', 'type': 'skybox', 'builtin': 'gradient', 'rgb1': '0.75 0.8 0.88', 'rgb2': '0.95 0.96 0.98', 'width': '256', 'height': '256'})
    ET.SubElement(asset, 'texture', {'name': 'board', 'type': '2d', 'builtin': 'checker', 'rgb1': '0.22 0.23 0.25', 'rgb2': '0.25 0.26 0.28', 'width': '512', 'height': '512'})
    ET.SubElement(asset, 'material', {'name': 'board', 'texture': 'board', 'texrepeat': '30 30'})
    for n, rgba in {'arm': '0.62 0.2 0.08 1', 'servo': '0.08 0.08 0.1 1', 'petg': '0.16 0.18 0.21 1', 'deck': '0.1 0.22 0.45 1',
                    'tyre': '0.05 0.05 0.05 1', 'petg_tyre': '0.55 0.57 0.6 1', 'hub': '0.1 0.22 0.45 1', 'wood': '0.86 0.74 0.55 1', 'wall': '0.82 0.83 0.85 1',
                    'beige': '0.9 0.8 0.62 1', 'blue': '0.1 0.3 0.75 1', 'red': '0.8 0.08 0.08 1', 'metal': '0.7 0.7 0.72 1'}.items():
        ET.SubElement(asset, 'material', {'name': n, 'rgba': rgba})
    for n in MESH_MASS_G:
        ET.SubElement(asset, 'mesh', {'name': n, 'file': os.path.join('arm_src', 'meshes', n + '.stl')})
    stl = os.path.join(D.CAD, 'stl')
    for n in ('RR-01_Chassis_Tub', 'RR-02_Deck_Plate', 'RR-05_Wheel_Hub', 'RR-06_Wheel_Tyre'):
        ET.SubElement(asset, 'mesh', {'name': n, 'file': os.path.relpath(os.path.join(stl, n + '.STL'), HERE), 'scale': '0.001 0.001 0.001'})
    # ramp + bridge + ramp-down as one convex prism (it is convex)
    prof = [(T.X_UP, 0), (T.X_TOP0, T.H), (T.X_TOP1, T.H), (T.X_DOWN, 0)]
    x0, x1 = -1.400, -1.000
    y0 = -0.800
    verts = []
    for xx in (x0, x1):
        for s, z in prof:
            verts += [xx, y0 + s / 1000, z / 1000]
    ET.SubElement(asset, 'mesh', {'name': 'ramp', 'vertex': fmt(verts)})

    wb = ET.SubElement(root, 'worldbody')
    ET.SubElement(wb, 'light', {'pos': '-1 -1 3', 'dir': '0.3 0.3 -1', 'directional': 'true', 'castshadow': 'true', 'diffuse': '0.8 0.8 0.8'})
    ET.SubElement(wb, 'light', {'pos': '1 1 3', 'dir': '-0.3 -0.3 -1', 'directional': 'true', 'castshadow': 'false', 'diffuse': '0.35 0.35 0.35'})
    ET.SubElement(wb, 'geom', {'name': 'floor', 'type': 'plane', 'size': '1.5 1.5 0.01', 'material': 'board', 'class': 'static'})

    def sgeom(name, **kw):
        a = {'name': name, 'class': 'static'}; a.update({k: (fmt(v) if isinstance(v, (list, tuple, np.ndarray)) else str(v)) for k, v in kw.items()})
        return ET.SubElement(wb, 'geom', a)

    def marking(name, cx, cy, w, h, mat='beige'):
        ET.SubElement(wb, 'geom', {'name': name, 'type': 'box', 'pos': fmt([cx, cy, 0.0005]), 'size': fmt([w / 2, h / 2, 0.0005]),
                                   'material': mat, 'contype': '0', 'conaffinity': '0'})

    sgeom('ramp', type='mesh', mesh='ramp', material='wood')
    lane_c = (-1.475 - 0.9125) / 2
    for i, c in enumerate(T.BREAKERS):
        sgeom(f'breaker_{i}', type='cylinder', pos=[lane_c, y0 + c / 1000, 0], size=[T.R_BREAK / 1000, (1.475 - 0.9125) / 2],
              quat=[0.7071068, 0, 0.7071068, 0], material='wood')
    walls = [(-1.5, -1.475, -1.5, 1.5), (-1.5, 1.5, -1.5, -1.475), (-0.9125, -0.8875, -1.1, 0.7)]
    for i, (a, b, c, d) in enumerate(walls):
        sgeom(f'wall_{i}', type='box', pos=[(a + b) / 2, (c + d) / 2, 0.03], size=[(b - a) / 2, (d - c) / 2, 0.03], material='wall')
    marking('start_zone', -1.25, -1.25, 0.45, 0.45)
    marking('pickup_spot', -1.2, -0.925, 0.1, 0.1)
    marking('touchdown_80', *TOUCHDOWN, 0.08, 0.08)
    # Downy: 40 mm post, 80 x 80 plate at 400 mm, target square on the floor
    px, py = POLE
    sgeom('pole_post', type='box', pos=[px, py, 0.195], size=[0.02, 0.02, 0.195], material='wood')
    sgeom('pole_top', type='box', pos=[px, py, 0.395], size=[0.04, 0.04, 0.005], material='wood')
    marking('downy_target', *DOWNY_TARGET, 0.1, 0.1)

    # objects
    for name, pos, half, mass in (('cube50', [-1.2, -0.925, 0.0252], 0.025, ASSUMPTIONS['cube50_mass_kg']),
                                  ('cube60', [px, py, 0.4002 + 0.03], 0.030, ASSUMPTIONS['cube60_mass_kg'])):
        b = ET.SubElement(wb, 'body', {'name': name, 'pos': fmt(pos)})
        ET.SubElement(b, 'freejoint', {'name': name})
        ET.SubElement(b, 'geom', {'name': name, 'type': 'box', 'size': fmt([half] * 3), 'mass': str(mass), 'material': 'blue', 'class': 'object'})

    # ---------------------------------------------------------------- robot
    m_ch, com_ch = chassis_inertial()
    rb = ET.SubElement(wb, 'body', {'name': 'chassis', 'pos': '0 0 0'})
    ET.SubElement(rb, 'freejoint', {'name': 'chassis'})
    ET.SubElement(rb, 'inertial', {'pos': fmt(com_ch), 'mass': f'{m_ch:.4f}',
                                   'diaginertia': fmt([m_ch / 12 * (0.138 ** 2 + 0.05 ** 2), m_ch / 12 * (0.24 ** 2 + 0.05 ** 2), m_ch / 12 * (0.24 ** 2 + 0.138 ** 2)])})
    ET.SubElement(rb, 'geom', {'name': 'tub_visual', 'type': 'mesh', 'mesh': 'RR-01_Chassis_Tub', 'material': 'petg', 'contype': '0', 'conaffinity': '0'})
    ET.SubElement(rb, 'geom', {'name': 'deck_visual', 'type': 'mesh', 'mesh': 'RR-02_Deck_Plate', 'material': 'deck', 'contype': '0', 'conaffinity': '0'})
    # collision proxies: belly/tub, deck, motor cans (lowest points) — any contact of these with obstacles = scraping
    ET.SubElement(rb, 'geom', {'name': 'col_tub', 'type': 'box', 'pos': fmt([0, 0, (D.FLOOR_Z0 + D.TUB_TOP_Z) / 2000]),
                               'size': fmt([D.TUB_X / 1000, D.TUB_Y / 1000, (D.TUB_TOP_Z - D.FLOOR_Z0) / 2000]), 'rgba': '1 0 0 0', 'contype': '1', 'conaffinity': '1'})
    ET.SubElement(rb, 'geom', {'name': 'col_deck', 'type': 'box', 'pos': fmt([0, 0, (D.DECK_Z0 + D.DECK_TOP) / 2000]),
                               'size': fmt([D.DECK_X / 1000, D.DECK_Y / 1000, D.DECK_T / 2000]), 'rgba': '1 0 0 0', 'contype': '1', 'conaffinity': '1'})
    for sx in (1, -1):
        for sy in (1, -1):
            yc = sy * (D.MOTOR_FACE_Y - 28) / 1000
            ET.SubElement(rb, 'geom', {'name': f'col_motor_{sx}{sy}', 'type': 'cylinder', 'pos': fmt([sx * D.AXLE_X / 1000, yc, D.AXLE_Z / 1000]),
                                       'size': '0.0125 0.028', 'quat': '0.7071068 0.7071068 0 0', 'material': 'metal', 'contype': '1', 'conaffinity': '1'})
    ET.SubElement(rb, 'geom', {'name': 'kill_switch', 'type': 'box', 'pos': fmt([D.KILL_XY[0] / 1000, D.KILL_XY[1] / 1000, (D.DECK_TOP + 4) / 1000]),
                               'size': '0.0116 0.0156 0.004', 'material': 'red', 'contype': '0', 'conaffinity': '0'})
    ET.SubElement(rb, 'site', {'name': 'chassis_centre', 'pos': '0 0 0.08', 'size': '0.005'})
    for sx, xn in ((1, 'F'), (-1, 'R')):
        for sy, yn in ((1, 'L'), (-1, 'R')):
            w = ET.SubElement(rb, 'body', {'name': f'wheel_{xn}{yn}', 'pos': fmt([sx * D.AXLE_X / 1000, sy * D.WHEEL_YC / 1000, D.AXLE_Z / 1000])})
            ET.SubElement(w, 'joint', {'name': f'wheel_{xn}{yn}', 'type': 'hinge', 'axis': '0 1 0', 'armature': str(ASSUMPTIONS['wheel_armature']), 'damping': '0.002'})
            ET.SubElement(w, 'inertial', {'pos': '0 0 0', 'mass': '0.112', 'diaginertia': '0.00026 0.00048 0.00026'})
            ET.SubElement(w, 'geom', {'name': f'tyre_{xn}{yn}', 'type': 'cylinder', 'size': fmt([D.WHEEL_D / 2000, 0.0125]), 'quat': '0.7071068 0.7071068 0 0',
                                      'rgba': '0 0 0 0', 'class': 'static', 'condim': '6', 'priority': '1',
                                      'friction': f"{ASSUMPTIONS['tyre_friction_TPU_on_ply'] if ASSUMPTIONS[('front' if xn == 'F' else 'rear') + '_tyre_material'] == 'TPU' else ASSUMPTIONS['tyre_friction_PETG_on_ply']} 0.01 0.001"})
            q = '0.7071068 -0.7071068 0 0' if sy > 0 else '0.7071068 0.7071068 0 0'
            off = fmt([0, -sy * D.WHEEL_W / 2000, 0])
            ET.SubElement(w, 'geom', {'type': 'mesh', 'mesh': 'RR-05_Wheel_Hub', 'quat': q, 'pos': off, 'material': 'hub', 'contype': '0', 'conaffinity': '0'})
            ET.SubElement(w, 'geom', {'type': 'mesh', 'mesh': 'RR-06_Wheel_Tyre', 'quat': q, 'pos': off, 'material': 'tyre' if ASSUMPTIONS[('front' if xn == 'F' else 'rear') + '_tyre_material'] == 'TPU' else 'petg_tyre', 'contype': '0', 'conaffinity': '0'})
    build_arm(rb)

    act = ET.SubElement(root, 'actuator')
    for xn in 'FR':
        for yn in 'LR':
            ET.SubElement(act, 'motor', {'name': f'm_{xn}{yn}', 'joint': f'wheel_{xn}{yn}', 'ctrlrange': '-1 1', 'gear': '1'})
    for jn in ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'):
        kp = ASSUMPTIONS['gripper_kp_Nm_per_rad'] if jn == 'gripper' else ASSUMPTIONS['servo_kp_Nm_per_rad']
        ET.SubElement(act, 'position', {'name': 's_' + jn, 'joint': jn, 'kp': str(kp), 'kv': '0.6' if jn != 'gripper' else '0.15',
                                        'forcerange': f"-{ASSUMPTIONS['servo_stall_Nm']} {ASSUMPTIONS['servo_stall_Nm']}", 'ctrlrange': '-1.5 1.5'})
    sens = ET.SubElement(root, 'sensor')
    ET.SubElement(sens, 'framepos', {'name': 'tool_pos', 'objtype': 'site', 'objname': 'tool'})
    ET.indent(root)
    ET.ElementTree(root).write(OUT, encoding='utf-8', xml_declaration=True)
    json.dump(ASSUMPTIONS, open(os.path.join(HERE, 'assumptions.json'), 'w'), indent=1)
    print('wrote', OUT)


TOUCHDOWN = (-1.15, 0.96)
POLE = (1.1, 0.8)
DOWNY_TARGET = (0.6, 0.7)

if __name__ == '__main__':
    import shutil
    src = r'C:\Users\testi\AppData\Local\Temp\claude\F--Projects-thenar-walker\e7e9f5d8-c635-40b8-9fd5-aea8a71a5c7f\scratchpad\thenar-arms\simulation\mujoco'
    if not os.path.isdir(SRC):
        shutil.copytree(src, SRC)
    build()
    import mujoco
    m = mujoco.MjModel.from_xml_path(OUT)
    print('compiled: nbody', m.nbody, 'njnt', m.njnt, 'nu', m.nu, 'total mass', round(float(sum(m.body_mass)), 3), 'kg')
