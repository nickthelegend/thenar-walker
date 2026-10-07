"""Export the assembled robot (tub v2 layout) as one GLB + metadata for the HyperFrames assembly film.

  python cad/tools/export_assembly_scene.py
  -> videos/thenar-assembly/assets/robot.glb   (one node per part, final assembled pose, metres, Y-up)
     videos/thenar-assembly/assets/scene.json  (part list, arm keyframe transforms, wire terminals)

Geometry: printed parts from cad/stl (tub v2 = tub_v2.py), wheels from the 6 mm-bore STLs, the arm from the
MuJoCo model (sim/thenar_walker.xml) posed with mujoco, everything else as simple dimensioned envelopes.
"""
import json
import math
import os

import mujoco
import numpy as np
import trimesh
from shapely.geometry import Point, Polygon, box as sbox

import design as D
import tub_v2 as T
import build_parts as BP

ROOT = D.ROOT
OUT = os.path.join(ROOT, 'videos', 'thenar-assembly', 'assets')
STL = os.path.join(D.CAD, 'stl')
YUP = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1]], float)   # robot Z-up (mm) -> three Y-up
MM = np.diag([0.001, 0.001, 0.001, 1.0])


def stl(name):
    return trimesh.load(os.path.join(STL, name + '.STL'), force='mesh')


def bx(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])


def cz(x, y, r, z0, z1, sec=32):
    return T.cyl_z(x, y, r, z0, z1, sec)


def cy(x, z, r, y0, y1, sec=48):
    return T.cyl_y(x, z, r, y0, y1, sec)


def wheel_tf(stl_mesh, x0, side):
    """Wheel STLs: axis +Z, z = 0 on the inner face. Place on the axle at |y| = WHEEL_INNER."""
    m = stl_mesh.copy()
    if side > 0:
        R = np.array([[1, 0, 0, x0], [0, 0, 1, D.WHEEL_INNER], [0, -1, 0, T.AXLE_Z], [0, 0, 0, 1]], float)
    else:
        R = np.array([[1, 0, 0, x0], [0, 0, -1, -D.WHEEL_INNER], [0, 1, 0, T.AXLE_Z], [0, 0, 0, 1]], float)
    m.apply_transform(R)
    return m


def deck_mesh():
    X, Y = D.DECK_X, D.DECK_Y
    import make_wood_deck_template as W
    poly = sbox(-X, -Y, X, Y)
    for _, x, y, d, _ in W.holes():
        poly = poly.difference(Point(x, y).buffer(d / 2, 24))
    for _, x0, x1, y0, y1, _ in W.windows():
        poly = poly.difference(sbox(x0, y0, x1, y1))
    m = trimesh.creation.extrude_polygon(poly, 3.0)
    m.apply_translation([0, 0, T.TOP])
    return m


def screw(x, y, z_seat, length, d=3.0, head='top'):
    """Socket screw seated at z_seat: head on the 'top' (shank going down) or on the 'bottom' (shank going up)."""
    if head == 'top':
        h, sh = cz(x, y, d * 0.85, z_seat, z_seat + d, 24), cz(x, y, d / 2, z_seat - length, z_seat, 16)
    else:
        h, sh = cz(x, y, d * 0.85, z_seat - d, z_seat, 24), cz(x, y, d / 2, z_seat, z_seat + length, 16)
    return trimesh.util.concatenate([h, sh])


def build_parts():
    P = {}   # name -> (mesh in robot mm, step, color key)

    def add(name, mesh, step, color):
        P[name] = (mesh, step, color)

    add('tub', stl('RR-01_Chassis_Tub'), 'tub', 'petg_blue')
    for i, (x, y) in enumerate(D.BOSS_XY, 1):
        add(f'insert_{i}', cz(x, y, 2.4, T.TOP - 6, T.TOP - 0.3), 'tub', 'brass')
    # motors (envelopes from tub_v2) + collar, shaft, nut
    for sx, xn in ((1, 'F'), (-1, 'R')):
        for sy, yn in ((1, 'L'), (-1, 'R')):
            k = xn + yn
            x, f = sx * T.AXLE_X, sy * T.FACE_Y
            add(f'motor_{k}_gearbox', cy(x, T.AXLE_Z, T.GB_D / 2, f, f - sy * T.GB_L, 64), 'motors', 'gearbox_white')
            add(f'motor_{k}_can', cy(x, T.AXLE_Z, T.CAN_D / 2, f - sy * T.GB_L, f - sy * (T.GB_L + T.CAN_L), 64), 'motors', 'can_silver')
            add(f'motor_{k}_cap', cy(x, T.AXLE_Z, T.CAP_D / 2, f - sy * (T.GB_L + T.CAN_L), f - sy * (T.GB_L + T.CAN_L + T.CAP_L), 48), 'motors', 'cap_yellow')
            add(f'motor_{k}_collar', cy(x, T.AXLE_Z, T.COLLAR_D / 2, f, sy * D.TUB_Y, 32), 'motors', 'gearbox_white')
            add(f'motor_{k}_shaft', cy(x, T.AXLE_Z, T.SHAFT_D / 2, sy * D.TUB_Y, sy * (D.TUB_Y + T.SHAFT_L), 24), 'motors', 'steel')
            nut = cy(x, T.AXLE_Z, T.NUT_AF / math.sqrt(3), sy * (D.TUB_Y - T.NUT_T - 0.2), sy * (D.TUB_Y - 0.2), 6)
            add(f'nut_{k}', nut, 'motors', 'steel')
    # battery cage, screws, battery, bars
    add('cage', stl('RR-03_Battery_Cradle'), 'cage', 'petg_grey')
    cx = T.CAGE_C[0] - T.BATT[0] / 2 - T.CAGE_T - 3
    for i, s in enumerate((1, -1), 1):
        add(f'cage_screw_{i}', screw(cx, s * 40.0, T.Z0, 11, head='bottom'), 'cage', 'steel')
    b = T.BATT
    add('battery', bx(T.CAGE_C[0] - b[0] / 2, T.CAGE_C[0] + b[0] / 2, -b[1] / 2, b[1] / 2, T.Z0 + T.FLOOR_T, T.Z0 + T.FLOOR_T + b[2]), 'battery', 'lipo')
    bar = stl('RR-04_Battery_Bar')
    for i, s in enumerate((1, -1), 1):
        m = bar.copy()
        m.apply_translation([0, s * T.BAR_Y - (bar.bounds[0][1] + bar.bounds[1][1]) / 2, 0])
        add(f'bar_{i}', m, 'battery', 'petg_grey')
    # floor boards
    zf = T.Z0 + T.FLOOR_T + D.STANDOFF
    for k, (x, y, sx_, sy_, h) in T.BOARDS.items():
        col = {'BUCK6V': 'pcb_green', 'PCA9685': 'pcb_blue', 'BUCK5V': 'pcb_green'}[k]
        parts = [bx(x - sx_ / 2, x + sx_ / 2, y - sy_ / 2, y + sy_ / 2, zf, zf + 1.6)]
        if k == 'BUCK6V':
            parts += [bx(x - 20, x + 20, y - 28, y - 8, zf + 1.6, zf + h), bx(x - 20, x + 20, y + 8, y + 28, zf + 1.6, zf + h),
                      cz(x, y, 11, zf + 1.6, zf + 15)]
        elif k == 'PCA9685':
            parts += [bx(x - 20, x + 20, y - 12.7, y - 4.5, zf + 1.6, zf + h), bx(x - 4, x + 4, y + 4, y + 12.7, zf + 1.6, zf + h)]
        else:
            parts += [bx(x - 4, x + 4, y - 4, y + 4, zf + 1.6, zf + h)]
        add(k.lower(), trimesh.util.concatenate(parts), 'boards', col)
        for j, (px, py) in enumerate(T.posts_for(x, y, sx_, sy_), 1):
            add(f'{k.lower()}_screw_{j}', screw(px, py, zf + 1.6, 6, d=2.5), 'boards', 'steel')
    # deck sub-assembly (ESP32 under, kill switch, driver on top)
    add('deck', deck_mesh(), 'deck', 'wood')
    ex, ey, el, ew = T.ESP32
    zb = T.TOP - 10
    add('esp32', trimesh.util.concatenate([bx(ex - el / 2, ex + el / 2, ey - ew / 2, ey + ew / 2, zb - 1.6, zb),
                                           bx(ex - 10, ex + 18, ey - 9, ey + 9, zb - 4.8, zb - 1.6),
                                           bx(ex - el / 2, ex + el / 2, ey - ew / 2, ey - ew / 2 + 2.5, zb, zb + 8.5),
                                           bx(ex - el / 2, ex + el / 2, ey + ew / 2 - 2.5, ey + ew / 2, zb, zb + 8.5)]), 'deck', 'pcb_black')
    for j, (px, py) in enumerate(BP.board_corners('ESP32'), 1):
        add(f'esp32_standoff_{j}', cz(px, py, 2.2, zb, T.TOP, 6), 'deck', 'brass')
    kx, ky = D.KILL_XY
    add('kill_switch', trimesh.util.concatenate([bx(kx - 11.6, kx + 11.6, ky - 15.6, ky + 15.6, T.TOP + 3, T.TOP + 5.5),
                                                 bx(kx - 7, kx + 7, ky - 11, ky + 11, T.TOP + 5.5, T.TOP + 11),
                                                 bx(kx - 10.8, kx + 10.8, ky - 14.8, ky + 14.8, T.TOP + 3 - 32, T.TOP + 3)]), 'deck', 'switch_red')
    bx0, by0, _, _ = D.BOARDS['MDD10A']
    zt = T.TOP + 3 + 6
    md = [bx(bx0 - 42.25, bx0 + 42.25, by0 - 31, by0 + 31, zt, zt + 1.6),
          bx(bx0 - 29 + 0, bx0 + 34, by0 - 28, by0 + 28, zt + 1.6, zt + 5),
          cz(bx0 + 25.3, by0 - 1.2, 5.3, zt + 1.6, zt + 12.6), cz(bx0 + 10.0, by0 - 1.2, 5.3, zt + 1.6, zt + 12.6)]
    add('mdd10a', trimesh.util.concatenate(md), 'deck', 'pcb_black')
    add('mdd10a_terminals', bx(bx0 + 29, bx0 + 38.5, by0 - 15.5, by0 + 15.5, zt + 1.6, zt + 12.5), 'deck', 'terminal_blue')
    add('mdd10a_header', bx(bx0 - 42, bx0 - 34, by0 - 7, by0 + 7, zt + 1.6, zt + 9), 'deck', 'header_white')
    for j, (px, py) in enumerate(BP.board_corners('MDD10A'), 1):
        add(f'mdd10a_spacer_{j}', cz(px, py, 2.6, T.TOP + 3, zt, 12), 'deck', 'nylon')
    for j, (px, py) in enumerate(D.BOSS_XY, 1):
        add(f'deck_screw_{j}', screw(px, py, T.TOP + 3, 10), 'deck_close', 'steel')
    for j, (sx_, sy_) in enumerate(D.ARM_SLOTS, 1):
        add(f'arm_screw_{j}', screw(D.ARM_X + sx_, sy_, T.TOP + 3 + 2.4 + 6, 20), 'arm', 'steel')
    # wheels: front hard one-piece, rear hub + TPU tyre
    fw, hub, tyre = stl('RR-07_Front_Wheel_PETG_6mm'), stl('RR-05_Wheel_Hub_6mm'), stl('RR-06_Wheel_Tyre')
    for sy, yn in ((1, 'L'), (-1, 'R')):
        add(f'wheel_F{yn}', wheel_tf(fw, T.AXLE_X, sy), 'wheels', 'pla_grey')
        add(f'hub_R{yn}', wheel_tf(hub, -T.AXLE_X, sy), 'wheels', 'petg_blue')
        add(f'tyre_R{yn}', wheel_tf(tyre, -T.AXLE_X, sy), 'wheels', 'tpu_black')
    return P


def arm_frames():
    """Arm geoms from the MuJoCo model: mesh in geom frame + world transform for each keyframe pose."""
    m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, 'sim', 'thenar_walker.xml'))
    d = mujoco.MjData(m)
    names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
    qadr = [m.jnt_qposadr[m.joint(n).id] for n in names]
    base = m.body('base_link').id
    geoms = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and _under(m, m.geom_bodyid[g], base)]
    meshes = {}
    for g in geoms:
        mid = m.geom_dataid[g]
        va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        fa, fn = m.mesh_faceadr[mid], m.mesh_facenum[mid]
        meshes[g] = trimesh.Trimesh(m.mesh_vert[va:va + vn].copy() * 1000.0, m.mesh_face[fa:fa + fn].copy(), process=False)
    keys = {'STOW': [0, -80, 80, 50, 85, 0], 'HOME': [0, -25, 35, 0, 0, 20], 'REACH_450': [-3.4, 12.0, -73.8, 61.3, -85.0, 35],
            'WAVE': [-35, -10, 20, 20, 0, 35]}
    frames = {}
    for kname, q in keys.items():
        mujoco.mj_resetData(m, d)
        for a, v in zip(qadr, q):
            d.qpos[a] = math.radians(v)
        mujoco.mj_kinematics(m, d)
        tfs = []
        for g in geoms:
            M = np.eye(4)
            M[:3, :3] = d.geom_xmat[g].reshape(3, 3)
            M[:3, 3] = d.geom_xpos[g] * 1000.0
            # chassis body sits at the origin in this model, so the world frame = robot frame (mm)
            tfs.append(M)
        frames[kname] = tfs
    mats = {g: ('servo_dark' if 'MG996R_Nominal' in m.mesh(m.geom_dataid[g]).name or 'Horn' in m.mesh(m.geom_dataid[g]).name else 'arm_copper') for g in geoms}
    return geoms, meshes, frames, mats


def _under(m, b, root):
    while b > 0:
        if b == root:
            return True
        b = m.body_parentid[b]
    return False


def to_three(M):
    """robot-frame (mm, Z-up) 4x4 -> three-frame (m, Y-up) 4x4."""
    return MM @ YUP @ M


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = build_parts()
    scene = trimesh.Scene()
    meta = {'parts': [], 'arm': {'geoms': [], 'keyframes': {}}}
    for name, (mesh, step, col) in parts.items():
        mm = mesh.copy()
        mm.apply_transform(to_three(np.eye(4)))
        c = mm.bounds.mean(0)
        scene.add_geometry(mm, node_name=name, geom_name=name)
        meta['parts'].append({'name': name, 'step': step, 'color': col, 'center': [round(float(v), 5) for v in c],
                              'faces': int(len(mm.faces))})
    geoms, meshes, frames, mats = arm_frames()
    for i, g in enumerate(geoms):
        n = f'arm_{i:02d}'
        scene.add_geometry(meshes[g].copy().apply_transform(MM), node_name=n, geom_name=n)   # geom-local, metres
        meta['arm']['geoms'].append({'name': n, 'color': mats[g]})
    for k, tfs in frames.items():
        # three node matrix = (robot->three) * world_mm * (mm scale applied to the vertices already) -> fix translation units
        out = []
        for M in tfs:
            Mt = YUP @ M
            Mt[:3, 3] *= 0.001
            out.append([round(float(v), 6) for v in Mt.T.flatten()])     # column-major for THREE.Matrix4.fromArray
        meta['arm']['keyframes'][k] = out
    path = os.path.join(OUT, 'robot.glb')
    scene.export(path)
    json.dump(meta, open(os.path.join(OUT, 'scene.json'), 'w'), indent=0)
    with open(os.path.join(OUT, 'scene.js'), 'w') as f:            # loaded with <script src>, no fetch at render time
        f.write('window.TW_SCENE = ' + json.dumps(meta) + ';' + chr(10))
    tot = sum(p['faces'] for p in meta['parts']) + sum(len(meshes[g].faces) for g in geoms)
    print(f'{len(parts)} parts + {len(geoms)} arm geoms, {tot} faces, glb {os.path.getsize(path) / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
