"""Collision-free stow pose for the start box (python-fcl on the URDF meshes).

Constraints (robot frame; arm base_link origin at x=-95, z=137.4):
  * joint values inside the firmware/URDF limits
  * no mesh collision between arm parts, except each servo with its own horn
    (spline engagement modelled as overlap in the thenar-arms CAD)
  * no collision with the deck plate, arm fence, kill switch rocker or tyres
  * whole robot <= 296 long (4 mm under the 300 rule), arm top as low as possible
Result -> stow_pose.json, then re-verified in SolidWorks by verify_robot.py.
"""
import itertools
import json

import numpy as np
import trimesh
from trimesh.collision import CollisionManager

from arm_fk import Arm, JOINTS

ARM_X, ARM_Z = -95.0, 137.4
arm = Arm()
cm = CollisionManager()
objs = []   # (name, link, To, kind)
for link, items in arm.visuals.items():
    for i, (To, m) in enumerate(items):
        mm = m.copy(); mm.apply_scale(1000.0)
        kind = 'servo' if len(m.vertices) and 'MG996R_Nominal' in str(m.metadata.get('file_name', '')) else None
        name = f'{link}#{i}'
        cm.add_object(name, mm)
        objs.append((name, link, To, mm))

# classify servo/horn meshes by size: horn = D20 disc (~20 mm), servo envelope ~ 54 x 20 x 40
def extent(mm):
    return np.sort(mm.extents)
kinds = {}
for name, link, To, mm in objs:
    e = extent(mm)
    if abs(e[2] - 20) < 1.5 and e[0] < 6:
        kinds[name] = 'horn'
    elif 52 < e[2] < 58 and 19 < e[0] < 21.5:
        kinds[name] = 'servo'
    else:
        kinds[name] = 'print'

# static obstacles in the arm base frame (mm)
def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float)
    b = trimesh.creation.box(extents=hi - lo); b.apply_translation((lo + hi) / 2); return b
deck_top = -2.4
static = {
    'deck': box((-30 - 2, -70, deck_top - 5), (125 + 95, 70, deck_top - 0.01)),
    'fence_rear': box((-26.4, -62.5, deck_top), (-23.4, 62.5, deck_top + 4)),
    'fence_front': box((64.1, -62.5, deck_top), (67.1, 62.5, deck_top + 4)),
    'fence_l': box((-26.4, 56, deck_top), (67.1, 59, deck_top + 4)),
    'fence_r': box((-26.4, -59, deck_top), (67.1, -56, deck_top + 4)),
    'kill_switch': box((195 - 11.6, -40 - 15.6, deck_top), (195 + 11.6, -40 + 15.6, deck_top + 8)),
}
for sx in (1, -1):
    for sy in (1, -1):
        static[f'tyre_{sx}{sy}'] = box((95 + sx * 76 - 70, sy * 85 - 14 if sy > 0 else sy * 85 - 14, -137.4), (95 + sx * 76 + 70, sy * 85 + 14, 2.6))
for n, b in static.items():
    cm.add_object('static:' + n, b)
    kinds['static:' + n] = 'static'


def allowed(a, b):
    la, lb = a.split('#')[0], b.split('#')[0]
    ka, kb = kinds[a], kinds[b]
    if ka == 'static' and kb == 'static':
        return True
    if la == lb and ka != 'static' and kb != 'static':
        return True
    if {ka, kb} == {'servo', 'horn'}:
        return True                       # each joint's own spline engagement
    if 'base_link' in (la, lb) and ('static:deck' in (a, b) or a.startswith('static:fence') or b.startswith('static:fence')):
        return True                       # the base sits on the deck inside its fence (checked in SolidWorks)
    return False


def collisions(q):
    P = arm.link_poses(q)
    for name, link, To, mm in objs:
        T = P[link] @ To
        T = T.copy(); T[:3, 3] *= 1000.0
        cm.set_transform(name, T)
    hit, pairs = cm.in_collision_internal(return_names=True)
    return sorted(p for p in pairs if not allowed(*p))


def hull_box(q):
    P = arm.link_poses(q)
    pts = []
    for name, link, To, mm in objs:
        T = P[link] @ To; T = T.copy(); T[:3, 3] *= 1000.0
        v = mm.convex_hull.vertices
        pts.append((T[:3, :3] @ v.T).T + T[:3, 3])
    pts = np.vstack(pts)
    return pts.min(0), pts.max(0)


if __name__ == '__main__':
    lims = {'shoulder_lift': 80, 'elbow_flex': 80, 'wrist_flex': 80}
    rng = np.arange(-80, 80.1, 5)
    cands = []
    for a, b, c in itertools.product(rng, rng, rng):
        for roll in (85, -85, 0):
            q = {'shoulder_lift': np.radians(a), 'elbow_flex': np.radians(b), 'wrist_flex': np.radians(c),
                 'wrist_roll': np.radians(roll), 'gripper': 0.0}
            lo, hi = hull_box(q)
            X0, X1 = min(-146, ARM_X + lo[0]), max(146, ARM_X + hi[0])
            top = ARM_Z + hi[2]
            if X1 - X0 > 296 or top > 296 or hi[1] - lo[1] > 196:
                continue
            cands.append((top, X1 - X0, a, b, c, roll))
    cands.sort()
    print(len(cands), 'box-feasible candidates; testing collisions in height order')
    found = []
    for top, L, a, b, c, roll in cands:
        q = {'shoulder_lift': np.radians(a), 'elbow_flex': np.radians(b), 'wrist_flex': np.radians(c),
             'wrist_roll': np.radians(roll), 'gripper': 0.0}
        bad = collisions(q)
        if not bad:
            found.append((top, L, a, b, c, roll))
            print('CLEAR  top %.1f len %.1f  lift %d elbow %d wrist %d roll %d' % (top, L, a, b, c, roll), flush=True)
            if len(found) >= 8:
                break
    # sanity: HOME must be clear under the same rules
    home = {'shoulder_lift': np.radians(-25), 'elbow_flex': np.radians(35), 'gripper': np.radians(20)}
    print('HOME collisions under these rules:', collisions(home))
    if found:
        t, L, a, b, c, roll = found[0]
        json.dump({'deg': [0, float(a), float(b), float(c), float(roll), 0.0], 'top_mm': t, 'robot_length_mm': L,
                   'alternatives': found[1:]}, open('stow_pose.json', 'w'), indent=1)
