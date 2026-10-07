"""Low-deck layout: arm base on the deck at DECK_TOP; all electronics on the tub floor.
Finds collision-free stow poses + arm mount x for the 300 x 200 x 300 start box."""
import itertools, json, sys
import numpy as np, trimesh
import stow_search as S
DECK_TOP = float(sys.argv[1]) if len(sys.argv) > 1 else 103.0
KILL = (100.0, -40.0)
def box(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float); b = trimesh.creation.box(extents=hi - lo); b.apply_translation((lo + hi) / 2); return b
def statics(arm_x):
    st = {'deck': box((-125, -70, DECK_TOP - 5), (125, 70, DECK_TOP - 0.01)),
          'kill_switch': box((KILL[0] - 11.6, KILL[1] - 15.6, DECK_TOP), (KILL[0] + 11.6, KILL[1] + 15.6, DECK_TOP + 8))}
    fx0, fx1 = arm_x - 22.9, arm_x + 64.1
    st.update({'fence_rear': box((fx0 - 3, -59, DECK_TOP), (fx0, 59, DECK_TOP + 4)), 'fence_front': box((fx1, -59, DECK_TOP), (fx1 + 3, 59, DECK_TOP + 4)),
               'fence_l': box((fx0 - 3, 56, DECK_TOP), (fx1 + 3, 59, DECK_TOP + 4)), 'fence_r': box((fx0 - 3, -59, DECK_TOP), (fx1 + 3, -56, DECK_TOP + 4))})
    for sx in (1, -1):
        for sy in (1, -1):
            st[f'tyre_{sx}{sy}'] = box((sx * 76 - 70, min(sy * 71, sy * 99), 0), (sx * 76 + 70, max(sy * 71, sy * 99), 140))
    return st
def set_statics(arm_x):
    for n in [k for k in S.kinds if k.startswith('static:')]:
        S.cm.remove_object(n); del S.kinds[n]
    off = np.array([arm_x, 0, DECK_TOP + 2.4])
    for n, b in statics(arm_x).items():
        b = b.copy(); b.apply_translation(-off); S.cm.add_object('static:' + n, b); S.kinds['static:' + n] = 'static'
def collide(q):
    P = S.arm.link_poses(q)
    for name, link, To, mm in S.objs:
        T = P[link] @ To; T = T.copy(); T[:3, 3] *= 1000.0; S.cm.set_transform(name, T)
    _, pairs = S.cm.in_collision_internal(return_names=True)
    return sorted(p for p in pairs if not S.allowed(*p))
if __name__ == '__main__':
    rng = np.arange(-80, 80.1, 5)
    out = []
    for a, b, c in itertools.product(rng, rng, rng):
        for roll in (85, 0):
            q = {'shoulder_lift': np.radians(a), 'elbow_flex': np.radians(b), 'wrist_flex': np.radians(c), 'wrist_roll': np.radians(roll), 'gripper': 0.0}
            lo, hi = S.hull_box(q)
            top = DECK_TOP + 2.4 + hi[2]
            if top > 294 or lo[2] < -2.5 or hi[0] - lo[0] > 292:
                continue
            out.append((top, a, b, c, roll, lo, hi))
    out.sort(key=lambda r: r[0]); print(len(out), 'height-feasible', flush=True)
    best = []
    for top, a, b, c, roll, lo, hi in out:
        q = {'shoulder_lift': np.radians(a), 'elbow_flex': np.radians(b), 'wrist_flex': np.radians(c), 'wrist_roll': np.radians(roll), 'gripper': 0.0}
        for ax in np.arange(-99, -40, 2.0):
            X0, X1 = min(-146, ax + lo[0]), max(146, ax + hi[0])
            if X1 - X0 > 294:
                continue
            set_statics(ax)
            if not collide(q):
                best.append({'top': round(float(top), 1), 'length': round(float(X1 - X0), 1), 'arm_x': float(ax), 'deg': [0, float(a), float(b), float(c), float(roll), 0],
                             'arm_box_robot': [round(float(ax + lo[0]), 1), round(float(ax + hi[0]), 1), round(float(DECK_TOP + 2.4 + lo[2]), 1), round(float(top), 1)]})
                print('CLEAR', best[-1], flush=True); break
        if len(best) >= 6: break
    json.dump(best, open('stow_lowdeck.json', 'w'), indent=1)
