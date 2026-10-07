"""Re-solve REACH_450 / PICK_FLOOR for the low-deck layout, collision-checked against chassis statics."""
import itertools, json, numpy as np
import stow_search as S, stow_search4 as L
ARM_X, DECK_TOP = -87.0, 103.0
BZ = DECK_TOP + 2.4
L.DECK_TOP = DECK_TOP; L.KILL = (102.0, -40.0)
L.set_statics(ARM_X)
def tool(q):
    T = S.arm.link_poses(q)['gripper_frame_link']; return T[:3, 3] * 1000 + np.array([ARM_X, 0, BZ]), T[:3, :3]
rng = np.arange(-80, 80.1, 2.5)
reach, floor = [], []
for a, b, c in itertools.product(rng, rng, rng):
    q = {'shoulder_lift': np.radians(a), 'elbow_flex': np.radians(b), 'wrist_flex': np.radians(c), 'gripper': np.radians(35)}
    p, R = tool(q); ap = R[:, 2]
    if abs(p[2] - 450) < 1.5 and abs(ap[2]) < 0.08: reach.append((-p[0], a, b, c, p))
    if abs(p[2] - 25) < 1.5 and ap[2] < -0.97 and p[0] > 76 + 70 + 25 + 10: floor.append((-p[0], a, b, c, p))
def pick(lst, name):
    lst.sort(key=lambda r: r[0])
    for negx, a, b, c, p in lst:
        q = {'shoulder_lift': np.radians(a), 'elbow_flex': np.radians(b), 'wrist_flex': np.radians(c), 'gripper': np.radians(35)}
        if not L.collide(q):
            print(name, 'deg', [0, a, b, c, 0, 35], 'grasp', p.round(1)); return [0, float(a), float(b), float(c), 0.0, 35.0]
    print(name, 'NONE')
out = {'REACH_450': pick(reach, 'REACH_450'), 'PICK_FLOOR': pick(floor, 'PICK_FLOOR')}
json.dump(out, open('poses_lowdeck.json', 'w'), indent=1)
