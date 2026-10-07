"""REACH_450 / PICK_FLOOR solved for the TRUE grasp centre (object centre between the jaws):
grasp = tool + R @ (-(w/2 + 1.5 mm), 0, 0)   (the URDF tool point sits on the fixed finger).
Collision-checked against the chassis (python-fcl, stow_search4 statics)."""
import json, math, itertools
import numpy as np
from scipy.optimize import least_squares
import stow_search as S, stow_search4 as L
ARM_X, DECK_TOP = -87.0, 103.0
BZ = DECK_TOP + 2.4
L.DECK_TOP = DECK_TOP; L.KILL = (102.0, -40.0); L.set_statics(ARM_X)
NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
def tool(q5):
    T = S.arm.link_poses({n: math.radians(v) for n, v in zip(NAMES, q5)})['gripper_frame_link']
    return T[:3, 3] * 1000 + np.array([ARM_X, 0, BZ]), T[:3, :3]
def solve(target, approach, horiz, half, x0):
    approach = np.array(approach, float)
    lo, hi = [-85, -80, -80, -80, -85], [85, 80, 80, 80, 85]
    def res(q):
        p, R = tool(q); g = p - (half + 1.5) * R[:, 0]
        r = list(g - target) + list(60 * (R[:, 2] - approach))
        if horiz: r.append(60 * R[2, 0])
        return r
    best = None
    for s0 in x0:
        sol = least_squares(res, s0, bounds=(lo, hi))
        if best is None or sol.cost < best.cost: best = sol
    p, R = tool(best.x); g = p - (half + 1.5) * R[:, 0]
    return [round(float(v), 1) for v in best.x], g, R
out = {}
# REACH_450: 50 mm object centred at 450 mm, as far ahead as possible, jaws closing sideways
best = None
for xg in range(330, 180, -10):
    q, g, R = solve(np.array([xg, 0, 450.0]), [1, 0, 0], True, 25, [[0, 20, -60, 40, 85], [0, 10, -70, 60, -85], [0, 30, -50, 20, 85]])
    qd = {n: math.radians(v) for n, v in zip(NAMES, q)}; qd['gripper'] = math.radians(35)
    if np.linalg.norm(g - [xg, 0, 450]) < 1.0 and not L.collide(qd):
        best = (q, g, R); break
q, g, R = best
out['REACH_450'] = {'deg': q + [35.0], 'grasp_centre_mm': g.round(1).tolist(), 'closing_axis': R[:, 0].round(3).tolist()}
# PICK_FLOOR: 50 mm cube on the floor, centre z = 25, vertical approach, as far ahead as possible
for xg in range(300, 200, -5):
    q, g, R = solve(np.array([xg, 0, 25.0]), [0, 0, -1], False, 25, [[0, 75, -60, 60, 0], [0, 80, -67, 65, 0]])
    qd = {n: math.radians(v) for n, v in zip(NAMES, q)}; qd['gripper'] = math.radians(35)
    if np.linalg.norm(g - [xg, 0, 25]) < 1.0 and not L.collide(qd):
        out['PICK_FLOOR'] = {'deg': q + [35.0], 'grasp_centre_mm': g.round(1).tolist(), 'closing_axis': R[:, 0].round(3).tolist()}; break
for k, v in out.items(): print(k, v)
json.dump(out, open('poses_grasp_centre.json', 'w'), indent=1)
