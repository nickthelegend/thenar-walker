"""Fingertip opening of the R3 follower gripper vs gripper joint angle (URDF meshes).
Tip of each finger = its vertex farthest along the tool approach axis; opening = distance
between the moving-jaw tip and the fixed finger measured perpendicular to the approach axis
at 15 mm behind the tips (where a cube is actually held)."""
import json, numpy as np
from arm_fk import Arm
arm = Arm()
def verts(link, q):
    P = arm.link_poses(q)
    return np.vstack([(P[link] @ To @ np.c_[m.vertices, np.ones(len(m.vertices))].T).T[:, :3] * 1000 for To, m in arm.visuals[link] if len(m.vertices) > 200])
out = {}
for g in range(0, 71, 5):
    q = {'gripper': np.radians(g)}
    T = arm.link_poses(q)['gripper_frame_link']; tool = T[:3, 3] * 1000; a = T[:3, 2]
    jaw = verts('moving_jaw_so101_v1_link', q); body = verts('gripper_link', q)
    sj = (jaw - tool) @ a; sb = (body - tool) @ a
    tip_j, tip_b = sj.max(), sb.max()
    s = min(tip_j, tip_b) - 15          # station 15 mm behind the shorter tip
    band_j = jaw[np.abs(sj - s) < 2]; band_b = body[np.abs(sb - s) < 2]
    # lateral direction = from fixed finger centroid to jaw centroid, perpendicular to a
    lat = band_j.mean(0) - band_b.mean(0); lat -= (lat @ a) * a; lat /= np.linalg.norm(lat)
    gap = (band_j @ lat).min() - (band_b @ lat).max()
    out[g] = {'tip_to_tip_mm': round(float(np.linalg.norm(jaw[sj.argmax()] - body[sb.argmax()])), 1), 'inner_gap_15mm_from_tip': round(float(gap), 1)}
for k, v in out.items(): print(k, v)
json.dump(out, open('gripper_gap.json', 'w'), indent=1)
