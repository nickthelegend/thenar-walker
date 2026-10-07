"""Forward kinematics of the Thenar R3 follower from its URDF + meshes.

Used to size the mobile base: grasp height reach (rule: grasp object centre at
450 mm) and the folded start pose (rule: whole bot inside 300 x 200 x 300 mm).
All lengths returned in millimetres in the arm base_link frame (Z up, z=0 is the
underside of the printed base that bolts to the deck).
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import trimesh

HERE = Path(__file__).parent
URDF = HERE / 'so101_description/urdf/so101.urdf'
JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                     [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                     [-sp, cp*sr, cp*cr]])


def tf(origin):
    T = np.eye(4)
    if origin is None:
        return T
    T[:3, 3] = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    T[:3, :3] = rpy(*[float(v) for v in origin.get('rpy', '0 0 0').split()])
    return T


def axis_rot(axis, a):
    k = np.asarray(axis, float); k /= np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4); T[:3, :3] = np.eye(3) + np.sin(a)*K + (1-np.cos(a))*K@K
    return T


class Arm:
    def __init__(self):
        root = ET.parse(URDF).getroot()
        self.joints = {}
        for j in root.findall('joint'):
            ax = j.find('axis'); lim = j.find('limit')
            self.joints[j.get('name')] = dict(
                parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                T=tf(j.find('origin')), axis=[float(v) for v in ax.get('xyz').split()] if ax is not None else None,
                lim=(float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else (0, 0),
                type=j.get('type'))
        self.visuals = {}
        for link in root.findall('link'):
            items = []
            for v in link.findall('visual'):
                fn = v.find('geometry/mesh').get('filename').replace('package://so101_description/', '')
                m = trimesh.load(HERE / 'so101_description' / fn, force='mesh')
                items.append((tf(v.find('origin')), m))
            self.visuals[link.get('name')] = items
        self.child_of = {j['child']: n for n, j in self.joints.items()}

    def link_poses(self, q):
        """q: dict joint->radians. Returns dict link->4x4 in base_link (metres)."""
        poses = {'base_link': np.eye(4)}
        def pose(link):
            if link in poses:
                return poses[link]
            jn = self.child_of[link]; j = self.joints[jn]
            T = pose(j['parent']) @ j['T']
            if j['type'] == 'revolute':
                T = T @ axis_rot(j['axis'], q.get(jn, 0.0))
            poses[link] = T
            return T
        for link in list(self.visuals) + ['gripper_frame_link']:
            pose(link)
        return poses

    def mesh(self, q):
        P = self.link_poses(q); parts = []
        for link, items in self.visuals.items():
            for To, m in items:
                c = m.copy(); c.apply_transform(P[link] @ To); parts.append(c)
        out = trimesh.util.concatenate(parts); out.apply_scale(1000.0)
        return out

    def tool(self, q):
        return self.link_poses(q)['gripper_frame_link'][:3, 3] * 1000.0


def deg(*a):
    return {n: np.radians(v) for n, v in zip(JOINTS, a)}


if __name__ == '__main__':
    arm = Arm()
    for name, q in {'ZERO': deg(0, 0, 0, 0, 0, 0), 'HOME': deg(0, -25, 35, 0, 0, 20)}.items():
        b = arm.mesh(q).bounds
        print(name, 'bbox mm', np.round(b, 1).tolist(), 'tool', np.round(arm.tool(q), 1).tolist())
    # Max tool height within limits (pan/roll irrelevant to height).
    best = (-1e9, None)
    g = np.radians(np.arange(-80, 80.1, 5))
    for a in g:
        for b in g:
            for c in g:
                q = {'shoulder_lift': a, 'elbow_flex': b, 'wrist_flex': c}
                z = arm.tool(q)[2]
                if z > best[0]:
                    best = (z, (np.degrees(a), np.degrees(b), np.degrees(c)))
    print('max tool z above base underside (mm):', round(best[0], 1), 'at lift/elbow/wrist deg', best[1])
