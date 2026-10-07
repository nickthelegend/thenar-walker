"""Assemble Thenar Walker (mobile base + Thenar R3 follower arm) in SolidWorks.

Output: cad/assemblies/Thenar_Walker.SLDASM with configurations STOW (start box),
HOME, REACH_450 and PICK_FLOOR. Each top-level configuration references the arm
configuration of the same name (created by prepare_arm.py).
Components are placed from design.py coordinates and fixed.
"""
import os

import numpy as np
import pythoncom
from win32com.client import VARIANT

import swlib
from swlib import cast, sw, MM
import design as D
import build_parts as BP
import prepare_arm as PA

ASM_PATH = os.path.join(D.ASM, 'Thenar_Walker.SLDASM')
ARM = PA.MASTER


def part(name):
    return os.path.join(D.PARTS, name + '.SLDPRT')


def tf(R=np.eye(3), t=(0, 0, 0)):
    """R columns = images of local X, Y, Z in robot frame; t in mm."""
    R = np.asarray(R, float)
    data = list(R[:, 0]) + list(R[:, 1]) + list(R[:, 2]) + [t[0] * MM, t[1] * MM, t[2] * MM, 1.0, 0, 0, 0]
    mu = cast(sw().GetMathUtility(), 'IMathUtility')
    return mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, data))


def axis_y(sign):
    """Local +Z -> robot sign*Y, local +X -> robot +X (right-handed)."""
    z = np.array([0, sign, 0.0]); x = np.array([1.0, 0, 0]); y = np.cross(z, x)
    return np.column_stack([x, y, z])


def rot_z(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])


FLIP_X = np.diag([1.0, -1.0, -1.0])      # board hung upside-down under the deck


def placements():
    """(key, part path, (R, t), group)"""
    I = np.eye(3)
    P = [('tub', part('RR-01_Chassis_Tub'), (I, (0, 0, 0)), 'chassis'),
         ('deck', part('RR-02_Deck_Plate'), (I, (0, 0, 0)), 'chassis'),
         ('cradle', part('RR-03_Battery_Cradle'), (I, (0, 0, 0)), 'chassis'),
         ('battery', part('P-02_LiPo_3S_2200mAh'), (I, (D.BATT_C[0], D.BATT_C[1], D.FLOOR_Z0 + D.FLOOR_T)), 'power'),
         ('kill', part('P-08_Kill_Switch_KCD4_30A'), (I, (D.KILL_XY[0], D.KILL_XY[1], D.DECK_TOP)), 'power')]
    for s in (1, -1):
        P.append((f'bar_{s}', part('RR-04_Battery_Bar'), (I, (D.BATT_C[0] + s * D.BATT_L / 4, 0, 0)), 'chassis'))
    for sx, xn in ((1, 'F'), (-1, 'R')):
        for sy, yn in ((1, 'L'), (-1, 'R')):
            P.append((f'motor_{xn}{yn}', part('P-01_Motor_JGA25-370_12V'), (axis_y(sy), (sx * D.AXLE_X, sy * D.MOTOR_FACE_Y, D.AXLE_Z)), 'drive'))
            wpos = (axis_y(sy), (sx * D.AXLE_X, sy * D.WHEEL_INNER, D.AXLE_Z))
            if sx > 0:      # front: one-piece hard PETG wheel (turning)
                P.append((f'wheel_{xn}{yn}', part('RR-07_Front_Wheel_PETG'), wpos, 'wheel'))
            else:           # rear: PETG hub + TPU tyre (traction over the speed breakers)
                P.append((f'hub_{xn}{yn}', part('RR-05_Wheel_Hub'), wpos, 'wheel'))
                P.append((f'tyre_{xn}{yn}', part('RR-06_Wheel_Tyre'), wpos, 'wheel'))
    names = {'ESP32': 'P-03_ESP32_DevKitC', 'PCA9685': 'P-04_PCA9685_16ch', 'MDD10A': 'P-05_Cytron_MDD10A',
             'BUCK6V': 'P-06_Buck_6V_10A', 'BUCK5V': 'P-07_Buck_5V_MP1584'}
    for k, (cx, cy, rot, mount) in D.BOARDS.items():
        if mount == 'floor':
            R, z = rot_z(rot), D.FLOOR_Z0 + D.FLOOR_T + D.STANDOFF
        elif mount == 'deck_top':
            R, z = rot_z(rot), D.DECK_TOP + D.DECK_BOSS_H
        else:
            R, z = rot_z(rot) @ FLIP_X, D.DECK_Z0 - BP.ESP32_POST
        P.append((k.lower(), part(names[k]), (R, (cx, cy, z)), 'electronics'))
    return P


def open_silent(path, kind):
    r = sw().OpenDoc6(path, kind, 1, '', 0, 0)
    return cast(r[0] if isinstance(r, tuple) else r, 'IModelDoc2')


def build():
    s = sw()
    for _, path, _, _ in placements():
        open_silent(path, 1)
    arm_doc = open_silent(ARM, 2)
    m = cast(s.NewDocument(swlib.template('asm'), 0, 0, 0), 'IModelDoc2')
    asm = cast(m, 'IAssemblyDoc')
    s.ActivateDoc3(m.GetTitle(), False, 0, 0)
    comps = {}
    for key, path, (R, t), grp in placements():
        c = cast(asm.AddComponent5(path, 0, '', False, '', 0, 0, 0), 'IComponent2')
        assert c is not None, path
        c.Transform2 = tf(R, t)
        comps[key] = c
    arm = cast(asm.AddComponent5(ARM, 0, '', False, '', 0, 0, 0), 'IComponent2')
    assert arm is not None
    arm.Transform2 = tf(np.eye(3), (D.ARM_X, 0, D.DECK_TOP))
    comps['arm'] = arm
    m.ClearSelection2(True)
    for c in comps.values():
        c.Select4(True, None, False)
    asm.FixComponent()
    m.ClearSelection2(True)
    m.ForceRebuild3(False)
    assert m.SaveAs3(ASM_PATH, 0, 1) == 0
    # configurations: one per arm pose
    arm_name = arm.Name2
    for name in D.POSES:
        if name not in list(m.GetConfigurationNames()):
            assert m.AddConfiguration3(name, 'Arm pose ' + name, '', 0), name
        m.ShowConfiguration2(name)
        a = [cast(c, 'IComponent2') for c in asm.GetComponents(True) if cast(c, 'IComponent2').Name2 == arm_name][0]
        a.ReferencedConfiguration = name
        m.ForceRebuild3(False)
    # DRIVE_ONLY: arm suppressed, for fast chassis-vs-terrain interference checks
    if 'DRIVE_ONLY' not in list(m.GetConfigurationNames()):
        assert m.AddConfiguration3('DRIVE_ONLY', 'Chassis + drive only (arm suppressed)', '', 0)
    m.ShowConfiguration2('DRIVE_ONLY')
    a = [cast(c, 'IComponent2') for c in asm.GetComponents(True) if cast(c, 'IComponent2').Name2 == arm_name][0]
    a.SetSuppression2(0)   # swComponentSuppressed
    m.ForceRebuild3(False)
    m.ShowConfiguration2('STOW')
    m.ForceRebuild3(False)
    swlib.hide_refs(m)
    assert m.SaveAs3(ASM_PATH, 0, 1) == 0
    print('saved', ASM_PATH, 'configs', list(m.GetConfigurationNames()))
    return m


if __name__ == '__main__':
    m = build()
