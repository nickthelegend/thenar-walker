"""Add the RoboReach arm poses as configurations of the vendored follower master.

Uses the master's own native pose-driver angle mates (J1..J6_Pose_driver; native
angle = 90 deg + logical joint angle) exactly as thenar-arms set_robot_pose.py
does, in configurations copied from HOME. Then cross-checks every pose against
the URDF forward kinematics (analysis/arm_fk.py): SolidWorks assembly bounding
box vs FK mesh bounding box.
"""
import json
import math
import os
import sys

import numpy as np
import pythoncom
from win32com.client import VARIANT

import swlib
from swlib import cast, sw
import design as D

sys.path.insert(0, os.path.join(D.ROOT, 'analysis'))
from arm_fk import Arm  # noqa: E402

MASTER = os.path.join(D.VENDOR, 'assemblies', 'SO101_Follower_Master.SLDASM')
SW_SET_SPECIFIC = 3   # swSetValue_InSpecificConfigurations
SUPPRESS, UNSUPPRESS, THIS_CONFIG = 0, 1, 1   # swSuppressFeature, swUnSuppressFeature, swThisConfiguration


def open_doc(path, kind=2):
    r = sw().OpenDoc6(path, kind, 1, '', 0, 0)
    m = cast(r[0] if isinstance(r, tuple) else r, 'IModelDoc2')
    if m is None:
        raise RuntimeError('open failed ' + path)
    sw().ActivateDoc3(m.GetTitle(), False, 0, 0)
    return m


def feature(m, name):
    f = cast(cast(m, 'IAssemblyDoc').FeatureByName(name), 'IFeature')
    if f is None:
        # mates live under the MateGroup; walk it
        mg = cast(m.FirstFeature(), 'IFeature')
        while mg is not None:
            if mg.GetTypeName2() == 'MateGroup':
                sub = cast(mg.GetFirstSubFeature(), 'IFeature')
                while sub is not None:
                    if sub.Name == name:
                        return sub
                    sub = cast(sub.GetNextSubFeature(), 'IFeature')
            mg = cast(mg.GetNextFeature(), 'IFeature')
        raise KeyError(name)
    return f


def set_pose(m, config, q):
    m.ShowConfiguration2(config)
    for j in range(6):
        lim = feature(m, f'J{j + 1}_Source_preview_limits')
        drv = feature(m, f'J{j + 1}_Pose_driver')
        if drv.IsSuppressed():
            assert drv.SetSuppression2(UNSUPPRESS, THIS_CONFIG, None)
        if not lim.IsSuppressed():
            assert lim.SetSuppression2(SUPPRESS, THIS_CONFIG, None)
        dim = cast(cast(drv.GetFirstDisplayDimension(), 'IDisplayDimension').GetDimension2(0), 'IDimension')
        err = dim.SetSystemValue3(math.radians(90 + q[j]), SW_SET_SPECIFIC,
                                  VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_BSTR, [config]))
        assert err == 0, (config, j, err)
    m.ForceRebuild3(False)


def asm_box(m):
    lo, hi, _ = swlib.tight_box(cast(m, 'IAssemblyDoc').GetComponents(False))
    return [round(v, 1) for v in lo + hi]


def main():
    m = open_doc(MASTER)
    names = list(m.GetConfigurationNames())
    print('existing configurations:', names)
    arm = Arm()
    report = {}
    for name, q in D.POSES.items():
        if name not in names:
            m.ShowConfiguration2('HOME')
            ok = m.AddConfiguration3(name, 'Thenar Walker RoboReach pose', '', 0)
            assert ok, 'AddConfiguration3 failed ' + name
        set_pose(m, name, q)
        sw_box = asm_box(m)
        fk = arm.mesh({n: math.radians(v) for n, v in zip(
            ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'], q)})
        fk_box = [round(v + (D.ARM_BASE_DROP if i % 3 == 2 else 0), 1) for i, v in enumerate(list(fk.bounds[0]) + list(fk.bounds[1]))]  # CAD scene = URDF base + 2.4 mm in Z
        err = float(np.max(np.abs(np.array(sw_box) - np.array(fk_box))))
        rows = {'deg': q, 'solidworks_box_mm': sw_box, 'urdf_fk_box_mm': fk_box, 'max_box_diff_mm': round(err, 2)}
        mates = [(n, cast(feature(m, n), 'IFeature').GetErrorCode2()) for n in
                 [f'J{j + 1}_Pose_driver' for j in range(6)]]
        rows['mate_errors'] = [n for n, e in mates if e[0] != 0]
        report[name] = rows
        print(name, rows, flush=True)
    # the original configurations must still hold their own poses
    for name, q in {'ZERO': [0] * 6, 'HOME': [0, -25, 35, 0, 0, 20], **D.POSES}.items():
        m.ShowConfiguration2(name); m.ForceRebuild3(False)
        fk = arm.mesh({n: math.radians(v) for n, v in zip(
            ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'], q)})
        fk_box = [v + (D.ARM_BASE_DROP if i % 3 == 2 else 0) for i, v in enumerate(list(fk.bounds[0]) + list(fk.bounds[1]))]
        err = float(np.max(np.abs(np.array(asm_box(m)) - np.array(fk_box))))
        report.setdefault('reopen_check', {})[name] = round(err, 2)
        print('re-check', name, 'max box diff mm', round(err, 2), flush=True)
        assert err < 1.0, name
    m.ShowConfiguration2('STOW')
    assert m.SaveAs3(MASTER, 0, 1) == 0
    os.makedirs(D.EVIDENCE, exist_ok=True)
    json.dump(report, open(os.path.join(D.EVIDENCE, 'arm_pose_crosscheck.json'), 'w'), indent=1)
    sw().CloseDoc(m.GetTitle())


if __name__ == '__main__':
    main()
