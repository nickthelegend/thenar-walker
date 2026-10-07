"""Arena assembly + terrain verification.

  python build_arena_asm.py arena    -> cad/arena/RoboReach_Arena.SLDASM (game objects placed, robot at start)
  python build_arena_asm.py terrain  -> robot on the real ramp / bridge / speed breakers of the arena board;
                                        SolidWorks interference robot-vs-board at each solved pose
                                        -> cad/evidence/terrain_check.json + renders
"""
import json
import math
import os
import sys

import numpy as np
import pythoncom
from win32com.client import VARIANT

import swlib
from swlib import cast, sw, MM
import design as D
import terrain as T
import build_arena as BA
import build_robot as BR

ARENA_ASM = os.path.join(D.ARENA, 'RoboReach_Arena.SLDASM')
TERRAIN_ASM = os.path.join(D.ARENA, 'Terrain_Check.SLDASM')
RZ90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])     # robot +X -> arena +Y (lane A direction)


def apart(n):
    return os.path.join(D.ARENA, n + '.SLDPRT')


def open_silent(path, kind):
    r = sw().OpenDoc6(path, kind, 1, '', 0, 0)
    return cast(r[0] if isinstance(r, tuple) else r, 'IModelDoc2')


def new_asm():
    m = cast(sw().NewDocument(swlib.template('asm'), 0, 0, 0), 'IModelDoc2')
    sw().ActivateDoc3(m.GetTitle(), False, 0, 0)
    return m, cast(m, 'IAssemblyDoc')


def add(asm, path, R=np.eye(3), t=(0, 0, 0), config=None):
    c = cast(asm.AddComponent5(path, 0, '', False, '', 0, 0, 0), 'IComponent2')
    assert c is not None, path
    c.Transform2 = BR.tf(R, t)
    if config:
        c.ReferencedConfiguration = config
    return c


def objects():
    rz = lambda d: BR.rot_z(d)
    O = [('RA-10_Cube_50', (-1200, -925, 1), 0),                                        # pickup
         ('RA-10_Cube_50', (-650, 1200, 1), 0),                                         # longshot
         ('RA-14_Marker', (650, 1250, 1), 0),
         ('RA-11_Cube_60', (BA.POLE[0], BA.POLE[1], 400), 0)]                           # Downy cube on the pole
    O += [('RA-12_Cylinder_60x80', (-1440, y, 0), 0) for y in (1060, 1160, 1260)]
    O += [('RA-13_Cone_R30x100', (-1440, 1380, 0), 0)]
    hx, hy = BA.ZONES['Hanoi_1'][:2]
    O += [('RA-10_Cube_50', (hx, hy, 1 + 50 * k), 0) for k in range(3)]                 # Tower of Hanoi stack
    ex, ey = BA.ESCAPE_C
    O += [('RA-12_Cylinder_60x80', (ex - 120, ey + 60, 0), 0), ('RA-10_Cube_50', (ex + 40, ey - 140, 0), 0),
          ('RA-16_Escape_Triangle', (ex + 130, ey + 110, 0), 0), ('RA-17_Escape_Hexagon', (ex - 60, ey - 60, 0), 0),
          ('RA-10_Cube_50', (ex + 100, ey - 10, 0), 30), ('RA-12_Cylinder_60x80', (ex + 20, ey + 150, 0), 0),
          ('RA-10_Cube_50', (ex - 150, ey - 150, 0), 15)]
    (ax, ay), _ = BA.BUZZ
    O += [('RA-15_Buzz_Loop', (ax + 60, ay, 290 - 150), 90)]                             # ring threaded on the wire
    return [(n, rz(a), p) for n, p, a in O]


def build_arena():
    for n in {o[0] for o in objects()} | {'RA-01_Arena_Board'}:
        open_silent(apart(n), 1)
    open_silent(BR.ASM_PATH, 2)
    m, asm = new_asm()
    add(asm, apart('RA-01_Arena_Board'))
    for n, R, p in objects():
        add(asm, apart(n), R, p)
    sx, sy = BA.ZONES['Start'][:2]
    bot = add(asm, BR.ASM_PATH, RZ90, (sx, sy, 1), 'STOW')
    m.ClearSelection2(True)
    for c in asm.GetComponents(True):
        cast(c, 'IComponent2').Select4(True, None, False)
    asm.FixComponent(); m.ClearSelection2(True)
    m.ForceRebuild3(False)
    swlib.hide_refs(m)
    assert m.SaveAs3(ARENA_ASM, 0, 1) == 0
    os.makedirs(os.path.join(D.RENDERS, 'arena'), exist_ok=True)
    for name, eye in (('arena_iso', (-1.0, -1.0, 0.9)), ('arena_iso_2', (1.0, -1.2, 1.1)), ('arena_top', (0.0001, 0, 1))):
        swlib.zup_view(m, eye); m.ViewZoomtofit2()
        m.SaveAs3(os.path.join(D.RENDERS, 'arena', name + '.png'), 0, 3)
    # close-up of the robot in the start zone
    swlib.zup_view(m, (1.0, -1.25, 0.9))
    swlib.zoom_to(m, bot, 0.55)
    m.SaveAs3(os.path.join(D.RENDERS, 'arena', 'robot_at_start.png'), 0, 3)
    print('saved', ARENA_ASM)


def interferences(asm):
    mgr = asm.InterferenceDetectionManager
    mgr.TreatCoincidenceAsInterference = False
    mgr.TreatSubAssembliesAsComponents = False
    mgr.IncludeMultibodyPartInterferences = False
    mgr.IgnoreHiddenBodies = True
    out = []
    for it in mgr.GetInterferences() or []:
        it = cast(it, 'IInterference')
        out.append({'components': sorted(cast(c, 'IComponent2').Name2 for c in it.Components), 'volume_mm3': round(it.Volume * 1e9, 3)})
    mgr.Done()
    return out


def terrain_check():
    open_silent(apart('RA-01_Arena_Board'), 1)
    open_silent(BR.ASM_PATH, 2)
    m, asm = new_asm()
    board = add(asm, apart('RA-01_Arena_Board'))
    robot = add(asm, BR.ASM_PATH, RZ90, (0, 0, 0), 'DRIVE_ONLY')
    lane_x = BA.RAMP_X0 + BA.RAMP_W / 2
    rep = {}
    for case, xr in T.CASES.items():
        R, t, pitch = T.robot_transform(xr)
        Ra = RZ90 @ R
        ta = RZ90 @ t + np.array([lane_x, BA.RAMP_Y0, 0])
        robot.ReferencedConfiguration = 'DRIVE_ONLY'
        robot.Transform2 = BR.tf(Ra, ta)
        m.ForceRebuild3(False)
        hits = [i for i in interferences(asm) if any('RA-01_Arena_Board' in n for n in i['components'])]
        tyre = [i for i in hits if any('RR-06_Wheel_Tyre' in n for n in i['components'])]
        other = [i for i in hits if i not in tyre]
        lo, hi, per = swlib.tight_box([cast(c, 'IComponent2') for c in asm.GetComponents(False)
                                       if 'RA-01' not in cast(c, 'IComponent2').Name2])
        rep[case] = {'rear_axle_x_on_profile_mm': round(xr, 1), 'pitch_deg': round(pitch, 2),
                     'chassis_terrain_interference': other, 'tyre_contact_overlaps': tyre,
                     'pass': not other}
        print(f'{case:22s} pitch {pitch:6.2f}  chassis hits {len(other)}  tyre contacts {len(tyre)} '
              f'(max {max([i["volume_mm3"] for i in tyre], default=0):.2f} mm3)', flush=True)
        if case in ('on_ramp', 'ramp_crest', 'bridge_exit', 'breaker_under_belly', 'front_on_breaker_1'):
            robot.ReferencedConfiguration = 'HOME'
            m.ForceRebuild3(False)
            c = ta
            swlib.hide_refs(m)
            swlib.zup_view(m, (-1.0, 0.25, 0.35))
            swlib.zoom_to(m, robot, 0.5)
            m.SaveAs3(os.path.join(D.RENDERS, 'arena', f'terrain_{case}.png'), 0, 3)
    rep['_summary'] = {'all_pass': all(v['pass'] for k, v in rep.items() if not k.startswith('_'))}
    json.dump(rep, open(os.path.join(D.EVIDENCE, 'terrain_check.json'), 'w'), indent=1)
    print('ALL PASS' if rep['_summary']['all_pass'] else 'FAILURES PRESENT')
    assert m.SaveAs3(TERRAIN_ASM, 0, 1) == 0


if __name__ == '__main__':
    what = sys.argv[1:] or ['arena', 'terrain']
    if 'arena' in what:
        build_arena()
    if 'terrain' in what:
        terrain_check()
