"""Verify Thenar Walker against the RoboReach rules, in SolidWorks.

For every configuration (STOW, HOME, REACH_450, PICK_FLOOR):
  * interference detection through all sub-assembly levels (exact B-rep)
  * exact bounding box from body extreme points (start box rule, STOW)
  * ground clearance = lowest point of every non-wheel component
Plus: grasp-centre height (URDF FK, cross-checked to SolidWorks at 0.04 mm in
prepare_arm.py), mass / centre of gravity from SolidWorks volumes + datasheet
masses, and static tip-over margins on flat ground and on the 15 deg ramp.
Writes cad/evidence/verification.json and prints a PASS/FAIL table.
"""
import json
import math
import os
import sys

import numpy as np

import swlib
from swlib import cast, sw
import design as D
import build_robot as BR
import render_robot as RR

sys.path.insert(0, os.path.join(D.ROOT, 'analysis'))
from arm_fk import Arm, JOINTS  # noqa: E402

WHEEL_KEYS = ('RR-05_Wheel_Hub', 'RR-06_Wheel_Tyre', 'RR-07_Front_Wheel_PETG')


def leaf_components(asm):
    return [cast(c, 'IComponent2') for c in asm.GetComponents(False)]


def interferences(m, asm):
    mgr = asm.InterferenceDetectionManager
    mgr.TreatCoincidenceAsInterference = False
    mgr.TreatSubAssembliesAsComponents = False
    mgr.IncludeMultibodyPartInterferences = False
    mgr.IgnoreHiddenBodies = True
    out = []
    for it in mgr.GetInterferences() or []:
        it = cast(it, 'IInterference')
        names = sorted(cast(c, 'IComponent2').Name2 for c in it.Components)
        out.append({'components': names, 'volume_mm3': round(it.Volume * 1e9, 3)})
    mgr.Done()
    return out


def intended(i):
    """Servo spline <-> its own horn: modelled as overlap in the thenar-arms CAD (press fit on the spline)."""
    n = [x.split('/')[-1] for x in i['components']]
    s = [x for x in n if x.startswith('MG996R_Nominal_R3-')]
    h = [x for x in n if x.startswith('Metal_Horn_D20_PCD14_R3-')]
    return len(s) == 1 and len(h) == 1 and s[0].split('-')[-1] == h[0].split('-')[-1] and i['volume_mm3'] < 10


def body_mass_props(c):
    """(volume mm3, centroid world mm) of a component's solid bodies."""
    M = swlib.comp_matrix(c)
    vol, mom = 0.0, np.zeros(3)
    for b in swlib.comp_bodies(c):
        mp = b.GetMassProperties(1.0)
        if not mp:
            continue
        cx, cy, cz, v = mp[0], mp[1], mp[2], mp[3]
        pw = M[:3, :3] @ (np.array([cx, cy, cz]) * 1000) + M[:3, 3]
        vol += v * 1e9; mom += pw * v * 1e9
    return vol, (mom / vol if vol > 0 else None)


def real_mass_g(name, vol_mm3):
    n = name.split('/')[-1]
    table = [('P-01_Motor', D.MASS['motor_jga25']), ('P-02_LiPo', D.MASS['battery_3s2200']), ('P-03_ESP32', D.MASS['esp32']),
             ('P-04_PCA9685', D.MASS['pca9685']), ('P-05_Cytron', D.MASS['mdd10a']), ('P-06_Buck', D.MASS['buck_6v']),
             ('P-07_Buck', D.MASS['buck_5v']), ('P-08_Kill', D.MASS['kill_switch']), ('MG996R_Nominal', D.MASS['mg996r']),
             ('Metal_Horn', D.MASS['horn']), ('RR-06_Wheel_Tyre', None)]
    for key, g in table:
        if key in n:
            if g is None:   # TPU tyre: 1.21 g/cc at the tyre print fill
                return vol_mm3 / 1000 * 1.21 * 0.6
            return float(g)
    return vol_mm3 / 1000 * D.PETG_G_CC * D.PRINT_FILL   # printed PETG


def support_polygon():
    return [(sx * D.AXLE_X, sy * D.WHEEL_YC) for sx in (1, -1) for sy in (1, -1)]


def tip_margins(cog):
    """Static margins (mm) from COG ground projection to each support edge, flat + on 15 deg ramp."""
    x, y, z = cog
    th = math.radians(D.RULES['ramp_deg'])
    flat = {'front': D.AXLE_X - x, 'rear': D.AXLE_X + x, 'left': D.WHEEL_YC - y, 'right': D.WHEEL_YC + y}
    # on a slope the gravity line shifts by z*tan(th) towards the downhill axle (z measured from ground contact)
    shift = z * math.tan(th)
    ramp = {'climb_15deg_rear_axle': D.AXLE_X + x - shift, 'descend_15deg_front_axle': D.AXLE_X - x - shift,
            'traverse_15deg_side': D.WHEEL_YC - abs(y) - shift}
    return {k: round(float(v), 1) for k, v in {**flat, **ramp}.items()}


def main():
    m = RR.open_asm()
    asm = cast(m, 'IAssemblyDoc')
    arm = Arm()
    rep = {'rules': D.RULES, 'configs': {}}
    for cfg, q in D.POSES.items():
        m.ShowConfiguration2(cfg)
        m.ForceRebuild3(False)
        comps = leaf_components(asm)
        lo, hi, per = swlib.tight_box(comps)
        nonwheel_min_z = min(v[0][2] for k, v in per.items() if not any(w in k for w in WHEEL_KEYS))
        lowest = min(((v[0][2], k) for k, v in per.items() if not any(w in k for w in WHEEL_KEYS)))
        inter = interferences(m, asm)
        # mass + COG
        tot, mom, rows = 0.0, np.zeros(3), []
        for c in comps:
            vol, cen = body_mass_props(c)
            if cen is None:
                continue
            g = real_mass_g(c.Name2, vol)
            tot += g; mom += g * cen
            rows.append({'component': c.Name2, 'mass_g': round(g, 1), 'centroid_mm': cen.round(1).tolist()})
        extra = D.MASS['wiring_misc'] + D.MASS['fasteners']
        cog_extra = np.array([0, 0, 100.0])
        tot += extra; mom += extra * cog_extra
        cog = mom / tot
        qq = {n: math.radians(v) for n, v in zip(JOINTS, q)}
        Tg = arm.link_poses(qq)['gripper_frame_link']
        # grasp centre of a 50 mm object: the URDF tool point is on the fixed finger
        tool = Tg[:3, 3] * 1000 - D.GRASP_OFFSET_50 * Tg[:3, 0] + np.array([D.ARM_X, 0, D.DECK_TOP + D.ARM_BASE_DROP])
        size = (np.array(hi) - np.array(lo)).round(1).tolist()
        r = {'joint_deg': q, 'bbox_lo_mm': lo, 'bbox_hi_mm': hi, 'size_lwh_mm': size,
             'lowest_non_wheel': {'z_mm': round(lowest[0], 2), 'component': lowest[1]},
             'interferences': inter, 'mass_total_g': round(tot, 0), 'cog_mm': cog.round(1).tolist(),
             'tip_margins_mm': tip_margins(cog), 'grasp_centre_mm': tool.round(1).tolist(), 'mass_rows': rows}
        rep['configs'][cfg] = r
        print(f'{cfg:11s} size {size}  lowest {lowest[0]:.1f} ({lowest[1]})  interferences {len(inter)}  '
              f'mass {tot:.0f} g  COG {cog.round(1).tolist()}  grasp {tool.round(1).tolist()}', flush=True)
    m.ShowConfiguration2('STOW')
    # ---------------------------------------------------------------- rule table
    s = rep['configs']
    L, W, H = D.RULES['start_box_lwh']
    checks = []
    def chk(name, ok, detail):
        checks.append({'check': name, 'pass': bool(ok), 'detail': detail})
    st = s['STOW']['size_lwh_mm']
    chk('Start box 300 x 200 x 300 (STOW)', st[0] <= L and st[1] <= W and st[2] <= H and s['STOW']['bbox_lo_mm'][2] >= -0.01,
        f'{st[0]} x {st[1]} x {st[2]} mm')
    for c in ('STOW', 'HOME', 'REACH_450'):
        z = s[c]['lowest_non_wheel']['z_mm']
        chk(f'Ground clearance >= 50 mm ({c})', z >= D.RULES['min_ground_clearance'], f"{z} mm at {s[c]['lowest_non_wheel']['component']}")
    g = s['REACH_450']['grasp_centre_mm']
    chk('Grasp object centred at 450 mm (REACH_450)', abs(g[2] - 450) <= 5, f'50 mm object centre z = {g[2]} mm, {g[0] - D.TUB_X:.0f} mm ahead of the chassis')
    gp = s['PICK_FLOOR']['grasp_centre_mm']
    chk('Floor pick: 50 mm cube on the ground ahead of the wheels', abs(gp[2] - 25) <= 5 and gp[0] > D.AXLE_X + D.WHEEL_D / 2 + 25,
        f'grasp centre {gp} (cube centre z = 25)')
    for c, r in s.items():
        robot_hits = [i for i in r['interferences'] if i['volume_mm3'] > 0.001 and not intended(i)]
        n_int = sum(1 for i in r['interferences'] if intended(i))
        chk(f'No interference ({c})', not robot_hits,
            (json.dumps(robot_hits)[:600] if robot_hits else 'clear') + f' (+{n_int} intended servo-spline/horn overlaps from the arm CAD)')
    for c in ('STOW', 'HOME', 'REACH_450', 'PICK_FLOOR'):
        mg = s[c]['tip_margins_mm']
        worst = min(mg.values())
        chk(f'Static stability ({c}, flat + 15 deg ramp)', worst > 10, f'worst margin {worst} mm: {mg}')
    rep['checks'] = checks
    os.makedirs(D.EVIDENCE, exist_ok=True)
    json.dump(rep, open(os.path.join(D.EVIDENCE, 'verification.json'), 'w'), indent=1)
    print()
    for c in checks:
        print(('PASS ' if c['pass'] else 'FAIL ') + c['check'] + ' — ' + c['detail'])
    return rep


if __name__ == '__main__':
    main()
