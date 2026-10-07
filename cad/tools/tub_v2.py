"""Tub v2 — chassis for the team's 370 plastic-gearbox motors (gearbox Ø37.5 × 20, M8 × 13 collar, Ø6 × 22 shaft),
a 3 mm wooden deck, and engraved THENAR.IO / THENARLABS logos.

Built as meshes (trimesh + manifold) because SolidWorks was unavailable; every fit is checked here with exact
mesh booleans against envelopes of all the parts (motors, wheels, battery, boards, kill switch, ESP32, deck).

  python cad/tools/tub_v2.py      -> cad/stl/RR-01_Chassis_Tub.STL (engraved), RR-03_Battery_Cradle.STL,
                                     RR-04_Battery_Bar.STL, cad/evidence/tub_v2_checks.json

Motor mount: the M8 collar passes through a 13 mm thick wall pad (collar flush with the outer face); the
13 mm nut sits in a Ø26 × 7 pocket on the outside, so nothing sticks out toward the wheel (1 mm gap).
Motor can + yellow cap are not measured yet: the envelope assumes can Ø34 × 22 and cap Ø30 × 8
(total length 50 from the gearbox face); facing motors then keep a 12 mm gap.
"""
import json
import math
import os

import numpy as np
import trimesh

import design as D
import engrave_logos as E

STL = os.path.join(D.CAD, 'stl')
EV = os.path.join(D.CAD, 'evidence')
MAN = 'manifold'

# ------------------------------------------------------------------ v2 numbers (mm, robot frame)
AXLE_X = 77.0                    # wheelbase 154 (was 152): room between the bigger gearboxes and the battery cage
AXLE_Z = D.AXLE_Z                # 70, wheel Ø140
TUB_X, TUB_Y, Z0, FLOOR_T = D.TUB_X, D.TUB_Y, D.FLOOR_Z0, D.FLOOR_T
TOP = 100.0                      # walls 2 mm taller: the 3 mm wooden deck's top stays at 103
WALL_S, WALL_E = D.WALL_SIDE_T, D.WALL_END_T
GB_D, GB_L = 37.5, 20.0          # measured
CAN_D, CAN_L, CAP_D, CAP_L = 34.0, 22.0, 30.0, 8.0     # estimated from the photo (not measured yet)
COLLAR_D, COLLAR_L = 8.0, 13.0   # M8, measured
SHAFT_D, SHAFT_L = 6.0, 22.0     # measured, past the collar
NUT_AF, NUT_T = 13.0, 6.5
FACE_Y = TUB_Y - COLLAR_L        # 56: gearbox face sits on the inside of the 13 mm pad
PAD = (24.0, 92.0)               # pad half-width along X, pad top Z
HOLE_D = 8.6
POCKET_D, POCKET_T = 26.0, 7.0
WIN = (18.0, 4.0)                # floor window half-width (X) and inner |Y| end

CAGE_C = (31.0, 0.0)             # battery now lies along Y, right of the centre line
BATT = (34.0, 106.0, 26.0)       # x, y, z
CAGE_T, CAGE_H = 3.0, 27.0
BAR_Y = 26.5

# floor boards (cx, cy, size_x, size_y, height) — top of the 3 mm posts at z 63
BOARDS = {'BUCK6V': (-28.0, -29.0, 48.0, 66.0, 18.0), 'PCA9685': (-25.0, 18.5, 62.2, 25.4, 11.0), 'BUCK5V': (-40.0, 50.0, 22.0, 17.0, 4.5)}
ESP32 = (-30.0, -30.0, 55.0, 28.0)                     # hung under the deck (unchanged)
KILL = (-9.0, 47.0)                                    # moved: the old spot is above the front-right motor now
WHEEL_W = D.WHEEL_W


def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(bounds=[[x0, y0, z0], [x1, y1, z1]])


def cyl_y(x, z, r, y0, y1, sections=64):
    c = trimesh.creation.cylinder(radius=r, height=abs(y1 - y0), sections=sections)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    c.apply_translation([x, (y0 + y1) / 2, z])
    return c


def cyl_z(x, y, r, z0, z1, sections=48):
    c = trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=sections)
    c.apply_translation([x, y, (z0 + z1) / 2])
    return c


def union(ms):
    return trimesh.boolean.union(ms, engine=MAN)


def diff(a, bs):
    return trimesh.boolean.difference([a] + list(bs), engine=MAN)


def posts_for(cx, cy, sx, sy, inset=3.5):
    return [(cx + a * (sx / 2 - inset), cy + b * (sy / 2 - inset)) for a in (1, -1) for b in (1, -1)]


# ------------------------------------------------------------------ parts
def tub():
    shell = diff(box(-TUB_X, TUB_X, -TUB_Y, TUB_Y, Z0, TOP),
                 [box(-TUB_X + WALL_E, TUB_X - WALL_E, -TUB_Y + WALL_S, TUB_Y - WALL_S, Z0 + FLOOR_T, TOP + 1)])
    add = [cyl_z(x, y, D.BOSS_D / 2, Z0 + FLOOR_T, TOP) for x, y in D.BOSS_XY]
    for sx in (1, -1):
        for sy in (1, -1):
            y_in, y_wall = sy * FACE_Y, sy * (TUB_Y - WALL_S)
            add.append(box(sx * AXLE_X - PAD[0], sx * AXLE_X + PAD[0], min(y_in, y_wall), max(y_in, y_wall), Z0 + FLOOR_T, PAD[1]))
    posts = [p for cx, cy, sx_, sy_, _ in BOARDS.values() for p in posts_for(cx, cy, sx_, sy_)]
    add += [cyl_z(x, y, 3.0, Z0 + FLOOR_T, Z0 + FLOOR_T + D.STANDOFF) for x, y in posts]
    t = union([shell] + add)
    cuts = []
    for sx in (1, -1):
        for sy in (1, -1):
            x = sx * AXLE_X
            cuts.append(cyl_y(x, AXLE_Z, HOLE_D / 2, sy * (FACE_Y - 2), sy * (TUB_Y + 2)))              # M8 collar
            cuts.append(cyl_y(x, AXLE_Z, POCKET_D / 2, sy * (TUB_Y - POCKET_T), sy * (TUB_Y + 2)))      # nut pocket
            y_a, y_b = sy * WIN[1], sy * (FACE_Y + 0.5)
            cuts.append(box(x - WIN[0], x + WIN[0], min(y_a, y_b), max(y_a, y_b), Z0 - 1, Z0 + FLOOR_T + 0.5))   # floor window
    cuts += [cyl_z(x, y, 1.1, Z0 + FLOOR_T - 1, Z0 + FLOOR_T + D.STANDOFF + 1) for x, y in posts]        # M2.5 self-tap pilots
    cuts += [cyl_z(x, y, 2.0, TOP - 8, TOP + 1) for x, y in D.BOSS_XY]                                   # heat-set insert bores
    cuts += [cyl_z(CAGE_C[0] - BATT[0] / 2 - CAGE_T - 3, s * 40.0, 1.7, Z0 - 1, Z0 + FLOOR_T + 1) for s in (1, -1)]   # cage screws
    cuts.append(box(-TUB_X - 1, -TUB_X + WALL_E + 1, -38, -22, 82, 96))                                  # USB cable window
    bx, by, _, _ = D.BOARDS['MDD10A']                    # nuts of the driver screws under the wooden deck, 1 mm from the wall
    for x, y in [(bx + a_ * 39.37, by + 27.94) for a_ in (1, -1)]:
        cuts.append(box(x - 5, x + 5, y - 5, TUB_Y - 2, TOP - 5, TOP + 1))                               # 10 × 5 deep notch, 3 of 5 mm wall
    t = diff(t, cuts)
    logos = [E.logo_solid(*l, D.LOGO_DEPTH) for l in D.LOGOS]
    return diff(t, logos), posts


def cage():
    cx, cy = CAGE_C
    ix, iy = BATT[0] / 2 + 1, BATT[1] / 2 + 1                  # 1 mm clearance all round
    ox, oy = ix + CAGE_T, iy + CAGE_T
    z0, z1 = Z0 + FLOOR_T, Z0 + FLOOR_T + CAGE_H
    walls = diff(box(cx - ox, cx + ox, cy - oy, cy + oy, z0, z1), [box(cx - ix, cx + ix, cy - iy, cy + iy, z0 - 1, z1 + 1)])
    tabs = [box(cx - ox - 6, cx - ox, s * 40 - 6, s * 40 + 6, z0, z0 + 3) for s in (1, -1)]
    bosses = []
    for sgn in (1, -1):                                         # bar bosses on the outside of both long walls
        xa = cx + sgn * ox
        for s in (1, -1):
            bosses.append(box(min(xa, xa + sgn * 3), max(xa, xa + sgn * 3), s * BAR_Y - 5, s * BAR_Y + 5, z1 - 8, z1))
    c = union([walls] + tabs + bosses)
    holes = [cyl_z(cx - ox - 3, s * 40, 1.7, z0 - 1, z0 + 4) for s in (1, -1)]
    holes += [cyl_z(cx + sgn * (ox + 1.5), s * BAR_Y, 1.3, z1 - 8, z1 + 1) for sgn in (1, -1) for s in (1, -1)]
    holes.append(box(cx - 9, cx + 9, cy + iy - 1, cy + oy + 1, z0 + 6, z1 + 1))     # battery lead exit (+Y end)
    return diff(c, holes)


def bar():
    cx = CAGE_C[0]
    ox = BATT[0] / 2 + 1 + CAGE_T
    z0 = Z0 + FLOOR_T + CAGE_H
    b = box(cx - ox - 3, cx + ox + 3, -5, 5, z0, z0 + 3)
    return diff(b, [cyl_z(cx + s * (ox + 1.5), 0, 1.7, z0 - 1, z0 + 4) for s in (1, -1)])


# ------------------------------------------------------------------ envelopes for the fit check
def motor(sx, sy):
    x, f = sx * AXLE_X, sy * FACE_Y
    gb = cyl_y(x, AXLE_Z, GB_D / 2, f, f - sy * GB_L)
    can = cyl_y(x, AXLE_Z, CAN_D / 2, f - sy * GB_L, f - sy * (GB_L + CAN_L))
    cap = cyl_y(x, AXLE_Z, CAP_D / 2, f - sy * (GB_L + CAN_L), f - sy * (GB_L + CAN_L + CAP_L))
    return union([gb, can, cap])


def envelopes():
    env = {}
    for sx, xn in ((1, 'F'), (-1, 'R')):
        for sy, yn in ((1, 'L'), (-1, 'R')):
            env[f'motor_{xn}{yn}'] = motor(sx, sy)
            env[f'nut_{xn}{yn}'] = cyl_y(sx * AXLE_X, AXLE_Z, NUT_AF / math.sqrt(3), sy * (TUB_Y - NUT_T - 0.2), sy * (TUB_Y - 0.2), sections=6)
            env[f'wheel_{xn}{yn}'] = cyl_y(sx * AXLE_X, AXLE_Z, D.WHEEL_D / 2, sy * D.WHEEL_INNER, sy * (D.WHEEL_INNER + WHEEL_W))
    cx, cy = CAGE_C
    env['battery'] = box(cx - BATT[0] / 2, cx + BATT[0] / 2, cy - BATT[1] / 2, cy + BATT[1] / 2, Z0 + FLOOR_T, Z0 + FLOOR_T + BATT[2])
    for k, (bx, by, sx_, sy_, h) in BOARDS.items():
        z = Z0 + FLOOR_T + D.STANDOFF
        env[k] = box(bx - sx_ / 2, bx + sx_ / 2, by - sy_ / 2, by + sy_ / 2, z, z + h)
    ex, ey, el, ew = ESP32
    env['ESP32'] = box(ex - el / 2, ex + el / 2, ey - ew / 2, ey + ew / 2, TOP - 10 - 6.8, TOP - 10 + 1.6)      # under-deck components
    kx, ky = KILL
    env['kill_switch_body'] = box(kx - 10.8, kx + 10.8, ky - 14.8, ky + 14.8, TOP + 3 - 32, TOP)
    bx, by, _, _ = D.BOARDS['MDD10A']
    env['MDD10A_nuts'] = union([cyl_z(x, y, 3.5, TOP - 3, TOP) for x, y in [(bx + a * 39.37, by + b * 27.94) for a in (1, -1) for b in (1, -1)]])
    return env


def main():
    t, posts = tub()
    c, b = cage(), bar()
    bars = [b.copy(), b.copy()]
    bars[0].apply_translation([0, BAR_Y, 0])
    bars[1].apply_translation([0, -BAR_Y, 0])
    parts = {'tub': t, 'cage': c, 'bar_L': bars[0], 'bar_R': bars[1]}
    env = envelopes()
    allm = {**parts, **env}
    report = {'checks': [], 'assumed_motor': {'gearbox': [GB_D, GB_L], 'can': [CAN_D, CAN_L], 'cap': [CAP_D, CAP_L],
                                               'total_length_from_face': GB_L + CAN_L + CAP_L}}

    def chk(name, ok, detail):
        report['checks'].append({'check': name, 'pass': bool(ok), 'detail': detail})
        print(('PASS ' if ok else 'FAIL ') + name + ' — ' + detail)

    # 1. collisions (every pair except the intended contacts)
    ok_pairs = {('battery', 'cage')}
    names = list(allm)
    hits = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b_ = names[i], names[j]
            if (a, b_) in ok_pairs or (b_, a) in ok_pairs:
                continue
            if a.startswith('wheel') and b_.startswith('wheel'):
                continue
            ia, ib = allm[a].bounds, allm[b_].bounds
            if np.any(ia[1] < ib[0]) or np.any(ib[1] < ia[0]):
                continue
            inter = trimesh.boolean.intersection([allm[a], allm[b_]], engine=MAN)
            v = 0.0 if inter.is_empty else abs(inter.volume)
            if v > 0.01:
                hits.append((a, b_, round(v, 2)))
    chk('No part touches another (motors, nuts, wheels, battery, boards, ESP32, kill switch, cage, tub)', not hits,
        'clear' if not hits else str(hits))
    # 2. clearances that matter
    gap_center = 2 * (FACE_Y - (GB_L + CAN_L + CAP_L))
    chk('Facing left/right motors keep a gap in the middle', gap_center >= 6, f'{gap_center:.1f} mm (motor total length assumed {GB_L + CAN_L + CAP_L:.0f})')
    gap_cage = (AXLE_X - GB_D / 2) - (CAGE_C[0] + BATT[0] / 2 + 1 + CAGE_T + 3)
    chk('Gearbox to battery-cage boss gap', gap_cage >= 1.5, f'{gap_cage:.2f} mm')
    lowest = min(m.bounds[0][2] for k, m in allm.items() if not k.startswith('wheel'))
    chk('Ground clearance ≥ 50 mm (lowest point except wheels)', lowest >= 50, f'{lowest:.2f} mm (gearbox bottom)')
    xs = [m.bounds[:, 0] for k, m in allm.items()]
    ys = [m.bounds[:, 1] for k, m in allm.items()]
    L = max(max(x[1] for x in xs), D.DECK_X) - min(min(x[0] for x in xs), -D.DECK_X)
    W = max(y[1] for y in ys) - min(y[0] for y in ys)
    chk('Fits 300 × 200 (length × width, wheels included)', L <= 300 and W <= 200, f'{L:.1f} × {W:.1f} mm')
    shaft_in = (TUB_Y + SHAFT_L) - D.WHEEL_INNER
    chk('Shaft reaches into the wheel hub', shaft_in >= 15, f'{shaft_in:.0f} mm of shaft inside the hub (bore 28 deep)')
    gap_wheel = D.WHEEL_INNER - TUB_Y
    chk('Wheel clears the tub wall and the nut', gap_wheel >= 0.8, f'{gap_wheel:.1f} mm (nut sits inside a 7 mm pocket)')
    for k, m in parts.items():
        chk(f'{k} mesh is watertight', m.is_watertight, f'{len(m.faces)} faces, {m.volume / 1000:.1f} cm³')
    # export (STL frames: robot frame, same as v1)
    t.export(os.path.join(STL, 'RR-01_Chassis_Tub.STL'))
    c.export(os.path.join(STL, 'RR-03_Battery_Cradle.STL'))
    bars[0].export(os.path.join(STL, 'RR-04_Battery_Bar.STL'))
    os.makedirs(EV, exist_ok=True)
    json.dump(report, open(os.path.join(EV, 'tub_v2_checks.json'), 'w'), indent=1)
    print('exported RR-01 / RR-03 / RR-04 STLs;', sum(c_['pass'] for c_ in report['checks']), '/', len(report['checks']), 'checks pass')


if __name__ == '__main__':
    main()
