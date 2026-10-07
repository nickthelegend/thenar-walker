"""Build every Thenar Walker part as a native SolidWorks part (sketch + extrude/cut/revolve).

  python build_parts.py            # all parts
  python build_parts.py RR-01 P-03 # only these

Printed parts (RR-xx) also export STL to cad/stl/ for slicing.
Purchased parts (P-xx) are nominal envelopes for fit/clearance — verify on your units.
All coordinates come from design.py (robot frame for chassis parts; local frames noted).
Only documents this script creates are closed; other open SolidWorks work is untouched.
"""
import math
import os
import sys

import swlib
from swlib import Part, sw
import design as D

PNG = os.path.join(D.RENDERS, 'parts')


# ------------------------------------------------------------------ model-frame helpers
def zrects(p, rects, z0, z1, name, cut=False):
    """Rectangles (x0,x1,y0,y1) on plane Z=z0, extruded/cut to z1 (> z0)."""
    p.sketch(p.plane('Front', z0))
    for x0, x1, y0, y1 in rects:
        p.rect(x0, y0, x1, y1)
    return p.cut(z1 - z0, name) if cut else p.extrude(z1 - z0, name)


def zcircles(p, circles, z0, z1, name, cut=False):
    p.sketch(p.plane('Front', z0))
    for x, y, r in circles:
        p.circle(x, y, r)
    return p.cut(z1 - z0, name) if cut else p.extrude(z1 - z0, name)


def zpoly(p, pts, z0, z1, name, cut=False):
    p.sketch(p.plane('Front', z0))
    p.poly(pts)
    return p.cut(z1 - z0, name) if cut else p.extrude(z1 - z0, name)


def ycircles(p, circles, y0, y1, name, cut=False):
    """Circles (x,z,r) on plane Y=y0, extruded/cut along +Y to y1."""
    p.sketch(p.plane('Top', y0))
    for x, z, r in circles:
        p.circle(x, -z, r)
    return p.cut(y1 - y0, name) if cut else p.extrude(y1 - y0, name)


def yholes_through(p, circles, name):
    p.sketch('Top Plane')
    for x, z, r in circles:
        p.circle(x, -z, r)
    return p.cut(1, name, both=True, through=True)


def xholes_through(p, circles, name):
    """Circles (y,z,r) cut through everything along X."""
    p.sketch('Right Plane')
    for y, z, r in circles:
        p.circle(-z, y, r)
    return p.cut(1, name, both=True, through=True)


def xcircles(p, circles, x0, x1, name, cut=False):
    p.sketch(p.plane('Right', x0))
    for y, z, r in circles:
        p.circle(-z, y, r)
    return p.cut(x1 - x0, name) if cut else p.extrude(x1 - x0, name)


def mirror4(x, y):
    return [(sx * x, sy * y) for sx in (1, -1) for sy in (1, -1)]


def finish(p, rgb, printed, desc):
    p.color(*rgb)
    try:
        cm = p.m.Extension.CustomPropertyManager('')
        cm.Add3('Description', 30, desc, 2)
        cm.Add3('Make', 30, 'Print' if printed else 'Buy (envelope)', 2)
    except Exception as e:
        print('  (custom properties skipped:', e, ')')
    path = p.save(stl=printed)
    os.makedirs(PNG, exist_ok=True)
    p.snapshot(os.path.join(PNG, p.name + '.png'))
    print(f'{p.name:32s} bbox {p.bbox()}  -> {path}', flush=True)
    p.close()


PETG = (0.16, 0.18, 0.21)
PETG_BLUE = (0.10, 0.22, 0.45)
TPU = (0.05, 0.05, 0.05)
PCB_GREEN = (0.05, 0.35, 0.15)
PCB_BLUE = (0.05, 0.20, 0.55)
PCB_RED = (0.60, 0.08, 0.08)
METAL = (0.70, 0.70, 0.72)


# ================================================================== printed parts
def board_corners(name, inset=3.5):
    """Standoff positions (x, y) of a board from design.BOARDS / BOARD_SIZE (rotation about Z)."""
    cx, cy, rot, mount = D.BOARDS[name]
    L, W = D.BOARD_SIZE[name]
    a = math.radians(rot)
    if name == 'ESP32':      # header pins along both long edges: two end posts + cable tie
        local = [(sx * (L / 2 - 3.5), 0) for sx in (1, -1)]
    elif name == 'MDD10A':   # datasheet hole pattern
        hx, hy = D.MDD10A_HOLES
        local = [(sx * hx / 2, sy * hy / 2) for sx in (1, -1) for sy in (1, -1)]
    else:
        local = [(sx * (L / 2 - inset), sy * (W / 2 - inset)) for sx in (1, -1) for sy in (1, -1)]
    return [(cx + x * math.cos(a) - y * math.sin(a), cy + x * math.sin(a) + y * math.cos(a)) for x, y in local]


ESP32_POST = 10.0     # deck-hung ESP32: header pins (8.5 mm) point up into this gap


def cradle_box():
    L, W, t = D.BATT_L + 2, D.BATT_W + 2, D.CRADLE_T
    cx, cy = D.BATT_C
    return cx, cy, L, W, t


def rr01_tub():
    p = Part('RR-01_Chassis_Tub', D.PARTS)
    X, Y = D.TUB_X, D.TUB_Y
    f0, f1, top = D.FLOOR_Z0, D.FLOOR_Z0 + D.FLOOR_T, D.TUB_TOP_Z
    zrects(p, [(-X, X, -Y, Y)], f0, f1, 'Floor')
    p.sketch(p.plane('Front', f1))
    p.rect(-X, -Y, X, Y)
    p.rect(-X + D.WALL_END_T, -Y + D.WALL_SIDE_T, X - D.WALL_END_T, Y - D.WALL_SIDE_T)
    p.extrude(top - f1, 'Walls')
    zcircles(p, [(x, y, D.BOSS_D / 2) for x, y in D.BOSS_XY], f1, top, 'Insert_columns')
    # board standoffs on the floor (M2.5 self-tap pilots)
    posts = [xy for n, v in D.BOARDS.items() if v[3] == 'floor' for xy in board_corners(n)]
    posts += [(cx + sx * (L / 2 - 3.5), cy + sy * (W / 2 - 3.5))      # spare pads (ex-MDD3A), tub unchanged
              for cx, cy, L, W in D.SPARE_FLOOR_PADS.values() for sx in (1, -1) for sy in (1, -1)]
    zcircles(p, [(x, y, 3.0) for x, y in posts], f1, f1 + D.STANDOFF, 'Board_standoffs')
    zcircles(p, [(x, y, 1.1) for x, y in posts], f1 - 1, f1 + D.STANDOFF + 1, 'Standoff_pilots', cut=True)
    # floor windows so the motor cans can sit below the floor top (lowest point stays FLOOR_Z0)
    ax0, ax1, ay0, ay1 = D.MOTOR_WINDOW
    wins = [(sx * ax0, sx * ax1, sy * ay0, sy * ay1) for sx in (1, -1) for sy in (1, -1)]
    wins = [(min(a, b), max(a, b), min(c, d), max(c, d)) for a, b, c, d in wins]
    zrects(p, wins, f0 - 1, f1 + 0.5, 'Motor_floor_windows', cut=True)
    # motor gearbox pockets: U-slot (round top, open through the floor); face seats at |y| = MOTOR_FACE_Y
    inner = D.TUB_Y - D.WALL_SIDE_T
    r = D.POCKET_D / 2
    for y0, y1, nm in ((inner, D.MOTOR_FACE_Y, 'Motor_pocket_L'), (-D.MOTOR_FACE_Y, -inner, 'Motor_pocket_R')):
        p.sketch(p.plane('Top', y0))
        for sx in (1, -1):
            x = sx * D.AXLE_X
            p.arc(x, -D.AXLE_Z, r, 180, 360)                       # sketch v = -z
            p.line(x + r, -D.AXLE_Z, x + r, -(D.FLOOR_Z0 - 5))
            p.line(x + r, -(D.FLOOR_Z0 - 5), x - r, -(D.FLOOR_Z0 - 5))
            p.line(x - r, -(D.FLOOR_Z0 - 5), x - r, -D.AXLE_Z)
        p.cut(y1 - y0, nm)
    holes = []
    for sx in (1, -1):
        holes.append((sx * D.AXLE_X, D.AXLE_Z, 4.0))                      # shaft/boss clearance
        for dx in (-D.MOTOR_HOLE_PITCH / 2, D.MOTOR_HOLE_PITCH / 2):
            holes.append((sx * D.AXLE_X + dx, D.AXLE_Z, 1.7))             # M3 clearance
    yholes_through(p, holes, 'Motor_shaft_and_M3_holes')
    p.sketch(p.plane('Front', top))
    for x, y in D.BOSS_XY:
        p.circle(x, y, 2.0)
    p.cut(8, 'M3_insert_bores', reverse=True)
    # battery cradle screws (tabs at both ends of the cradle)
    cx, cy, L, W, t = cradle_box()
    zcircles(p, [(cx + s * (L / 2 + t + 3), cy, 1.7) for s in (1, -1)], f0 - 1, f1 + 1, 'Cradle_M3_holes', cut=True)
    # USB pigtail grommet in the rear wall
    p.sketch(p.plane('Right', -X - 1))
    p.rect(-96, -38, -82, -22)  # (u=-z, v=y)
    p.cut(D.WALL_END_T + 2, 'USB_grommet')
    # no front vent slots any more: the motor driver sits on the deck, and the front wall carries the
    # THENAR.IO logo. Logos are cut into the exported STL by engrave_logos.py (a few hundred sketch
    # segments of lettering made the SolidWorks cut take far too long).
    finish(p, PETG, True, 'Chassis tub, PETG, 4 walls 30% gyroid; 4 x JGA25-370, all electronics on the floor')


def rr02_deck():
    p = Part('RR-02_Deck_Plate', D.PARTS)
    z0, z1 = D.DECK_Z0, D.DECK_TOP
    zrects(p, [(-D.DECK_X, D.DECK_X, -D.DECK_Y, D.DECK_Y)], z0, z1, 'Plate')
    fx0, fx1, fy0, fy1 = D.ARM_FOOT
    c = 0.5
    ix0, ix1, iy = D.ARM_X + fx0 - c, D.ARM_X + fx1 + c, fy1 + c
    t = D.FENCE_T
    p.sketch(p.plane('Front', z1))
    p.rect(ix0 - t, -iy - t, ix1 + t, iy + t)
    p.rect(ix0, -iy, ix1, iy)
    p.extrude(D.FENCE_H, 'Arm_locating_fence')
    zcircles(p, [(D.ARM_X + x, y, 1.7) for x, y in D.ARM_SLOTS], z0 - 1, z1 + 1, 'Arm_M3_holes', cut=True)
    zcircles(p, [(x, y, 1.7) for x, y in D.BOSS_XY], z0 - 1, z1 + 1, 'Deck_M3_holes', cut=True)
    p.sketch(p.plane('Front', z1))
    for x, y in D.BOSS_XY:
        p.circle(x, y, 3.25)
    p.cut(2.5, 'Deck_counterbores', reverse=True)
    kx, ky = D.KILL_XY
    w, h = D.KILL_CUT
    zrects(p, [(kx - h / 2, kx + h / 2, ky - w / 2, ky + w / 2)], z0 - 1, z1 + 1, 'Kill_switch_cutout', cut=True)
    zrects(p, [(ix1 + t + 2, ix1 + t + 14, -15, 15)], z0 - 1, z1 + 1, 'Servo_cable_slot', cut=True)
    # ESP32 hangs under the deck on two M2.5 x 10 brass standoffs (holes only: the deck prints flat, no supports)
    posts = board_corners('ESP32')
    zcircles(p, [(x, y, 1.4) for x, y in posts], z0 - 1, z1 + 1, 'ESP32_standoff_holes_M2p5', cut=True)
    # Cytron MDD10A on 4 printed bosses (M3 x 10 self-tapping into the 2.6 mm pilots); wires drop through the slot
    posts = board_corners('MDD10A')
    zcircles(p, [(x, y, 3.5) for x, y in posts], z1, z1 + D.DECK_BOSS_H, 'MDD10A_bosses')
    zcircles(p, [(x, y, 1.3) for x, y in posts], z0 - 1, z1 + D.DECK_BOSS_H + 1, 'MDD10A_pilots_M3', cut=True)
    zrects(p, [D.MOTOR_WIRE_SLOT], z0 - 1, z1 + 1, 'Motor_wire_slot', cut=True)
    zrects(p, [(15, 75, -62, -24)], z1 - 0.6, z1 + 1, 'Name_plate_recess', cut=True)
    finish(p, PETG_BLUE, True, 'Deck plate, PETG; arm fence, M3 arm bolts through base vent slots, kill switch, MDD10A bosses')


def rr03_cradle():
    p = Part('RR-03_Battery_Cradle', D.PARTS)
    cx, cy, L, W, t = cradle_box()
    z0 = D.FLOOR_Z0 + D.FLOOR_T
    H = D.CRADLE_H
    p.sketch(p.plane('Front', z0))
    p.rect(cx - L / 2 - t, cy - W / 2 - t, cx + L / 2 + t, cy + W / 2 + t)
    p.rect(cx - L / 2, cy - W / 2, cx + L / 2, cy + W / 2)
    p.extrude(H, 'Walls')
    zrects(p, [(cx + L / 2 + t, cx + L / 2 + t + 6, cy - 6, cy + 6), (cx - L / 2 - t - 6, cx - L / 2 - t, cy - 6, cy + 6)],
           z0, z0 + 3, 'Floor_tabs')
    zcircles(p, [(cx + s * (L / 2 + t + 3), cy, 1.7) for s in (1, -1)], z0 - 1, z0 + 4, 'Tab_holes', cut=True)
    # bosses for the two retaining bars (M3 self-tap) on the long walls
    bars = [cx + s * D.BATT_L / 4 for s in (1, -1)]
    zrects(p, [(x - 5, x + 5, cy + W / 2, cy + W / 2 + t) for x in bars] + [(x - 5, x + 5, cy - W / 2 - t, cy - W / 2) for x in bars],
           z0 + H - 8, z0 + H, 'Bar_bosses')
    zcircles(p, [(x, cy + s * (W / 2 + t / 2), 1.3) for x in bars for s in (1, -1)], z0 + H - 8, z0 + H + 1, 'Bar_pilots', cut=True)
    # battery lead exit at +X end
    p.sketch(p.plane('Right', cx + L / 2 - 1))
    p.rect(-(z0 + H + 1), cy - 9, -(z0 + 6), cy + 9)
    p.cut(t + 2, 'Lead_exit')
    finish(p, PETG, True, 'Battery cage; with RR-04 bars the pack is boxed in and cannot fall out if the bot flips')


def rr04_bar():
    p = Part('RR-04_Battery_Bar', D.PARTS)
    cx, cy, L, W, t = cradle_box()
    z0 = D.FLOOR_Z0 + D.FLOOR_T + D.CRADLE_H
    zrects(p, [(-5, 5, cy - W / 2 - t, cy + W / 2 + t)], z0, z0 + 3, 'Bar')
    zcircles(p, [(0, cy + s * (W / 2 + t / 2), 1.7) for s in (1, -1)], z0 - 1, z0 + 4, 'M3_holes', cut=True)
    finish(p, PETG, True, 'Battery retaining bar x2 (M3 x 8 self-tap into the cage bosses)')


def hub_features(p, bore='D4'):
    """Wheel hub. Local frame: wheel axis = +Z, z=0 is the inner (chassis-side) face.
    bore 'D4' = 4 mm D-shaft (25GA-370); 'R6' = 6 mm round shaft (the team's local plastic-gearbox 370 motor)."""
    W = D.WHEEL_W
    R = D.WHEEL_D / 2 - D.TYRE_T   # 64
    zcircles(p, [(0, 0, R), (0, 0, R - 4)], 0, W, 'Rim')
    zcircles(p, [(0, 0, R + 1.5), (0, 0, R - 1)], 0, 1.5, 'Inner_bead_lip')
    zcircles(p, [(0, 0, R + 1.5), (0, 0, R - 1)], W - 1.5, W, 'Outer_bead_lip')
    # web sits on the inner face (z 0..7) so the hub prints flat on the bed without supports
    zcircles(p, [(0, 0, R - 3)], 0, 7, 'Web')
    zcircles(p, [(0, 0, 8)], 0, W - 4, 'Hub_boss')
    zcircles(p, [(37 * math.cos(math.radians(a)), 37 * math.sin(math.radians(a)), 13) for a in range(30, 360, 60)],
             -1, 8, 'Lightening_holes', cut=True)
    if bore == 'R6':     # round 6 mm shaft: 6.3 mm hole (printed holes shrink ~0.2), M3 grub clamps the shaft
        zcircles(p, [(0, 0, D.SHAFT6_BORE / 2)], -1, W + 1, 'Round_bore_6mm', cut=True)
        return
    # D-bore for the 4 mm shaft (flat at 1.5 mm)
    p.sketch(p.plane('Front', -1))
    a = math.degrees(math.acos(1.5 / 2.05))
    p.arc(0, 0, 2.05, a, 360 - a)
    p.line(1.5, -2.05 * math.sin(math.radians(a)), 1.5, 2.05 * math.sin(math.radians(a)))
    p.cut(W + 2, 'D_bore_4mm')


def tyre_features(p):
    """Revolved tread between the bead lips (same local frame)."""
    R0, R1 = D.WHEEL_D / 2 - D.TYRE_T, D.WHEEL_D / 2
    pts = [(1.5, R0), (26.5, R0), (26.5, R1 - 1), (25.5, R1), (19, R1), (19, R1 - 1.2), (17, R1 - 1.2), (17, R1),
           (11, R1), (11, R1 - 1.2), (9, R1 - 1.2), (9, R1), (2.5, R1), (1.5, R1 - 1)]
    p.sketch('Right Plane')
    p.poly([(-z, r) for z, r in pts])
    p.cline(0, 0, -30, 0)
    p.revolve(360, 'Tyre')


def grub_holes(p, through_tyre=False):
    R = D.WHEEL_D / 2 - D.TYRE_T
    xcircles(p, [(0, 3.6, 1.3)], 0, 9, 'Grub_screw_M3', cut=True)          # M3 grub onto the shaft flat
    xcircles(p, [(0, 3.6, 1.9)], 8.5, (D.WHEEL_D / 2 + 1) if through_tyre else R + 2, 'Grub_key_access', cut=True)


def rr05_hub():
    p = Part('RR-05_Wheel_Hub', D.PARTS)
    hub_features(p)
    grub_holes(p)
    finish(p, PETG_BLUE, True, 'Rear wheel hub D128 rim, PETG; prints web-down without supports; takes the RR-06 TPU tyre')


def rr05_hub_6mm():
    p = Part('RR-05_Wheel_Hub_6mm', D.PARTS)
    hub_features(p, 'R6')
    grub_holes(p)
    finish(p, PETG_BLUE, True, 'Rear wheel hub for a 6 mm round shaft, PETG; takes the RR-06 TPU tyre')


def rr06_tyre():
    """Local frame as the hub. Revolved TPU tyre between the bead lips (rear wheels)."""
    p = Part('RR-06_Wheel_Tyre', D.PARTS)
    tyre_features(p)
    R0, R1 = D.WHEEL_D / 2 - D.TYRE_T, D.WHEEL_D / 2
    xcircles(p, [(0, 3.6, 2.2)], R0 - 1, R1 + 1, 'Grub_key_hole', cut=True)
    finish(p, TPU, True, 'Rear tyre, TPU 95A, 3 walls 20% infill; key hole over hub grub screw')


def rr07_front_wheel():
    """One-piece PETG front wheel (hub + hard tread). A rigid PETG tread cannot be stretched over
    the bead lips, so the front wheels are printed as a single part."""
    p = Part('RR-07_Front_Wheel_PETG', D.PARTS)
    hub_features(p)
    tyre_features(p)
    # fill under the tread's first 1.5 mm so the one-piece wheel sits flat on the bed (no overhang ledge)
    zcircles(p, [(0, 0, D.WHEEL_D / 2 - 1), (0, 0, D.WHEEL_D / 2 - D.TYRE_T)], 0, 1.5, 'Bed_ring')
    grub_holes(p, through_tyre=True)
    finish(p, (0.55, 0.57, 0.6), True, 'Front wheel D140, one-piece PETG (hard tread lets the skid steer turn)')


def rr07_front_wheel_6mm():
    p = Part('RR-07_Front_Wheel_PETG_6mm', D.PARTS)
    hub_features(p, 'R6')
    tyre_features(p)
    zcircles(p, [(0, 0, D.WHEEL_D / 2 - 1), (0, 0, D.WHEEL_D / 2 - D.TYRE_T)], 0, 1.5, 'Bed_ring')
    grub_holes(p, through_tyre=True)
    finish(p, (0.55, 0.57, 0.6), True, 'Front wheel D140 for a 6 mm round shaft, one-piece PETG')


# ================================================================== purchased envelopes
def p01_motor():
    """JGA25-370 12 V. Local: shaft axis +Z, gearbox face at z=0, body toward -Z."""
    p = Part('P-01_Motor_JGA25-370_12V', D.PARTS)
    zcircles(p, [(0, 0, D.MOTOR_GB_D / 2)], -D.MOTOR_GB_L, 0, 'Gearbox')
    zcircles(p, [(0, 0, D.MOTOR_CAN_D / 2)], -D.MOTOR_GB_L - D.MOTOR_CAN_L, -D.MOTOR_GB_L, 'Can')
    zcircles(p, [(0, 0, 11)], -D.MOTOR_GB_L - D.MOTOR_CAN_L - D.MOTOR_CAP_L, -D.MOTOR_GB_L - D.MOTOR_CAN_L, 'End_cap')
    zcircles(p, [(0, 0, D.MOTOR_BOSS_D / 2)], 0, D.MOTOR_BOSS_L, 'Bearing_boss')
    # 4 mm D-shaft, flat at 1.45 mm (hub D-bore flat is at 1.5 mm)
    p.sketch(p.plane('Front', D.MOTOR_BOSS_L))
    r = D.MOTOR_SHAFT_D / 2
    a = math.degrees(math.acos(1.45 / r))
    p.arc(0, 0, r, a, 360 - a)
    p.line(1.45, -r * math.sin(math.radians(a)), 1.45, r * math.sin(math.radians(a)))
    p.extrude(D.MOTOR_SHAFT_L - D.MOTOR_BOSS_L, 'Shaft_4mm_D')
    p.sketch(p.plane('Front', 0))
    for dx in (-D.MOTOR_HOLE_PITCH / 2, D.MOTOR_HOLE_PITCH / 2):
        p.circle(dx, 0, 1.25)
    p.cut(5, 'M3_face_holes', reverse=True)
    finish(p, METAL, False, 'JGA25-370 12 V 77 rpm (78:1) metal gearmotor, 4 mm D shaft (nominal envelope)')


def box_part(name, desc, rgb, blocks, cyls=()):
    """Generic envelope: blocks = [(x0,x1,y0,y1,z0,z1,name)], cyls = [(x,y,r,z0,z1,name)]"""
    p = Part(name, D.PARTS)
    for x0, x1, y0, y1, z0, z1, n in blocks:
        zrects(p, [(x0, x1, y0, y1)], z0, z1, n)
    for x, y, r, z0, z1, n in cyls:
        zcircles(p, [(x, y, r)], z0, z1, n)
    finish(p, rgb, False, desc)


def purchased():
    # local frames: board underside at z=0, centred on the board
    box_part('P-02_LiPo_3S_2200mAh', '3S 11.1 V 2200 mAh 30C LiPo with XT60 (nominal 106x34x26)', (0.15, 0.25, 0.65),
             [(-D.BATT_L / 2, D.BATT_L / 2, -D.BATT_W / 2, D.BATT_W / 2, 0, D.BATT_H, 'Pack'),
              (D.BATT_L / 2, D.BATT_L / 2 + 8, -6, 6, 8, 18, 'Lead_exit')])
    box_part('P-03_ESP32_DevKitC', 'ESP32-DevKitC-32E (38-pin)', (0.1, 0.1, 0.12),
             [(-27.5, 27.5, -14, 14, 0, 1.6, 'PCB'),
              (-27.5, 27.5, -14, -11.5, -8.5, 0, 'Header_A'), (-27.5, 27.5, 11.5, 14, -8.5, 0, 'Header_B'),
              (9, 27.5, -9, 9, 1.6, 4.8, 'WROOM_module'), (-29.5, -22, -4, 4, 1.6, 4.6, 'USB')])
    box_part('P-04_PCA9685_16ch', 'PCA9685 16-channel 12-bit PWM driver (servo outputs)', (0.1, 0.25, 0.6),
             [(-31.1, 31.1, -12.7, 12.7, 0, 1.6, 'PCB'), (-20, 20, -12.7, -4.5, 1.6, 10.6, 'Servo_headers'),
              (-4, 4, 4, 12.7, 1.6, 11, 'V+_terminal')])
    # MDD10A Rev2.0 (user manual V2.0 p.6-7): terminal block on the -x short edge (wires enter from -x),
    # 5-pin 2510 input header on the +x edge (plugged connector sticks out ~12 mm), 2 x 330 uF caps 10 x 10.5
    box_part('P-05_Cytron_MDD10A', 'Cytron MDD10A Rev2.0 dual 10 A 5-30 V motor driver (84.5 x 62)', (0.08, 0.08, 0.09),
             [(-42.25, 42.25, -31, 31, 0, 1.6, 'PCB'), (-38.5, -29, -15.5, 15.5, 1.6, 12.5, 'Terminal_block'),
              (-38.5, -29, -15.5, 15.5, -2.5, 0, 'Terminal_pins'), (-29, 34, -28, 28, 1.6, 6, 'Components'),
              (34, 54, -7, 7, 1.6, 9, 'Input_connector')],
             [(-25.3, 1.2, 5.3, 1.6, 12.6, 'Cap_1'), (-10.0, 1.2, 5.3, 1.6, 12.6, 'Cap_2')])
    box_part('P-06_Buck_6V_10A', 'DC-DC buck 7-24 V -> 6.0 V >= 10 A (XL4016 / 300 W class) for 6 x MG996R', (0.1, 0.3, 0.12),
             [(-33, 33, -24, 24, 0, 1.6, 'PCB'), (-30, -10, -20, 20, 1.6, 18, 'Heatsink_in'), (10, 30, -20, 20, 1.6, 18, 'Heatsink_out')],
             [(0, 0, 11, 1.6, 16, 'Inductor')])
    box_part('P-07_Buck_5V_MP1584', 'MP1584 mini buck, 5.0 V for ESP32', (0.1, 0.3, 0.12),
             [(-11, 11, -8.5, 8.5, 0, 1.6, 'PCB'), (-4, 4, -4, 4, 1.6, 4.5, 'Inductor')])
    # kill switch local: z=0 = deck top surface, body hangs below
    box_part('P-08_Kill_Switch_KCD4_30A', 'KCD4 DPST rocker 30 A, red; panel 22.2 x 30.2 cut-out', (0.75, 0.05, 0.05),
             [(-11.6, 11.6, -15.6, 15.6, 0, 2.5, 'Bezel'), (-7, 7, -11, 11, 2.5, 8, 'Rocker'),
              (-10.8, 10.8, -14.8, 14.8, -24, 0, 'Body'), (-6, 6, -10, 10, -32, -24, 'Terminals')])


ALL = {'RR-01': rr01_tub, 'RR-02': rr02_deck, 'RR-03': rr03_cradle, 'RR-04': rr04_bar, 'RR-07': rr07_front_wheel,
       'RR-05': rr05_hub, 'RR-06': rr06_tyre, 'RR-05-6': rr05_hub_6mm, 'RR-07-6': rr07_front_wheel_6mm, 'P-01': p01_motor, 'P-XX': purchased}

if __name__ == '__main__':
    sw()
    keys = sys.argv[1:] or list(ALL)
    for k in keys:
        ALL[k]()
