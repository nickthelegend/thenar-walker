"""RoboReach 2026-27 arena (3 x 3 m) in SolidWorks, from the rulebook text + renders.

Exact (rulebook): 15 deg ramp to 80 mm, 400 mm bridge, R40 speed breakers (40 mm peak),
50/60 mm cubes, D60 x 80 cylinders, R30 x 100 cone, 400 mm Downy pole, buzz wire <= 400 mm,
600 x 600 escape box, 100 x 100 / 80 x 80 target squares, A4 (210 x 297) paper, 350 x 250 end
zone, 250 mm diagonal track, throw lines every 100 mm over 1 m.
Approximate: where things sit on the board (rulebook: 'exact dimensions may differ by ~10 %').

Frame: board top surface is z = 0, origin at the board centre; the start corner is (-1500,-1500).
Lane A runs along the left edge (+Y): pickup -> ramp -> bridge -> ramp down -> speed breakers.
Outputs cad/arena/*.SLDPRT and cad/arena/RoboReach_Arena.SLDASM (robot parked in the start zone).
"""
import math
import os
import sys

import numpy as np
import pythoncom
from win32com.client import VARIANT

import swlib
from swlib import Part, cast, sw, MM
import design as D
import terrain as T
import build_parts as BP

A = D.ARENA
WALL_H, WALL_T = 60.0, 25.0
LANE_A_X = (-1475.0, -912.5)
RAMP_X0, RAMP_W, RAMP_Y0 = -1400.0, 400.0, -800.0   # ramp runs along +Y in lane A

GREY_BOARD = (0.22, 0.23, 0.25)
GREY_WALL = (0.82, 0.83, 0.85)
WOOD = (0.86, 0.74, 0.55)
BEIGE = (0.90, 0.80, 0.62)
RED = (0.80, 0.08, 0.08)
BLUE = (0.10, 0.30, 0.75)
WHITE = (0.97, 0.97, 0.97)
COPPER = (0.80, 0.45, 0.20)

ZONES = {   # name: (cx, cy, w(x), h(y)) — 1 mm floor markings
    'Start': (-1250.0, -1250.0, 450, 450),
    'Pickup_spot': (-1200.0, -925.0, 100, 100),
    'Touchdown_80': (-1150.0, 960.0, 80, 80),          # same as sim/build_model.TOUCHDOWN
    'Stack_100': (-1150.0, 1150.0, 100, 100),
    'Longshot_spot': (-650.0, 1200.0, 100, 100),
    'Marker_spot': (650.0, 1250.0, 100, 100),
    'Downy_target': (600.0, 700.0, 100, 100),
    'Hanoi_1': (650.0, 250.0, 100, 100),
    'Hanoi_2': (950.0, 250.0, 100, 100),
    'Hanoi_3': (1250.0, 250.0, 100, 100),
    'End_350x250': (175.0, -1340.0, 350, 250),
}
PAPER = (1250.0, 1300.0, 297, 210)
THROW_LINES = [(-500.0 + 100 * i) for i in range(11)]
POLE = (1100.0, 800.0)
BUZZ = ((650.0, -750.0), (1300.0, -750.0))
ESCAPE_C = (-250.0, 250.0)
WALLS = [  # x0, x1, y0, y1
    (-912.5, -887.5, -1100, 700),        # lane A inner
    (-900, 400, 887.5, 912.5),           # lane B inner
    (387.5, 412.5, 600, 1000),           # rooms separator (upper)
    (387.5, 412.5, -1100, -300),         # rooms separator (lower)
    (400, 1100, 1037.5, 1062.5),         # paper | pole
    (700, 1475, 537.5, 562.5),           # pole | hanoi
    (400, 1150, -462.5, -437.5),         # hanoi | buzz
    (-900, 387.5, -412.5, -387.5),       # centre | slalom
]
SLALOM = [  # triangle (base on a wall, 800 long, 50 high) -> 250+ mm passages
    [(-850, -1475), (-750, -1475), (-800, -675)],
    [(-475, -412.5), (-375, -412.5), (-425, -1212.5)],
    [(-100, -1475), (0, -1475), (-50, -675)],
    [(275, -412.5), (375, -412.5), (325, -1212.5)],
]
DIAGONALS = [((800.0, -1300.0), 500, 20.0), ((706.0, -1042.0), 500, 20.0)]   # (centre, length, angle) -> 250 mm channel


def zone_rect(cx, cy, w, h):
    return (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2)


def ramp_profile_yz():
    """Ramp + bridge + ramp-down polygon in (y, z) for lane A."""
    y0 = RAMP_Y0
    return [(y0 + T.X_UP, 0), (y0 + T.X_TOP0, T.H), (y0 + T.X_TOP1, T.H), (y0 + T.X_DOWN, 0)]


def breaker_centres_y():
    return [RAMP_Y0 + c for c in T.BREAKERS]


def colour(p, feat, rgb):
    try:
        p.color_faces(feat, *rgb)
    except Exception as e:   # colouring is cosmetic
        print('  colour skipped', e)


def board():
    p = Part('RA-01_Arena_Board', A)
    f = BP.zrects(p, [(-1500, 1500, -1500, 1500)], -18, 0, 'Board_18mm_ply')
    p.color(*GREY_BOARD)
    # perimeter ring
    p.sketch(p.plane('Front', 0))
    p.rect(-1500, -1500, 1500, 1500)
    p.rect(-1475, -1475, 1475, 1475)
    colour(p, p.extrude(WALL_H, 'Perimeter_wall'), GREY_WALL)
    for i, (x0, x1, y0, y1) in enumerate(WALLS):
        colour(p, BP.zrects(p, [(x0, x1, y0, y1)], 0, WALL_H, f'Wall_{i + 1}'), GREY_WALL)
    for i, ((cx, cy), L, a) in enumerate(DIAGONALS):
        p.sketch(p.plane('Front', 0)); p.crect(cx, cy, L, WALL_T, a)
        colour(p, p.extrude(WALL_H, f'Diagonal_{i + 1}'), GREY_WALL)
    for i, tri in enumerate(SLALOM):
        colour(p, BP.zpoly(p, tri, 0, 50, f'Slalom_ridge_{i + 1}'), GREY_WALL)
    # floor markings
    for n, (cx, cy, w, h) in ZONES.items():
        colour(p, BP.zrects(p, [zone_rect(cx, cy, w, h)], 0, 1, 'Zone_' + n), BEIGE)
    colour(p, BP.zrects(p, [(x - 10, x + 10, 950, 1450) for x in THROW_LINES], 0, 1, 'Longshot_lines_100mm'), RED)
    colour(p, BP.zrects(p, [zone_rect(*PAPER)], 0, 0.5, 'Paper_A4_TF'), WHITE)
    # ramp + bridge (profile in YZ, extruded across the 400 mm width)
    p.sketch(p.plane('Right', RAMP_X0))
    p.poly([(-z, y) for y, z in ramp_profile_yz()])
    colour(p, p.extrude(RAMP_W, 'Ramp_15deg_bridge_400'), WOOD)
    # speed breakers: half cylinders R40 across lane A
    p.sketch(p.plane('Right', LANE_A_X[0]))
    for yc in breaker_centres_y():
        r = T.R_BREAK
        p.arc(0, yc, r, 90, 270)
        p.line(0, yc - r, 0, yc + r)
    colour(p, p.extrude(LANE_A_X[1] - LANE_A_X[0], 'Speed_breakers_R40'), WOOD)
    # Downy pole: 40 x 40 post, 80 x 80 top plate at 400 mm
    px, py = POLE
    BP.zrects(p, [(px - 60, px + 60, py - 60, py + 60)], 0, 12, 'Pole_foot')
    BP.zrects(p, [(px - 20, px + 20, py - 20, py + 20)], 12, 390, 'Pole_post')
    colour(p, BP.zrects(p, [(px - 40, px + 40, py - 40, py + 40)], 390, 400, 'Pole_top_400'), WOOD)
    # escape room: 600 x 600 box, 80 high, four corner blocks with shaped holes
    ex, ey = ESCAPE_C
    p.sketch(p.plane('Front', 0))
    p.rect(ex - 300, ey - 300, ex + 300, ey + 300)
    p.rect(ex - 275, ey - 275, ex + 275, ey + 275)
    colour(p, p.extrude(80, 'Escape_box_600'), GREY_WALL)
    corners = [(ex + sx * 225, ey + sy * 225) for sx in (-1, 1) for sy in (-1, 1)]
    colour(p, BP.zrects(p, [(cx - 50, cx + 50, cy - 50, cy + 50) for cx, cy in corners], 60, 80, 'Escape_corner_plates'), GREY_WALL)
    (c1, c2, c3, c4) = corners
    p.sketch(p.plane('Front', 59))
    p.circle(c1[0], c1[1], 33)                                                     # cylinder hole
    p.rect(c2[0] - 31, c2[1] - 31, c2[0] + 31, c2[1] + 31)                         # cube hole
    tri = [(c3[0] + 40 * math.cos(math.radians(a)), c3[1] + 40 * math.sin(math.radians(a))) for a in (90, 210, 330)]
    p.poly(tri)                                                                    # triangular prism hole
    p.poly([(c4[0] + 34 * math.cos(math.radians(a)), c4[1] + 34 * math.sin(math.radians(a))) for a in range(0, 360, 60)])  # hexagon
    p.cut(30, 'Escape_shape_holes')
    # buzz wire: two posts (200 mm) and a swept D4 wire peaking at 400 mm
    (ax, ay), (bx, by) = BUZZ
    BP.zcircles(p, [(ax, ay, 8), (bx, by, 8)], 0, 200, 'Buzz_posts')
    BP.zrects(p, [(ax - 40, ax + 40, ay - 40, ay + 40), (bx - 40, bx + 40, by - 40, by + 40)], 0, 10, 'Buzz_feet')
    pts = [(ax, 200), (ax + 60, 290), (ax + 170, 380), (ax + 300, 300), (ax + 420, 395), (ax + 560, 330), (bx, 200)]
    p.sketch(p.plane('Top', ay))
    data = []
    for x, z in pts:
        data += [x * MM, -z * MM, 0.0]
    p.sm.CreateSpline2(VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, data), True)
    nm = p.end_sketch()
    p.clear(); p.ext.SelectByID2(nm, 'SKETCH', 0, 0, 0, False, 4, None, 0)
    f = p.fm.InsertProtrusionSwept4(False, False, 0, False, False, 0, 0, False, 0, 0, 0, 0, True, True, True, 0, True, True, 4 * MM, 0)
    if f is not None:
        colour(p, cast(f, 'IFeature'), COPPER)
    else:
        print('  buzz wire sweep failed (posts kept)')
    finish(p)


def finish(p, rgb=None):
    if rgb:
        p.color(*rgb)
    p.save(stl=False)
    os.makedirs(os.path.join(D.RENDERS, 'arena'), exist_ok=True)
    p.snapshot(os.path.join(D.RENDERS, 'arena', p.name + '.png'))
    print(f'{p.name:28s} bbox {p.bbox()}', flush=True)
    p.close()


def objects():
    def obj(name, rgb, build):
        p = Part(name, A); build(p); finish(p, rgb)
    obj('RA-10_Cube_50', BLUE, lambda p: BP.zrects(p, [(-25, 25, -25, 25)], 0, 50, 'Cube'))
    obj('RA-11_Cube_60', BLUE, lambda p: BP.zrects(p, [(-30, 30, -30, 30)], 0, 60, 'Cube'))
    obj('RA-12_Cylinder_60x80', BLUE, lambda p: BP.zcircles(p, [(0, 0, 30)], 0, 80, 'Cylinder'))

    def cone(p):
        p.sketch('Right Plane'); p.poly([(0, 0), (0, 30), (-100, 0)]); p.cline(0, 0, -110, 0); p.revolve(360, 'Cone')
    obj('RA-13_Cone_R30x100', BLUE, cone)

    def marker(p):
        BP.zcircles(p, [(0, 0, 9)], 0, 120, 'Barrel'); BP.zcircles(p, [(0, 0, 9.5)], 120, 150, 'Cap')
    obj('RA-14_Marker', (0.1, 0.1, 0.1), marker)

    def loop(p):   # buzz loop: ring ID 40 on a 150 mm handle
        p.sketch(p.plane('Top', -3)); p.circle(0, -150, 28); p.circle(0, -150, 20); p.extrude(6, 'Ring')
        p.sketch(p.plane('Top', -6)); p.rect(-6, -122, 6, 0); p.extrude(12, 'Handle')
    obj('RA-15_Buzz_Loop', (0.9, 0.75, 0.1), loop)
    obj('RA-16_Escape_Triangle', BLUE, lambda p: BP.zpoly(p, [(30 * math.cos(math.radians(a)), 30 * math.sin(math.radians(a))) for a in (90, 210, 330)], 0, 60, 'Prism'))
    obj('RA-17_Escape_Hexagon', BLUE, lambda p: BP.zpoly(p, [(28 * math.cos(math.radians(a)), 28 * math.sin(math.radians(a))) for a in range(0, 360, 60)], 0, 60, 'Prism'))


if __name__ == '__main__':
    sw()
    which = sys.argv[1:] or ['board', 'objects']
    if 'board' in which:
        board()
    if 'objects' in which:
        objects()
