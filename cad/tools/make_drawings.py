"""Dimensioned 2-D drawings of every printed rover part, cut straight from the exported STL geometry.

Each view is a planar section through the part. Every closed hole in the section is measured
(circles -> diameter, anything else -> width x length of its bounding rectangle), given an ID and
listed with its centre position. Positions are measured from the bottom-left corner of the view as
drawn (or from the centre for round parts), so they can be checked with a ruler or caliper.

  python cad/tools/make_drawings.py   -> docs/drawings/*.png + docs/drawings/drawings.json
Also draws the overall robot dimensions and the floor / deck layouts from design.py.
"""
import json
import math
import os
import string

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                   # noqa: E402
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle  # noqa: E402
import numpy as np                                                # noqa: E402
import trimesh                                                    # noqa: E402
from shapely.affinity import scale                              # noqa: E402
from shapely.geometry import MultiPolygon, Polygon               # noqa: E402

import design as D                                                # noqa: E402

STL = os.path.join(D.CAD, 'stl')
OUT = os.path.join(D.ROOT, 'docs', 'drawings')
INK, FILL, HOLE, DIM, ACC = '#1b2a3d', '#dfe6ef', '#ffffff', '#5b6573', '#b8531f'
AX = {'x': (1, 2), 'y': (0, 2), 'z': (0, 1)}
plt.rcParams.update({'font.family': 'Segoe UI', 'font.size': 8})


def section(mesh, axis, value):
    i = 'xyz'.index(axis)
    origin = [0.0, 0.0, 0.0]
    origin[i] = value
    normal = [0.0, 0.0, 0.0]
    normal[i] = 1.0
    sec = mesh.section(plane_origin=origin, plane_normal=normal)
    a, b = AX[axis]
    region = None
    for e in sec.discrete:
        pts = e[:, [a, b]]
        if len(pts) < 4:
            continue
        p = Polygon(pts).buffer(0)
        if p.is_empty:
            continue
        region = p if region is None else region.symmetric_difference(p)
    return region


def polys(region):
    return list(region.geoms) if isinstance(region, MultiPolygon) else [region]


def measure(ring):
    p = Polygon(ring)
    A, P = p.area, p.length
    c = p.centroid
    if 4 * math.pi * A / P ** 2 > 0.96:
        return {'kind': 'round', 'size': round(2 * math.sqrt(A / math.pi), 1), 'cx': c.x, 'cy': c.y}
    r = p.minimum_rotated_rectangle
    xs, ys = r.exterior.coords.xy
    e = sorted({round(math.dist((xs[k], ys[k]), (xs[k + 1], ys[k + 1])), 1) for k in range(4)})
    w, l = (e[0], e[-1]) if len(e) > 1 else (e[0], e[0])
    return {'kind': 'slot', 'size': (w, l), 'cx': c.x, 'cy': c.y}


def label_of(h):
    return f"Ø{h['size']:.1f}" if h['kind'] == 'round' else f"{h['size'][0]:.1f} × {h['size'][1]:.1f}"


def dim_line(ax, p0, p1, text, off, horizontal=True):
    (x0, y0), (x1, y1) = p0, p1
    if horizontal:
        y = y0 + off
        ax.plot([x0, x0], [y0, y], color=DIM, lw=0.5)
        ax.plot([x1, x1], [y1, y], color=DIM, lw=0.5)
        ax.add_patch(FancyArrowPatch((x0, y), (x1, y), arrowstyle='<|-|>', mutation_scale=7, color=DIM, lw=0.7))
        ax.text((x0 + x1) / 2, y, text, ha='center', va='center', fontsize=8, color=INK,
                bbox=dict(fc='white', ec='none', pad=1.2))
    else:
        x = x0 + off
        ax.plot([x0, x], [y0, y0], color=DIM, lw=0.5)
        ax.plot([x1, x], [y1, y1], color=DIM, lw=0.5)
        ax.add_patch(FancyArrowPatch((x, y0), (x, y1), arrowstyle='<|-|>', mutation_scale=7, color=DIM, lw=0.7))
        ax.text(x, (y0 + y1) / 2, text, ha='center', va='center', rotation=90, fontsize=8, color=INK,
                bbox=dict(fc='white', ec='none', pad=1.2))


def view(name, stl, axis, value, title, *, flip=False, origin='corner', hint=None, notes=(), figw=7.2, min_hole=0.0, dims=None):
    mesh = trimesh.load(os.path.join(STL, stl + '.STL'))
    region = section(mesh, axis, value)
    if flip:                     # mirror so the view is as seen from outside the part
        region = scale(region, xfact=-1, yfact=1, origin=(0, 0))
    x0, y0, x1, y1 = region.bounds
    ox, oy = ((x0 + x1) / 2, (y0 + y1) / 2) if origin == 'center' else (x0, y0)
    holes = []
    for p in polys(region):
        for ring in p.interiors:
            h = measure(ring)
            if h['kind'] == 'round' and h['size'] < min_hole:
                continue
            if Polygon(ring).area > 0.25 * (x1 - x0) * (y1 - y0):   # the open inside of a box, not a hole
                continue
            h['x'], h['y'] = round(h['cx'] - ox, 1), round(h['cy'] - oy, 1)
            h['ring'] = np.array(ring.coords)
            holes.append(h)
    # group identical sizes -> letters
    keys = sorted({(h['kind'], h['size'] if h['kind'] == 'round' else tuple(h['size'])) for h in holes},
                  key=lambda k: (k[0] != 'round', k[1] if k[0] == 'round' else k[1][0] * k[1][1]))
    letter = {k: string.ascii_uppercase[i] for i, k in enumerate(keys)}
    for k in keys:
        grp = [h for h in holes if (h['kind'], h['size'] if h['kind'] == 'round' else tuple(h['size'])) == k]
        grp.sort(key=lambda h: (round(h['x'] / 3), h['y']))
        for n, h in enumerate(grp, 1):
            h['id'] = f'{letter[k]}{n}'
    W_, H_ = x1 - x0, y1 - y0
    figh = max(2.2, figw * H_ / W_ * 0.92 + 1.1)
    fig, ax = plt.subplots(figsize=(figw, figh), dpi=200)
    for p in polys(region):
        xs, ys = p.exterior.xy
        ax.fill(xs, ys, fc=FILL, ec=INK, lw=0.8)
        for ring in p.interiors:
            xs, ys = ring.xy
            ax.fill(xs, ys, fc=HOLE, ec=INK, lw=0.7)
    small = max(W_, H_) / 60
    for h in holes:
        ax.text(h['cx'], h['cy'] + (0 if (h['kind'] == 'slot' or h['size'] > 2.2 * small) else small * 0.9), h['id'],
                ha='center', va='center', fontsize=6.2 if len(holes) > 14 else 7, color=ACC, fontweight='bold')
    off = max(W_, H_) * 0.07
    dw, dh = dims or (f'{W_:.1f}', f'{H_:.1f}')     # round parts: label the true outside diameter
    dim_line(ax, (x0, y0), (x1, y0), dw, -off, True)
    dim_line(ax, (x0, y0), (x0, y1), dh, -off, False)
    if origin == 'corner':
        ax.plot(x0, y0, marker='o', ms=4, color=ACC)
        ax.text(x0, y0 - off * 0.45, '0,0', fontsize=7, color=ACC, ha='center', va='top')
    else:
        ax.plot([ox - small, ox + small], [oy, oy], color=ACC, lw=0.6)
        ax.plot([ox, ox], [oy - small, oy + small], color=ACC, lw=0.6)
    if hint:
        ax.text(x1, y1 + off * 0.35, hint, ha='right', va='bottom', fontsize=7.5, color=DIM)
    ax.set_aspect('equal')
    ax.set_xlim(x0 - off * 1.6, x1 + off * 0.5)
    ax.set_ylim(y0 - off * 1.6, y1 + off * 0.9)
    ax.axis('off')
    fig.tight_layout(pad=0.2)
    path = os.path.join(OUT, name + '.png')
    fig.savefig(path, facecolor='white')
    plt.close(fig)
    rows = [{'id': h['id'], 'size': label_of(h), 'kind': h['kind'], 'x': h['x'], 'y': h['y']} for h in holes]
    rows.sort(key=lambda r: (r['id'][0], int(r['id'][1:])))
    return {'name': name, 'part': stl, 'title': title, 'png': path, 'size': [round(W_, 1), round(H_, 1)],
            'origin': origin, 'holes': rows, 'notes': list(notes)}


# ------------------------------------------------------------------------------------------ overall + layouts
def robot_dims():
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(7.4, 3.6), dpi=200, gridspec_kw={'width_ratios': [1.25, 1]})
    R = D.WHEEL_D / 2
    # side view (x right = front)
    a1.plot([-180, 180], [0, 0], color=INK, lw=1)
    a1.add_patch(Rectangle((-D.TUB_X, D.FLOOR_Z0), 2 * D.TUB_X, D.TUB_TOP_Z - D.FLOOR_Z0, fc=FILL, ec=INK, lw=0.8))
    a1.add_patch(Rectangle((-D.DECK_X, D.DECK_Z0), 2 * D.DECK_X, D.DECK_T, fc='#bcc9da', ec=INK, lw=0.8))
    a1.add_patch(Rectangle((D.ARM_X - 25, D.DECK_TOP), 230, 292.9 - D.DECK_TOP, fc='none', ec=ACC, lw=0.7, ls='--'))
    a1.text(D.ARM_X + 90, 250, 'arm (STOW)', color=ACC, fontsize=7, ha='center')
    for sx in (1, -1):
        a1.add_patch(Circle((sx * D.AXLE_X, R), R, fc='none', ec=INK, lw=1))
        a1.plot(sx * D.AXLE_X, R, 'o', ms=2, color=INK)
    dim_line(a1, (-D.AXLE_X, R), (D.AXLE_X, R), f'wheelbase {2 * D.AXLE_X:.0f}', -R - 18)
    dim_line(a1, (-146.85, 0), (146.85, 0), 'overall 293.7', -48)
    dim_line(a1, (D.TUB_X, 0), (D.TUB_X, D.FLOOR_Z0), f'{D.FLOOR_Z0:.0f}', 18, False)
    dim_line(a1, (D.TUB_X + 10, 0), (D.TUB_X + 10, D.DECK_TOP), f'deck {D.DECK_TOP:.0f}', 40, False)
    dim_line(a1, (-D.TUB_X - 30, 0), (-D.TUB_X - 30, 292.9), 'height 292.9', -10, False)
    a1.text(D.AXLE_X, R, f'  Ø{D.WHEEL_D:.0f}', fontsize=7, va='bottom')
    a1.text(190, 300, 'FRONT →', fontsize=7, color=DIM, ha='right')
    a1.set_title('Side view (left side)', fontsize=9, color=INK)
    # front view
    yw = D.WHEEL_YC
    a2.plot([-130, 130], [0, 0], color=INK, lw=1)
    a2.add_patch(Rectangle((-D.TUB_Y, D.FLOOR_Z0), 2 * D.TUB_Y, D.TUB_TOP_Z - D.FLOOR_Z0, fc=FILL, ec=INK, lw=0.8))
    a2.add_patch(Rectangle((-D.DECK_Y, D.DECK_Z0), 2 * D.DECK_Y, D.DECK_T, fc='#bcc9da', ec=INK, lw=0.8))
    for sy in (1, -1):
        a2.add_patch(Rectangle((sy * yw - D.WHEEL_W / 2, 0), D.WHEEL_W, D.WHEEL_D, fc='#c9ced6', ec=INK, lw=0.8))
    dim_line(a2, (-yw, D.WHEEL_D), (yw, D.WHEEL_D), f'track {2 * yw:.0f}', 16)
    dim_line(a2, (-98, 0), (98, 0), 'overall 196', -26)
    dim_line(a2, (-D.TUB_Y, D.FLOOR_Z0), (D.TUB_Y, D.FLOOR_Z0), f'tub {2 * D.TUB_Y:.0f}', -14)
    a2.text(0, D.FLOOR_Z0 / 2, f'clearance {D.FLOOR_Z0:.0f}\n(rule ≥ 50)', ha='center', va='center', fontsize=7, color=ACC)
    a2.set_title('Front view', fontsize=9, color=INK)
    for a in (a1, a2):
        a.set_aspect('equal')
        a.axis('off')
    a1.set_xlim(-185, 200)
    a1.set_ylim(-65, 305)
    a2.set_xlim(-130, 130)
    a2.set_ylim(-40, 175)
    fig.tight_layout(pad=0.3)
    p = os.path.join(OUT, 'robot_dimensions.png')
    fig.savefig(p, facecolor='white')
    plt.close(fig)
    return p


def floor_layout():
    fig, ax = plt.subplots(figsize=(7.4, 4.6), dpi=200)
    X, Y = D.TUB_X, D.TUB_Y
    ax.add_patch(Rectangle((-X, -Y), 2 * X, 2 * Y, fc=FILL, ec=INK, lw=1))
    ax.add_patch(Rectangle((-X + D.WALL_END_T, -Y + D.WALL_SIDE_T), 2 * (X - D.WALL_END_T), 2 * (Y - D.WALL_SIDE_T), fc='white', ec=INK, lw=0.6))
    for sx in (1, -1):
        for sy in (1, -1):
            y_face = sy * D.MOTOR_FACE_Y
            L = D.MOTOR_GB_L + D.MOTOR_CAN_L + D.MOTOR_CAP_L
            y0 = y_face - sy * L
            ax.add_patch(Rectangle((sx * D.AXLE_X - D.MOTOR_GB_D / 2, min(y0, y_face)), D.MOTOR_GB_D, L, fc='#d8dce3', ec=INK, lw=0.6, hatch='///'))
    ax.text(D.AXLE_X, 0, 'motors\n(25 mm CAD\n— to be redrawn\nfor the 35 mm\nmotor)', ha='center', va='center', fontsize=6, color=ACC)
    cx, cy = D.BATT_C
    L, W, t = D.BATT_L + 2, D.BATT_W + 2, D.CRADLE_T
    ax.add_patch(Rectangle((cx - L / 2 - t, cy - W / 2 - t), L + 2 * t, W + 2 * t, fc='#eef1f5', ec=INK, lw=0.7))
    ax.add_patch(Rectangle((cx - D.BATT_L / 2, cy - D.BATT_W / 2), D.BATT_L, D.BATT_W, fc='#c7d4ea', ec=INK, lw=0.5))
    ax.text(cx, cy, f'battery cage\nLiPo {D.BATT_L:.0f}×{D.BATT_W:.0f}×{D.BATT_H:.0f}', ha='center', va='center', fontsize=7)
    names = {'BUCK6V': '6 V buck', 'PCA9685': 'PCA9685', 'BUCK5V': '5 V\nbuck', 'ESP32': 'ESP32 (hung under deck)'}
    for k, (bx, by, rot, mount) in D.BOARDS.items():
        if mount == 'deck_top':
            continue
        L_, W_ = D.BOARD_SIZE[k]
        if rot % 180:
            L_, W_ = W_, L_
        ls = '--' if mount == 'deck' else '-'
        ax.add_patch(Rectangle((bx - L_ / 2, by - W_ / 2), L_, W_, fc='none' if mount == 'deck' else '#e9f2ea', ec=INK, lw=0.7, ls=ls))
        ax.text(bx, by + (W_ / 2 - 2 if mount == 'deck' else 0), names[k], ha='center', va='top' if mount == 'deck' else 'center', fontsize=6.5)
    for nm, (px, py, L_, W_) in D.SPARE_FLOOR_PADS.items():
        ax.add_patch(Rectangle((px - L_ / 2, py - W_ / 2), L_, W_, fc='none', ec=DIM, lw=0.5, ls=':'))
        ax.text(px, py, 'spare pad', ha='center', va='center', fontsize=6, color=DIM)
    for x, y in D.BOSS_XY:
        ax.add_patch(Circle((x, y), D.BOSS_D / 2, fc='#cfd6df', ec=INK, lw=0.5))
    dim_line(ax, (-X, -Y), (X, -Y), f'{2 * X:.0f}', -14)
    dim_line(ax, (-X, -Y), (-X, Y), f'{2 * Y:.0f}', -14, False)
    ax.text(X, Y + 6, 'FRONT →   (top view, deck removed; +Y = robot left)', ha='right', fontsize=7, color=DIM)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_xlim(-X - 30, X + 8)
    ax.set_ylim(-Y - 30, Y + 16)
    fig.tight_layout(pad=0.3)
    p = os.path.join(OUT, 'layout_floor.png')
    fig.savefig(p, facecolor='white')
    plt.close(fig)
    return p


def deck_layout():
    fig, ax = plt.subplots(figsize=(7.4, 4.6), dpi=200)
    X, Y = D.DECK_X, D.DECK_Y
    ax.add_patch(Rectangle((-X, -Y), 2 * X, 2 * Y, fc='#dbe3ee', ec=INK, lw=1))
    fx0, fx1, fy0, fy1 = D.ARM_FOOT
    ax.add_patch(Rectangle((D.ARM_X + fx0, fy0), fx1 - fx0, fy1 - fy0, fc='#f3e1d6', ec=ACC, lw=0.8))
    ax.text(D.ARM_X + (fx0 + fx1) / 2, 0, 'ARM BASE\n(inside the fence)', ha='center', va='center', fontsize=7, color=ACC)
    for x, y in D.ARM_SLOTS:
        ax.add_patch(Circle((D.ARM_X + x, y), 2.5, fc='white', ec=INK, lw=0.6))
    kx, ky = D.KILL_XY
    w, h = D.KILL_CUT
    ax.add_patch(Rectangle((kx - h / 2, ky - w / 2), h, w, fc='#f6c9c4', ec=INK, lw=0.7))
    ax.text(kx, ky, 'KILL\nSWITCH', ha='center', va='center', fontsize=6.5)
    bx, by, rot, _ = D.BOARDS['MDD10A']
    L_, W_ = D.BOARD_SIZE['MDD10A']
    ax.add_patch(Rectangle((bx - L_ / 2, by - W_ / 2), L_, W_, fc='#3b3f46', ec=INK, lw=0.7, alpha=0.85))
    ax.text(bx, by, 'MOTOR DRIVER\nCytron MDD10A\n84.5 × 62', ha='center', va='center', fontsize=7, color='white')
    ax.text(bx + L_ / 2 - 2, by, 'terminals ►', ha='right', va='bottom', fontsize=6, color='white', rotation=0)
    sx0, sx1, sy0, sy1 = D.MOTOR_WIRE_SLOT
    ax.add_patch(Rectangle((sx0, sy0), sx1 - sx0, sy1 - sy0, fc='white', ec=INK, lw=0.7))
    ax.text((sx0 + sx1) / 2, sy1 + 2, 'wire slot', ha='center', va='bottom', fontsize=6)
    ix1 = D.ARM_X + fx1 + 0.5 + D.FENCE_T
    ax.add_patch(Rectangle((ix1 + 2, -15), 12, 30, fc='white', ec=INK, lw=0.7))
    ax.text(ix1 + 8, -18, 'servo +\nsignal slot', ha='center', va='top', fontsize=6)
    ex, ey, _, _ = D.BOARDS['ESP32']
    ax.add_patch(Rectangle((ex - 27.5, ey - 14), 55, 28, fc='none', ec=DIM, lw=0.6, ls='--'))
    ax.text(ex, ey - 16, 'ESP32 (under the deck)', ha='center', va='top', fontsize=6, color=DIM)
    for x, y in D.BOSS_XY:
        ax.add_patch(Circle((x, y), 3.25, fc='white', ec=INK, lw=0.6))
    dim_line(ax, (-X, -Y), (X, -Y), f'{2 * X:.0f}', -14)
    dim_line(ax, (-X, -Y), (-X, Y), f'{2 * Y:.0f}', -14, False)
    ax.text(X, Y + 6, 'FRONT →   (top view; +Y = robot left)', ha='right', fontsize=7, color=DIM)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_xlim(-X - 30, X + 8)
    ax.set_ylim(-Y - 30, Y + 16)
    fig.tight_layout(pad=0.3)
    p = os.path.join(OUT, 'layout_deck.png')
    fig.savefig(p, facecolor='white')
    plt.close(fig)
    return p


def main():
    os.makedirs(OUT, exist_ok=True)
    zf = D.FLOOR_Z0 + D.FLOOR_T - 0.5
    views = [
        view('RR-01_floor', 'RR-01_Chassis_Tub', 'z', zf, 'RR-01 Chassis tub — floor (top view)',
             hint='FRONT →', notes=['Section through the 3 mm floor. Board standoff pilots (Ø2.2) are 1 mm deep into the floor, '
                                    'under 3 mm tall Ø6 posts.', 'The 4 big cut-outs at the corners are the motor floor windows.']),
        view('RR-01_wall_left', 'RR-01_Chassis_Tub', 'y', D.TUB_Y - 1.0, 'RR-01 Chassis tub — left side wall (seen from outside)',
             flip=True, hint='← FRONT', notes=['Motor slots: 26 mm wide U-slots, round top centred on the axle (13 mm above the tub bottom = '
                                               '70 mm above the ground), open at the bottom.',
                                               'Per motor: Ø8 shaft/boss hole + 2 × Ø3.4 (M3) holes 17 mm apart. The right wall is the mirror image.',
                                               'Sized for the 25 mm motor — will change for the 35 mm motor.']),
        view('RR-01_wall_front', 'RR-01_Chassis_Tub', 'x', D.TUB_X - D.WALL_END_T / 2, 'RR-01 Chassis tub — front wall (seen from the front)',
             flip=False, notes=['5 ventilation slots 8 × 24 mm.']),
        view('RR-01_wall_rear', 'RR-01_Chassis_Tub', 'x', -D.TUB_X + D.WALL_END_T / 2, 'RR-01 Chassis tub — rear wall (seen from the rear)',
             flip=True,
             notes=['USB grommet window 16 × 14 mm for the ESP32 cable.']),
        view('RR-01_top', 'RR-01_Chassis_Tub', 'z', D.TUB_TOP_Z - 2, 'RR-01 Chassis tub — top of the walls',
             hint='FRONT →', notes=['Corner columns Ø9 with Ø4.0 bores, 8 mm deep: melt in the M3 heat-set inserts.']),
        view('RR-02_deck', 'RR-02_Deck_Plate', 'z', D.DECK_Z0 + 1.5, 'RR-02 Deck plate (top view)', hint='FRONT →',
             notes=['Ø3.4 corner holes, counterbored Ø6.5 × 2.5 deep on top: deck → tub, M3 × 10.',
                    'Arm base: 4 × Ø3.4 through the arm vent slots, M3 × 20 + nyloc under the deck.',
                    'MDD10A: 4 × Ø2.6 pilots in Ø7 × 6 mm bosses on top, 78.74 × 55.88 apart (M3 × 10 self-tapping).',
                    'ESP32: 2 × Ø2.8 for the M2.5 brass standoffs.']),
        view('RR-03_cage_base', 'RR-03_Battery_Cradle', 'z', D.FLOOR_Z0 + D.FLOOR_T + 1.5, 'RR-03 Battery cage — base and floor tabs',
             notes=['Inside 108 × 36 mm (battery max 106 × 34 × 26). Tabs: 2 × Ø3.4 for M3 × 8 + nyloc through the tub floor.']),
        view('RR-03_cage_top', 'RR-03_Battery_Cradle', 'z', D.FLOOR_Z0 + D.FLOOR_T + D.CRADLE_H - 2, 'RR-03 Battery cage — top (bar screw bosses)',
             notes=['4 × Ø2.6 pilots, 8 mm deep: M3 × 8 self-tapping for the two bars.']),
        view('RR-04_bar', 'RR-04_Battery_Bar', 'z', D.FLOOR_Z0 + D.FLOOR_T + D.CRADLE_H + 1.5, 'RR-04 Battery bar (×2)',
             notes=['3 mm thick.']),
        view('RR-05_hub_6mm', 'RR-05_Wheel_Hub_6mm', 'z', 1.5, 'RR-05 Rear wheel hub, 6 mm shaft (inner face)', origin='center',
             dims=('Ø131 (bead lips) / Ø128 rim', 'Ø131'),
             notes=['Positions from the wheel centre. Rim Ø128 between two bead lips Ø131; width 28.',
                    'Bore Ø6.3 round. M3 grub screw: Ø2.6 radial hole 3.6 mm from the inner face, hex-key access Ø3.8 through the rim.']),
        view('RR-07_front_6mm', 'RR-07_Front_Wheel_PETG_6mm', 'z', 1.5, 'RR-07 Front wheel, 6 mm shaft (inner face)', origin='center',
             dims=('Ø140', 'Ø140'),
             notes=['Positions from the wheel centre. Ø140 one-piece, width 28, tread with 2 grooves.',
                    'Bore Ø6.3 round. Grub screw as the rear hub; the key access hole goes through the tread.']),
        view('RR-06F_strip', 'RR-06F_Tyre_Strip_Half', 'z', 3.0, 'RR-06F TPU tyre half-strip (top view, 4 per robot)',
             notes=['6 mm thick, 25 mm wide, 224.3 mm long including the knob. Two halves make one tyre (412.6 mm, 2 % shorter than the hub).',
                    'Ø4.4 key hole sits over the hub grub screw (edge with the hole toward the robot).']),
    ]
    extra = {'robot_dimensions': robot_dims(), 'layout_floor': floor_layout(), 'layout_deck': deck_layout()}
    json.dump({'views': views, 'extra': extra}, open(os.path.join(OUT, 'drawings.json'), 'w'), indent=1)
    for v in views:
        print(f"{v['name']:18s} {v['size'][0]:6.1f} x {v['size'][1]:6.1f}  holes {len(v['holes'])}")


if __name__ == '__main__':
    main()
