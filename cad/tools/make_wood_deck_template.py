"""1:1 paper template for the 3 mm wooden deck (replaces the printed RR-02 deck plate).

Print at 100 % ("Actual size") on A4 landscape, check the 100 mm scale bar, tape it on the wood, then
cut on the outline, drill at the cross-hairs and cut the rectangular windows.
Also writes an SVG (outline + holes + windows) for a laser-cutting shop.

  python cad/tools/make_wood_deck_template.py
  -> docs/wood_deck/Wood_Deck_Template_A4_1to1.pdf, Wood_Deck_Laser.svg, wood_deck.json
"""
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.font_manager import FontProperties              # noqa: E402
from matplotlib.patches import Circle, PathPatch, Rectangle     # noqa: E402
from matplotlib.textpath import TextPath                         # noqa: E402
from matplotlib.transforms import Affine2D                       # noqa: E402

import build_parts as BP                                         # noqa: E402  (board hole positions)
import design as D                                               # noqa: E402

OUT = os.path.join(D.ROOT, 'docs', 'wood_deck')
X, Y = D.DECK_X, D.DECK_Y                       # 250 x 138
INK, CUT, DRILL, ENGR = '#111111', '#c0392b', '#1f4e8c', '#7a4a12'
FONTS = os.path.join(os.environ.get('WINDIR', 'C:/Windows'), 'Fonts')
FONT = FontProperties(fname=next(p for p in (os.path.join(FONTS, f) for f in ('ariblk.ttf', 'seguibl.ttf', 'arialbd.ttf')) if os.path.exists(p)))

# logos: (text, centre x, centre y, cap height mm, rotation deg) — robot frame, mm from the deck centre
LOGOS = []   # logos are engraved on the printed tub (design.LOGOS), not on the wood           # left strip beside the arm base


def holes():
    """(label, x, y, diameter, what) in the robot frame."""
    hs = [('A', x, y, 3.4, 'deck → tub: M3 × 10 into the heat-set inserts') for x, y in D.BOSS_XY]
    hs += [('B', D.ARM_X + x, y, 3.4, 'arm base: M3 × 20 + washer + nyloc') for x, y in D.ARM_SLOTS]
    hs += [('C', x, y, 3.4, 'motor driver MDD10A: M3 × 16 + 6 mm spacer + nut') for x, y in BP.board_corners('MDD10A')]
    hs += [('D', x, y, 2.8, 'ESP32: M2.5 × 10 brass standoffs underneath') for x, y in BP.board_corners('ESP32')]
    return hs


def windows():
    """(label, x0, x1, y0, y1, what)"""
    kx, ky = D.KILL_XY
    w, h = D.KILL_CUT
    ix1 = D.ARM_X + D.ARM_FOOT[1] + 0.5 + D.FENCE_T
    sx0, sx1, sy0, sy1 = D.MOTOR_WIRE_SLOT
    return [('K', kx - h / 2, kx + h / 2, ky - w / 2, ky + w / 2, 'kill switch cut-out (KCD4 snaps in)'),
            ('S1', ix1 + 2, ix1 + 14, -15, 15, 'slot: servo leads + driver signal wires'),
            ('S2', sx0, sx1, sy0, sy1, 'slot: motor + battery wires to the driver')]


def logo_path(text, cx, cy, cap, rot):
    tp = TextPath((0, 0), text, size=cap / 0.716, prop=FONT)      # Arial Black cap height ≈ 0.716 em
    bb = tp.get_extents()
    tr = Affine2D().translate(-(bb.x0 + bb.x1) / 2, -(bb.y0 + bb.y1) / 2).rotate_deg(rot).translate(cx, cy)
    return tr.transform_path(tp), bb.width, bb.height


def check(hs, ws, logos):
    """Logos must stay clear of every hole, window and the parts mounted on top."""
    from shapely.geometry import Point, box
    keep = [Point(x, y).buffer(d / 2 + 2) for _, x, y, d, _ in hs] + [box(x0 - 2, y0 - 2, x1 + 2, y1 + 2) for _, x0, x1, y0, y1, _ in ws]
    fx0, fx1, fy0, fy1 = D.ARM_FOOT
    keep.append(box(D.ARM_X + fx0, fy0, D.ARM_X + fx1, fy1))                       # arm base footprint
    bx, by, _, _ = D.BOARDS['MDD10A']
    L, W = D.BOARD_SIZE['MDD10A']
    keep.append(box(bx - L / 2, by - W / 2, bx + L / 2, by + W / 2))              # driver on top
    for text, path, w, h in logos:
        x0, y0 = path.vertices.min(0)
        x1, y1 = path.vertices.max(0)
        r = box(x0, y0, x1, y1)
        assert -X + 3 < x0 and x1 < X - 3 and -Y + 3 < y0 and y1 < Y - 3, (text, 'too close to the edge')
        for k in keep:
            assert not r.intersects(k), (text, 'overlaps', k.bounds)
        print(f'{text}: {x1 - x0:.1f} x {y1 - y0:.1f} mm, clear of holes, windows, arm base and driver')


def draw_template(hs, ws, logos):
    fig = plt.figure(figsize=(297 / 25.4, 210 / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(-148.5, 148.5)                 # 1 data unit = 1 mm on the paper
    ax.set_ylim(-105, 105)
    ax.axis('off')
    oy = 12.0                                  # shift the deck up a little to leave room for notes below
    ax.add_patch(Rectangle((-X, -Y + oy), 2 * X, 2 * Y, fill=False, ec=CUT, lw=1.0, ls=(0, (6, 3))))
    ax.text(-X, Y + oy + 2.5, 'CUT ON THIS LINE  (250 × 138 mm)', color=CUT, fontsize=7, fontweight='bold')
    ax.text(X, Y + oy + 2.5, 'FRONT →', color=INK, fontsize=8, fontweight='bold', ha='right')
    for lab, x, y, d, _ in hs:
        yy = y + oy
        ax.add_patch(Circle((x, yy), d / 2, fill=False, ec=DRILL, lw=0.6))
        ax.plot([x - 3.5, x + 3.5], [yy, yy], color=DRILL, lw=0.35)
        ax.plot([x, x], [yy - 3.5, yy + 3.5], color=DRILL, lw=0.35)
        ax.text(x + 2.6, yy + 1.6, lab, color=DRILL, fontsize=5.5, fontweight='bold')
    for lab, x0, x1, y0, y1, _ in ws:
        ax.add_patch(Rectangle((x0, y0 + oy), x1 - x0, y1 - y0, fill=True, fc='#f6d6d2', ec=CUT, lw=0.6))
        ax.text((x0 + x1) / 2, (y0 + y1) / 2 + oy, lab, color=CUT, fontsize=6, ha='center', va='center', fontweight='bold')
    for text, path, w, h in logos:
        p = Affine2D().translate(0, oy).transform_path(path)
        ax.add_patch(PathPatch(p, fc='#e9d8c4', ec=ENGR, lw=0.4))
    # arm base + driver footprints for orientation (do not cut)
    fx0, fx1, fy0, fy1 = D.ARM_FOOT
    ax.add_patch(Rectangle((D.ARM_X + fx0, fy0 + oy), fx1 - fx0, fy1 - fy0, fill=False, ec='#999999', lw=0.4, ls=':'))
    ax.text(D.ARM_X + (fx0 + fx1) / 2, oy, 'arm base sits here', color='#888888', fontsize=6, ha='center')
    bx, by, _, _ = D.BOARDS['MDD10A']
    L, W = D.BOARD_SIZE['MDD10A']
    ax.add_patch(Rectangle((bx - L / 2, by - W / 2 + oy), L, W, fill=False, ec='#999999', lw=0.4, ls=':'))
    ax.text(bx, by + oy, 'motor driver sits here\n(terminals to the front)', color='#888888', fontsize=6, ha='center', va='center')
    # scale bar + notes
    ax.plot([-140, -40], [-96, -96], color=INK, lw=1.2)
    for x in (-140, -40):
        ax.plot([x, x], [-98, -94], color=INK, lw=1.2)
    ax.text(-90, -93.5, 'must measure exactly 100 mm — print at 100 % / "Actual size"', ha='center', fontsize=6.5)
    ax.text(-140, -103, 'Thenar Walker · 3 mm wooden deck (replaces printed RR-02) · top face up', fontsize=6, color='#555555')
    notes = ['A  Ø3.4 ×4  deck → tub (M3 × 10)', 'B  Ø3.4 ×4  arm base (M3 × 20 + nyloc)', 'C  Ø3.4 ×4  motor driver (M3 × 16 + 6 mm spacer)',
             'D  Ø2.8 ×2  ESP32 standoffs (M2.5)', 'K  kill switch 22.2 × 30.2', 'S1, S2  wire slots 12 × 30']
    for i, n in enumerate(notes):
        ax.text(-20 + (i // 4) * 88, -91 - (i % 4) * 4, n, fontsize=6, color=INK)
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, 'Wood_Deck_Template_A4_1to1.pdf')
    fig.savefig(p)
    fig.savefig(os.path.join(OUT, 'Wood_Deck_Template_preview.png'), dpi=110, facecolor='white')
    plt.close(fig)
    return p


def write_svg(hs, ws, logos):
    """Laser file: 250 x 138 mm, origin at the top-left corner. Red = cut, black = engrave."""
    def tx(x):
        return x + X

    def ty(y):
        return Y - y
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{2 * X}mm" height="{2 * Y}mm" viewBox="0 0 {2 * X} {2 * Y}">',
             f'<rect x="0" y="0" width="{2 * X}" height="{2 * Y}" fill="none" stroke="#ff0000" stroke-width="0.1"/>']
    for _, x, y, d, _ in hs:
        parts.append(f'<circle cx="{tx(x):.3f}" cy="{ty(y):.3f}" r="{d / 2:.3f}" fill="none" stroke="#ff0000" stroke-width="0.1"/>')
    for _, x0, x1, y0, y1, _ in ws:
        parts.append(f'<rect x="{tx(x0):.3f}" y="{ty(y1):.3f}" width="{x1 - x0:.3f}" height="{y1 - y0:.3f}" fill="none" stroke="#ff0000" stroke-width="0.1"/>')
    for text, path, _, _ in logos:
        d = []
        for verts, code in path.iter_segments(simplify=False, curves=True):
            pts = [(tx(verts[i]), ty(verts[i + 1])) for i in range(0, len(verts), 2)]
            cmd = {1: 'M', 2: 'L', 3: 'Q', 4: 'C', 79: 'Z'}[code]
            d.append(cmd + ' '.join(f'{a:.3f},{b:.3f}' for a, b in pts) if cmd != 'Z' else 'Z')
        parts.append(f'<path d="{" ".join(d)}" fill="#000000" stroke="none"><title>{text}</title></path>')
    parts.append('</svg>')
    p = os.path.join(OUT, 'Wood_Deck_Laser.svg')
    open(p, 'w', encoding='utf-8').write('\n'.join(parts))
    return p


def main():
    hs, ws = holes(), windows()
    logos = [(t,) + logo_path(t, cx, cy, cap, rot) for t, cx, cy, cap, rot in LOGOS]
    check(hs, ws, logos)
    pdf = draw_template(hs, ws, logos)
    svg = write_svg(hs, ws, logos)
    json.dump({'size_mm': [2 * X, 2 * Y], 'thickness_mm': 3.0,
               'holes': [{'id': f'{l}{i}', 'x_from_left': round(x + X, 1), 'y_from_front_edge_view_bottom': round(y + Y, 1), 'dia': d, 'use': u}
                         for i, (l, x, y, d, u) in enumerate(hs, 1)],
               'windows': [{'id': l, 'x0': round(x0 + X, 1), 'x1': round(x1 + X, 1), 'y0': round(y0 + Y, 1), 'y1': round(y1 + Y, 1), 'use': u}
                           for l, x0, x1, y0, y1, u in ws],
               'logos': [{'text': t, 'centre_from_left_bottom': [round(cx + X, 1), round(cy + Y, 1)], 'cap_height': c} for t, cx, cy, c, _ in LOGOS]},
              open(os.path.join(OUT, 'wood_deck.json'), 'w'), indent=1)
    print('->', pdf)
    print('->', svg)


if __name__ == '__main__':
    main()
