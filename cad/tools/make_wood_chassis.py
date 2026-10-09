"""All-wood chassis from the team's one 365 x 240 x 3 mm board, joined with L-clamps (L-brackets) + M3 screws.
Replaces the printed tub. Same robot frame as design.py (+X forward, +Y left, mm), same deck top (103 mm),
axle height (70 mm, wheel Ø140) and 154 mm wheelbase as tub v2.

Pieces (5 + 4 pads):
  deck 250 x 138          top plate: arm, motor driver, ESP32 (under), kill switch — holes from docs/wood_deck
  2 x side wall 250 x 50  hang under the deck edges; the 370 motors' M8 collars go through them (nut outside)
  floor 110 x 132         between the walls at the bottom, between the front and rear motors: battery + small boards
  4 x pad 45 x 40         glued inside each wall at a motor: the wall is 6 mm there, the gearbox face sits on it
Joints: 4 L-clamps deck<->walls (x = ±38), 4 L-clamps floor<->walls (x = ±15). Wood glue (Fevicol) optional on
the pads and the floor.

  python cad/tools/make_wood_chassis.py
  -> docs/wood_chassis/1_Cut_Plan.png, 2_Mark_Each_Piece.png, 3_How_It_Goes_Together.png,
     Wood_Chassis_Templates_1to1.pdf (print at 100 %), wood_chassis.json
"""
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages            # noqa: E402
from matplotlib.patches import Circle, Rectangle                 # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection          # noqa: E402

import design as D                                               # noqa: E402

OUT = os.path.join(D.ROOT, 'docs', 'wood_chassis')
DECK_JSON = os.path.join(D.ROOT, 'docs', 'wood_deck', 'wood_deck.json')

# ------------------------------------------------------------------ geometry (mm)
T = 3.0                                    # board thickness
BOARD = (365.0, 240.0)
DECK = (250.0, 138.0)
WALL = (250.0, 50.0)                       # length, height (two come from the 101 mm strip above the deck)
FLOOR = (110.0, DECK[1] - 2 * T)           # 110 x 132: fits between the walls and between the gearboxes
PAD = (45.0, 40.0)
DECK_TOP = 103.0
WALL_TOP = DECK_TOP - T                    # 100
WALL_BOT = WALL_TOP - WALL[1]              # 50 -> ground clearance 50 mm
AXLE_X, AXLE_Z = 77.0, 70.0                # tub v2 numbers
HOLE_FROM_TOP = WALL_TOP - AXLE_Z          # 30
M8_HOLE = 8.6
GB_D, GB_L, CAN_D, CAN_L, CAP_D, CAP_L = 37.5, 20.0, 34.0, 22.0, 30.0, 8.0
COLLAR_L, SHAFT_L, NUT_T = 13.0, 22.0, 6.5
WHEEL_D, WHEEL_W = D.WHEEL_D, D.WHEEL_W
DECK_BRK_X = (-38.0, 38.0)                 # deck <-> wall L-clamps (clear of K, the driver holes and the motors)
FLOOR_BRK_X = (-15.0, 15.0)                # floor <-> wall L-clamps
BRK = 20.0                                 # L-clamp width assumed for the marks (centre the real one on the mark)
BATT = (106.0, 34.0, 26.0)                 # 3S LiPo, lies along X in the middle of the floor
TIE_HOLES = [(sx * 30.0, sy * 21.0) for sx in (-1, 1) for sy in (-1, 1)]   # Ø5 holes for 2 zip ties / velcro
TIE_D = 5.0

WOOD, WOOD2, CUT, DRILL, BRKC, NOTE = '#e9cf9f', '#d9b97f', '#c0392b', '#1f4e8c', '#2e7d32', '#555555'


def deck_features():
    """Deck holes/windows from the wood-deck template, minus the old deck->tub screws (A)."""
    d = json.load(open(DECK_JSON, encoding='utf-8'))
    holes = [h for h in d['holes'] if not h['id'].startswith('A')]
    return holes, d['windows']


def checks():
    """Numbers that must hold for the parts to fit; written to wood_chassis.json and printed."""
    face = DECK[1] / 2 - 2 * T                       # gearbox face |y| (wall + pad)
    motor_tip = face - (GB_L + CAN_L + CAP_L)
    collar_out = COLLAR_L - 2 * T                    # collar length outside the wall
    c = {
        'ground_clearance_wall_bottom': WALL_BOT,
        'ground_clearance_gearbox_bottom': AXLE_Z - GB_D / 2,
        'gearbox_top_below_deck': WALL_TOP - (AXLE_Z + GB_D / 2),
        'gap_between_left_and_right_motor_tips': 2 * motor_tip,
        'floor_end_to_gearbox': AXLE_X - GB_D / 2 - FLOOR[0] / 2,
        'collar_left_for_the_nut': collar_out,
        'nut_fits': collar_out >= NUT_T,
        'wheel_inner_face_y': DECK[1] / 2 + collar_out,
        'overall_width': 2 * (DECK[1] / 2 + collar_out + WHEEL_W),
        'battery_to_motor': AXLE_X - GB_D / 2 - BATT[0] / 2,
        'wood_used': f'{365} x 240 board: deck + 2 walls from the left 250 mm, floor + 4 pads from the right 114 mm',
    }
    assert c['nut_fits'] and c['floor_end_to_gearbox'] > 1 and c['gap_between_left_and_right_motor_tips'] > 10
    assert c['battery_to_motor'] > 1 and c['ground_clearance_wall_bottom'] >= 50
    return c


# ------------------------------------------------------------------ drawing helpers (piece coordinates in mm)
def rect(ax, x, y, w, h, **kw):
    ax.add_patch(Rectangle((x, y), w, h, **kw))


def cross(ax, x, y, d, label=None, color=DRILL, fs=8):
    ax.add_patch(Circle((x, y), d / 2, fill=False, ec=color, lw=1.2))
    r = max(d / 2 + 3, 5)
    ax.plot([x - r, x + r], [y, y], color=color, lw=.6)
    ax.plot([x, x], [y - r, y + r], color=color, lw=.6)
    if label:
        ax.text(x + d / 2 + 1.5, y + d / 2 + 1.5, label, color=color, fontsize=fs, fontweight='bold')


def bracket_mark(ax, x0, y0, w, h, fs=7):
    rect(ax, x0, y0, w, h, fill=True, fc='#c8e6c9', ec=BRKC, lw=1.2, ls='--')
    ax.text(x0 + w / 2, y0 + h / 2, 'L', ha='center', va='center', color=BRKC, fontsize=fs + 2, fontweight='bold')


def draw_deck(ax, fs=8):
    w, h = DECK
    rect(ax, 0, 0, w, h, fc=WOOD, ec='k', lw=1.5)
    holes, wins = deck_features()
    for win in wins:
        rect(ax, win['x0'], win['y0'], win['x1'] - win['x0'], win['y1'] - win['y0'], fc='#f3c4bd', ec=CUT, lw=1.3)
        ax.text((win['x0'] + win['x1']) / 2, (win['y0'] + win['y1']) / 2, win['id'], ha='center', va='center',
                color=CUT, fontsize=fs + 1, fontweight='bold')
    for hl in holes:
        cross(ax, hl['x_from_left'], hl['y_from_front_edge_view_bottom'], hl['dia'], hl['id'][0], fs=fs)
    for x in DECK_BRK_X:                       # L-clamp deck legs along both long edges, on the UNDERSIDE
        u = x + w / 2
        bracket_mark(ax, u - BRK / 2, T, BRK, 22, fs)
        bracket_mark(ax, u - BRK / 2, h - T - 22, BRK, 22, fs)
    ax.text(w - 3, h / 2, 'FRONT →', ha='right', va='center', fontsize=fs + 2, fontweight='bold', rotation=90)


def draw_wall(ax, fs=8):
    """Side wall seen from inside; top edge (y = WALL[1]) goes under the deck. Both walls are identical."""
    L, H = WALL
    rect(ax, 0, 0, L, H, fc=WOOD, ec='k', lw=1.5)
    for x in (-AXLE_X, AXLE_X):
        u = x + L / 2
        rect(ax, u - PAD[0] / 2, 0, PAD[0], PAD[1], fc=WOOD2, ec='#8a6a3a', lw=1, ls=':')
        cross(ax, u, H - HOLE_FROM_TOP, M8_HOLE, 'M8', fs=fs)
    for x in DECK_BRK_X:
        bracket_mark(ax, x + L / 2 - BRK / 2, H - 20, BRK, 20, fs)
    for x in FLOOR_BRK_X:
        bracket_mark(ax, x + L / 2 - BRK / 2, 0, BRK, 20, fs)


def draw_floor(ax, fs=8):
    L, W = FLOOR
    rect(ax, 0, 0, L, W, fc=WOOD, ec='k', lw=1.5)
    rect(ax, L / 2 - BATT[0] / 2, W / 2 - BATT[1] / 2, BATT[0], BATT[1], fill=False, ec=NOTE, lw=1, ls='--')
    ax.text(L / 2, W / 2, 'battery', ha='center', va='center', color=NOTE, fontsize=fs)
    for x, y in TIE_HOLES:
        cross(ax, x + L / 2, y + W / 2, TIE_D, None, fs=fs)
    for x in FLOOR_BRK_X:
        bracket_mark(ax, x + L / 2 - BRK / 2, 0, BRK, 22, fs)
        bracket_mark(ax, x + L / 2 - BRK / 2, W - 22, BRK, 22, fs)


def draw_pad(ax, fs=8):
    rect(ax, 0, 0, PAD[0], PAD[1], fc=WOOD2, ec='k', lw=1.5)
    cross(ax, PAD[0] / 2, PAD[1] - HOLE_FROM_TOP + (WALL[1] - PAD[1]), M8_HOLE, 'M8', fs=fs)


def dim(ax, x0, y0, x1, y1, text, off=6, fs=9, vertical=False):
    if vertical:
        ax.annotate('', xy=(x0 - off, y1), xytext=(x0 - off, y0), arrowprops=dict(arrowstyle='<->', lw=1))
        ax.text(x0 - off - 2, (y0 + y1) / 2, text, ha='right', va='center', fontsize=fs, rotation=90)
    else:
        ax.annotate('', xy=(x1, y0 - off), xytext=(x0, y0 - off), arrowprops=dict(arrowstyle='<->', lw=1))
        ax.text((x0 + x1) / 2, y0 - off - 1.5, text, ha='center', va='top', fontsize=fs)


# ------------------------------------------------------------------ 1. cut plan on the board
def cut_plan(path):
    bw, bh = BOARD
    fig, ax = plt.subplots(figsize=(15, 10.5), dpi=120)
    rect(ax, 0, 0, bw, bh, fc='#cdb184', ec='#6b4f25', lw=2)
    k = 1.0                                                    # saw kerf
    pieces = [('DECK\n250 × 138', 0, 0, *DECK),
              ('SIDE WALL 1\n250 × 50', 0, DECK[1] + k, *WALL),
              ('SIDE WALL 2\n250 × 50', 0, DECK[1] + WALL[1] + 2 * k, WALL[0], bh - DECK[1] - WALL[1] - 2 * k),
              ('FLOOR\n110 × 132', DECK[0] + k, 0, FLOOR[0], FLOOR[1])]
    for i, (px, py) in enumerate([(0, 0), (1, 0), (0, 1), (1, 1)]):
        pieces.append((f'PAD {i + 1}\n45 × 40', DECK[0] + k + px * (PAD[0] + k), FLOOR[1] + k + py * (PAD[1] + k), *PAD))
    for name, x, y, w, h in pieces:
        rect(ax, x, y, w, h, fc=WOOD, ec='#6b4f25', lw=1)
        ax.text(x + w / 2, y + h / 2, name, ha='center', va='center', fontsize=12 if w > 60 else 8.5, fontweight='bold')
    cuts = [((DECK[0], 0), (DECK[0], bh), '1'),
            ((0, DECK[1]), (DECK[0], DECK[1]), '2'),
            ((0, DECK[1] + WALL[1] + k), (DECK[0], DECK[1] + WALL[1] + k), '3'),
            ((DECK[0], FLOOR[1]), (bw, FLOOR[1]), '4'),
            ((DECK[0] + FLOOR[0] + k, 0), (DECK[0] + FLOOR[0] + k, FLOOR[1]), '5'),
            ((DECK[0], FLOOR[1] + 2 * PAD[1] + 2 * k), (bw, FLOOR[1] + 2 * PAD[1] + 2 * k), '6'),
            ((DECK[0] + 2 * PAD[0] + 2 * k, FLOOR[1]), (DECK[0] + 2 * PAD[0] + 2 * k, FLOOR[1] + 2 * PAD[1] + 2 * k), '7'),
            ((DECK[0], FLOOR[1] + PAD[1] + k), (DECK[0] + 2 * PAD[0] + 2 * k, FLOOR[1] + PAD[1] + k), '8'),
            ((DECK[0] + PAD[0] + k, FLOOR[1]), (DECK[0] + PAD[0] + k, FLOOR[1] + 2 * PAD[1] + 2 * k), '9')]
    for (x0, y0), (x1, y1), n in cuts:
        ax.plot([x0, x1], [y0, y1], color=CUT, lw=3.2)
        ax.text(x0 + (x1 - x0) * .3, y0 + (y1 - y0) * .3, n, color='white',
                fontsize=10, fontweight='bold', ha='center', va='center',
                bbox=dict(boxstyle='circle,pad=0.25', fc=CUT, ec='none'))
    dim(ax, 0, 0, DECK[0], 0, '250', off=10, fs=11)
    dim(ax, DECK[0], 0, DECK[0] + FLOOR[0], 0, '110', off=10, fs=11)
    dim(ax, 0, 0, 0, DECK[1], '138', off=8, fs=11, vertical=True)
    dim(ax, 0, DECK[1], 0, DECK[1] + WALL[1], '50', off=8, fs=11, vertical=True)
    dim(ax, 0, DECK[1] + WALL[1], 0, bh, '50', off=8, fs=11, vertical=True)
    dim(ax, bw + 14, 0, bw, FLOOR[1], '132', off=0, fs=11, vertical=True)
    steps = ('Saw in this order (red numbers). Clamp the board with the cut line just past the table edge.\n'
             '1  full height, 250 from the left      2  138 up (deck done)      3  50 above cut 2 (two walls)\n'
             '4  132 up on the right piece      5  110 right of cut 1 (floor)\n'
             '6  82 above cut 4   7  92 right of cut 1   8  40 above cut 4   9  46 right of cut 1  (four 45 × 40 pads)\n'
             'Kerf ~1 mm per cut is already allowed for. Spare: the strip right of the floor and above the pads.')
    ax.text(0, -24, steps, fontsize=11.5, va='top', family='monospace')
    ax.set_xlim(-24, bw + 20)
    ax.set_ylim(-70, bh + 8)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title('1 — What to cut: your 365 × 240 × 3 mm board → deck, 2 side walls, floor, 4 motor pads',
                 fontsize=15, fontweight='bold')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


# ------------------------------------------------------------------ 2. what to mark on each piece
def marks(path):
    fig = plt.figure(figsize=(16, 12.5), dpi=120)
    gs = fig.add_gridspec(3, 3, height_ratios=[1.35, .62, 1.0], width_ratios=[1.1, .45, .55], hspace=0.28)
    a = fig.add_subplot(gs[0, :])
    draw_deck(a, fs=9)
    a.set_title('DECK 250 × 138 (top view, FRONT to the right)   blue = drill   red = cut out   green L = L-clamp under the deck',
                fontsize=12, fontweight='bold', loc='left')
    dim(a, 0, 0, DECK[0], 0, '250', off=6)
    dim(a, 0, 0, 0, DECK[1], '138', off=5, vertical=True)
    b = fig.add_subplot(gs[1, :])
    draw_wall(b, fs=9)
    L, H = WALL
    for x in (-AXLE_X, AXLE_X):
        dim(b, 0, 0, x + L / 2, 0, f'{x + L / 2:.0f}', off=5 if x < 0 else 11)
    b.annotate('', xy=(L + 6, H), xytext=(L + 6, H - HOLE_FROM_TOP), arrowprops=dict(arrowstyle='<->', lw=1))
    b.text(L + 8, H - HOLE_FROM_TOP / 2, f'{HOLE_FROM_TOP:.0f} from\nTOP edge', fontsize=9, va='center')
    b.set_title('2 × SIDE WALL 250 × 50 (identical)   M8 holes Ø8.5 at 48 and 202 from the end, 30 from the top   '
                'dotted = where the pad is glued (inside)', fontsize=12, fontweight='bold', loc='left')
    c = fig.add_subplot(gs[2, 0])
    draw_floor(c, fs=9)
    dim(c, 0, 0, FLOOR[0], 0, '110', off=6)
    dim(c, 0, 0, 0, FLOOR[1], '132', off=5, vertical=True)
    c.set_title('FLOOR 110 × 132   Ø5 = zip-tie holes (30 / 21 from the centre)', fontsize=11.5, fontweight='bold', loc='left')
    d = fig.add_subplot(gs[2, 1])
    draw_pad(d, fs=9)
    dim(d, 0, 0, PAD[0], 0, '45', off=5)
    dim(d, 0, 0, 0, PAD[1], '40', off=4, vertical=True)
    d.set_title('4 × PAD 45 × 40\nhole 20 up, centred', fontsize=11.5, fontweight='bold', loc='left')
    e = fig.add_subplot(gs[2, 2])
    e.axis('off')
    e.text(0, 1, 'DRILL SIZES\n'
           '  B  arm base           Ø3.4 ×4\n'
           '  C  motor driver       Ø3.4 ×4\n'
           '  D  ESP32 standoffs    Ø2.8 ×2\n'
           '  M8 motor collar       Ø8.5 ×4\n'
           '     (walls + pads, drill\n'
           '      together after gluing)\n'
           '  zip ties              Ø5   ×4\n'
           'CUT-OUTS\n'
           '  K   kill switch    22.2 × 30.2\n'
           '  S1, S2 wire slots   12 × 30\n'
           'L-CLAMPS: 8 (4 deck, 4 floor)\n'
           '  centre the clamp on the mark,\n'
           '  pencil through its holes, drill\n'
           '  Ø3.4, M3 screw + nut.\n'
           '  Wood screws do NOT hold in 3 mm.',
           va='top', family='monospace', fontsize=10.5)
    for ax, (w, h) in ((a, DECK), (b, WALL), (c, FLOOR), (d, PAD)):
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_xlim(-16, w + 30)
        ax.set_ylim(-20, h + 6)
    fig.suptitle('2 — What to mark on each piece (print the 1:1 PDF and tape it on — don\'t measure off this picture)',
                 fontsize=15, fontweight='bold')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


# ------------------------------------------------------------------ 3. assembly picture (3D)
def box3(x0, x1, y0, y1, z0, z1):
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return [[v[i] for i in q] for q in f]


def cyl_y(x, z, r, y0, y1, n=28):
    ring = [(x + r * math.cos(2 * math.pi * k / n), z + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    side = [[(ring[k][0], y0, ring[k][1]), (ring[(k + 1) % n][0], y0, ring[(k + 1) % n][1]),
             (ring[(k + 1) % n][0], y1, ring[(k + 1) % n][1]), (ring[k][0], y1, ring[k][1])] for k in range(n)]
    return side + [[(p[0], y0, p[1]) for p in ring], [(p[0], y1, p[1]) for p in ring]]


def section(ax):
    """Cross-section through the front axle, looking from the front: what touches what, with heights."""
    hy, co = DECK[1] / 2, COLLAR_L - 2 * T
    ax.add_patch(Rectangle((-150, -4), 300, 4, fc='#bdbdbd', ec='none'))
    ax.text(0, -2, 'ground', ha='center', va='center', fontsize=8)
    rect(ax, -hy, WALL_TOP, DECK[1], T, fc=WOOD, ec='k', lw=1)
    ax.text(0, DECK_TOP + 3, 'DECK (arm, driver, kill switch on top)', ha='center', fontsize=9)
    for s in (-1, 1):
        rect(ax, min(s * hy, s * (hy - T)), WALL_BOT, T, WALL[1], fc=WOOD2, ec='k', lw=1)
        rect(ax, min(s * (hy - T), s * (hy - 2 * T)), WALL_BOT, T, PAD[1], fc='#c79a5a', ec='k', lw=.8)
        face = s * (hy - 2 * T)
        rect(ax, min(face, face - s * GB_L), AXLE_Z - GB_D / 2, GB_L, GB_D, fc='#f0c419', ec='k', lw=.8)
        rect(ax, min(face - s * GB_L, face - s * (GB_L + CAN_L)), AXLE_Z - CAN_D / 2, CAN_L, CAN_D, fc='#9e9e9e', ec='k', lw=.8)
        rect(ax, min(face - s * (GB_L + CAN_L), face - s * (GB_L + CAN_L + CAP_L)), AXLE_Z - CAP_D / 2, CAP_L, CAP_D, fc='#bdbdbd', ec='k', lw=.8)
        rect(ax, min(s * hy, s * (hy + NUT_T)), AXLE_Z - 6.5, NUT_T, 13, fc='#607d8b', ec='k', lw=.8)
        rect(ax, min(face, s * (hy + co)), AXLE_Z - 4, abs(s * (hy + co) - face), 8, fc='#90a4ae', ec='k', lw=.6)
        rect(ax, min(s * (hy + co), s * (hy + co + SHAFT_L)), AXLE_Z - 3, SHAFT_L, 6, fc='#cfd8dc', ec='k', lw=.6)
        yi = s * (hy + co)
        rect(ax, min(yi, yi + s * WHEEL_W), 0, WHEEL_W, WHEEL_D, fc='#37474f', ec='k', lw=.8, alpha=.25)
        # L-clamps
        ax.plot([s * (hy - T), s * (hy - T - 20)], [WALL_TOP - 1, WALL_TOP - 1], color=BRKC, lw=3)
        ax.plot([s * (hy - T - 1), s * (hy - T - 1)], [WALL_TOP, WALL_TOP - 20], color=BRKC, lw=3)
        ax.plot([s * (hy - T), s * (hy - T - 20)], [WALL_BOT + T + 1, WALL_BOT + T + 1], color=BRKC, lw=3)
        ax.plot([s * (hy - T - 1), s * (hy - T - 1)], [WALL_BOT + T, WALL_BOT + T + 20], color=BRKC, lw=3)
    rect(ax, -hy + T, WALL_BOT, FLOOR[1], T, fc=WOOD, ec='k', lw=1, ls='--')
    ax.annotate('floor (between the motors, behind this cut)', xy=(0, WALL_BOT + 1.5), xytext=(0, 4), fontsize=8.5, ha='center', fontweight='bold', arrowprops=dict(arrowstyle='->', lw=.8))
    notes = [((-(hy - T / 2), 57), (-118, 40), 'side wall 3 mm'), ((-(hy - 1.5 * T), 62), (-70, 30), 'pad (glued)'),
             ((-(hy - 2 * T - GB_L / 2), 60), (-38, 18), 'gearbox'), ((-(hy - 2 * T - GB_L - CAN_L / 2), 58), (-5, 32), 'motor'),
             ((hy + NUT_T / 2, AXLE_Z - 6), (40, 18), 'M8 nut outside'), ((hy + co + 4, AXLE_Z - 3), (88, 32), 'shaft → wheel'),
             ((hy - T - 10, WALL_TOP - 1), (60, 120), 'L-clamps (they sit between\nthe motors, drawn here so\nyou see where they go)')]
    for xy, xyt, t in notes:
        ax.annotate(t, xy=xy, xytext=xyt, fontsize=8.5, fontweight='bold', ha='center',
                    arrowprops=dict(arrowstyle='->', lw=.8, color='#333333'))
    for z, t in ((DECK_TOP, f'{DECK_TOP:.0f} deck top'), (AXLE_Z, f'{AXLE_Z:.0f} axle'), (WALL_BOT, f'{WALL_BOT:.0f} wall bottom')):
        ax.plot([-150, -112], [z, z], color=NOTE, lw=.7, ls=':')
        ax.text(-150, z + 1.5, t, fontsize=8.5, color=NOTE)
    ax.set_xlim(-152, 152); ax.set_ylim(-6, 150)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_title('Cut through the front axle, seen from the front (mm above the ground)', fontsize=12)


def assembly(path):
    hy = DECK[1] / 2
    collar_out = COLLAR_L - 2 * T
    parts = []                                    # (faces, colour, alpha)
    parts.append((box3(-DECK[0] / 2, DECK[0] / 2, -hy, hy, WALL_TOP, DECK_TOP), WOOD, .55))
    for s in (-1, 1):
        parts.append((box3(-WALL[0] / 2, WALL[0] / 2, s * (hy - T), s * hy, WALL_BOT, WALL_TOP), WOOD2, .9))
        for x in (-AXLE_X, AXLE_X):
            parts.append((box3(x - PAD[0] / 2, x + PAD[0] / 2, s * (hy - 2 * T), s * (hy - T), WALL_BOT, WALL_BOT + PAD[1]), '#c79a5a', .95))
            face = s * (hy - 2 * T)
            parts.append((cyl_y(x, AXLE_Z, GB_D / 2, face, face - s * GB_L), '#f0c419', .95))
            parts.append((cyl_y(x, AXLE_Z, CAN_D / 2, face - s * GB_L, face - s * (GB_L + CAN_L)), '#9e9e9e', .95))
            parts.append((cyl_y(x, AXLE_Z, NUT_T, s * hy, s * (hy + NUT_T)), '#607d8b', 1))
            yi = s * (hy + collar_out)
            parts.append((cyl_y(x, AXLE_Z, WHEEL_D / 2, yi, yi + s * WHEEL_W), '#37474f', .12))
        for x in DECK_BRK_X:
            parts.append((box3(x - BRK / 2, x + BRK / 2, s * (hy - T - 20), s * (hy - T), WALL_TOP - 2, WALL_TOP), '#43a047', 1))
            parts.append((box3(x - BRK / 2, x + BRK / 2, s * (hy - T - 2), s * (hy - T), WALL_TOP - 20, WALL_TOP), '#43a047', 1))
        for x in FLOOR_BRK_X:
            parts.append((box3(x - BRK / 2, x + BRK / 2, s * (hy - T - 20), s * (hy - T), WALL_BOT + T, WALL_BOT + T + 2), '#43a047', 1))
            parts.append((box3(x - BRK / 2, x + BRK / 2, s * (hy - T - 2), s * (hy - T), WALL_BOT + T, WALL_BOT + T + 20), '#43a047', 1))
    parts.append((box3(-FLOOR[0] / 2, FLOOR[0] / 2, -hy + T, hy - T, WALL_BOT, WALL_BOT + T), WOOD, .95))
    parts.append((box3(-BATT[0] / 2, BATT[0] / 2, -BATT[1] / 2, BATT[1] / 2, WALL_BOT + T, WALL_BOT + T + BATT[2]), '#1e88e5', .9))

    fig = plt.figure(figsize=(17, 8.8), dpi=120)
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    for faces, col, al in parts:
        ax.add_collection3d(Poly3DCollection(faces, fc=col, ec='#333333', lw=.15, alpha=al))
    ax.set_xlim(-135, 135); ax.set_ylim(-135, 135); ax.set_zlim(0, 150)
    ax.set_box_aspect((270, 270, 150))
    ax.view_init(elev=24, azim=-60)
    ax.set_axis_off()
    ax.set_title('3D (wheels see-through), front is to the right', fontsize=12)
    section(fig.add_subplot(1, 2, 2))
    txt = ('Build order:  1 glue the 4 pads inside the walls at the dotted marks (clamp, let it dry)  ·  2 drill Ø8.5 through wall + pad  ·  '
           '3 motor: gearbox inside, M8 collar out, nut outside, tighten hard\n'
           '4 L-clamps (green) on the deck underside, then screw the walls to them  ·  5 L-clamps on the floor, slide it in between the walls at the '
           'bottom, screw  ·  6 battery on the floor (zip ties), wheels on the shafts\n'
           'Yellow = gearbox (Ø37.5), grey = motor can, dark = wheel Ø140, blue = battery. Wall bottom = 50 mm off the ground.')
    fig.text(0.02, 0.02, txt, fontsize=10.5, va='bottom')
    fig.suptitle('3 — How it goes together: deck on two side walls, floor between them, motors through the walls',
                 fontsize=15, fontweight='bold')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


# ------------------------------------------------------------------ 4. 1:1 templates (A4 landscape, 100 %)
def a4_page(pdf, title, items):
    """items: (draw_fn, x_mm, y_mm, label). Axes span the whole page in mm, so 1 unit = 1 mm when printed at 100 %."""
    fig = plt.figure(figsize=(297 / 25.4, 210 / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, 297); ax.set_ylim(0, 210)
    ax.axis('off')
    for fn, x, y, label in items:
        sub = fig.add_axes([0, 0, 1, 1], label=f'{label}{x}{y}')
        sub.set_xlim(-x, 297 - x); sub.set_ylim(-y, 210 - y)
        sub.axis('off')
        sub.patch.set_alpha(0)
        fn(sub, fs=6.5)
        ax.text(x, y - 4, label, fontsize=7.5, va='top', color=NOTE)
    ax.text(12, 203, title, fontsize=10, fontweight='bold', va='top')
    ax.plot([12, 112], [12, 12], color='k', lw=1.2)
    for xx in (12, 112):
        ax.plot([xx, xx], [10, 14], color='k', lw=1.2)
    ax.text(62, 14.5, 'must measure exactly 100 mm — print at 100 % / "Actual size"', ha='center', fontsize=7)
    pdf.savefig(fig)
    plt.close(fig)


def templates(path):
    with PdfPages(path) as pdf:
        a4_page(pdf, 'DECK 250 × 138 — top view, FRONT to the right. Tape on, drill at the crosses, cut the red windows.',
                [(draw_deck, 23, 40, 'deck')])
        a4_page(pdf, '2 × SIDE WALL 250 × 50 (inside view, top edge up). Each pad is glued on the dotted outline.',
                [(draw_wall, 23, 125, 'side wall 1'), (draw_wall, 23, 55, 'side wall 2')])
        a4_page(pdf, 'FLOOR 110 × 132 + 4 pads 45 × 40 (drill the M8 hole after gluing, through pad + wall)',
                [(draw_floor, 25, 40, 'floor'), (draw_pad, 160, 120, 'pad 1'), (draw_pad, 220, 120, 'pad 2'),
                 (draw_pad, 160, 40, 'pad 3'), (draw_pad, 220, 40, 'pad 4')])


def main():
    os.makedirs(OUT, exist_ok=True)
    c = checks()
    cut_plan(os.path.join(OUT, '1_Cut_Plan.png'))
    marks(os.path.join(OUT, '2_Mark_Each_Piece.png'))
    assembly(os.path.join(OUT, '3_How_It_Goes_Together.png'))
    templates(os.path.join(OUT, 'Wood_Chassis_Templates_1to1.pdf'))
    holes, wins = deck_features()
    json.dump({'checks': c, 'pieces': {'deck': DECK, 'side_wall': WALL, 'floor': FLOOR, 'pad': PAD},
               'deck_holes': holes, 'deck_windows': wins,
               'deck_brackets_x': DECK_BRK_X, 'floor_brackets_x': FLOOR_BRK_X, 'tie_holes': TIE_HOLES,
               'wall_m8_holes_from_end': [WALL[0] / 2 - AXLE_X, WALL[0] / 2 + AXLE_X], 'wall_m8_from_top': HOLE_FROM_TOP},
              open(os.path.join(OUT, 'wood_chassis.json'), 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    for k, v in c.items():
        print(f'{k:40s} {v}')


if __name__ == '__main__':
    main()
