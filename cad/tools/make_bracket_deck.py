"""Simplest chassis: ONE wooden deck 250 x 138, the four 370 motors hang under it on steel L-brackets
(the team's motor clamp: M8 collar through the bracket's face, 13 mm nut, base screwed to the board).

Width is the hard limit: the start box is 300 L x 200 W x 300 H including the wheels. The wheel sits on the
shaft just past the collar, so the bracket face has to be INSET from the board edge:
  gearbox face |y| = 57  ->  collar end 70  ->  wheel 71..99  ->  overall 198 mm.
The rear axle is 4 mm further forward (x = -73) so the rear bracket bases clear the arm's rear screws (B).

  python cad/tools/make_bracket_deck.py
  -> docs/wood_chassis/4_Bracket_Deck.png, Bracket_Deck_Template_1to1.pdf, bracket_deck.json
"""
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages            # noqa: E402
from matplotlib.patches import Rectangle                         # noqa: E402

import make_wood_chassis as W                                    # noqa: E402  (drawing helpers, deck holes)

DECK = W.DECK                       # 250 x 138
HY = DECK[1] / 2                    # 69
AXLES = (-73.0, 77.0)               # rear, front (front unchanged: forward tipping margin as designed)
FACE_Y = 57.0                       # gearbox face / bracket inner face |y|
BRK_T = 2.0                         # bracket steel
BRK_W, BRK_BASE = 45.0, 30.0        # bracket width along X, base depth inward (assumed: measure yours)
AXIS_H = 21.0                       # board underside -> shaft centre (assumed: bracket base + gearbox radius)
GB_D, MOTOR_L = W.GB_D, W.GB_L + W.CAN_L + W.CAP_L      # 37.5, 50 (can length estimated)
COLLAR_L, SHAFT_L, NUT_T = W.COLLAR_L, W.SHAFT_L, W.NUT_T
WHEEL_D, WHEEL_W = W.WHEEL_D, W.WHEEL_W
AXLE_Z = WHEEL_D / 2
DECK_BOT = AXLE_Z + AXIS_H
BOX = (300.0, 200.0)


def numbers():
    collar_end = FACE_Y + COLLAR_L
    wheel_in, wheel_out = collar_end + 1, collar_end + 1 + WHEEL_W
    holes, _ = W.deck_features()
    arm_rear = [h for h in holes if h['id'][0] == 'B' and h['x_from_left'] < 50]
    rear_base_x0 = AXLES[0] - BRK_W / 2 + DECK[0] / 2         # view x of the rear bracket's rear edge
    n = {
        'overall_width': 2 * wheel_out,
        'overall_length': (AXLES[1] + WHEEL_D / 2) - (AXLES[0] - WHEEL_D / 2),
        'wheelbase': AXLES[1] - AXLES[0],
        'bracket_face_in_from_board_edge': HY - FACE_Y,
        'gap_between_motor_tips': 2 * (FACE_Y - MOTOR_L),
        'nut_to_wheel': wheel_in - (FACE_Y + BRK_T + NUT_T),
        'deck_underside_z': DECK_BOT,
        'deck_top_z': DECK_BOT + W.T,
        'ground_clearance_gearbox': AXLE_Z - GB_D / 2,
        'room_under_deck_for_electronics_mm': DECK_BOT - 50.0,
        'electronics_zone_x': (AXLES[0] + BRK_W / 2, AXLES[1] - BRK_W / 2),
        'rear_bracket_to_arm_screw_B': rear_base_x0 - max(h['x_from_left'] for h in arm_rear),
    }
    assert n['overall_width'] <= BOX[1] and n['overall_length'] <= BOX[0]
    assert n['gap_between_motor_tips'] > 5 and n['nut_to_wheel'] >= 0 and n['rear_bracket_to_arm_screw_B'] > 3
    return n


def draw_deck(ax, fs=8, full=True):
    """Top view, FRONT to the right. Brackets are UNDER the deck: dashed blue."""
    w, h = DECK
    W.rect(ax, 0, 0, w, h, fc=W.WOOD, ec='k', lw=1.5)
    holes, wins = W.deck_features()
    for win in wins:
        if win['id'] == 'S2':                       # moved 6 mm forward: clear of the front bracket base
            win = dict(win, x0=win['x0'] + 6, x1=win['x1'] + 6)
        W.rect(ax, win['x0'], win['y0'], win['x1'] - win['x0'], win['y1'] - win['y0'], fc='#f3c4bd', ec=W.CUT, lw=1.3)
        ax.text((win['x0'] + win['x1']) / 2, (win['y0'] + win['y1']) / 2, win['id'], ha='center', va='center',
                color=W.CUT, fontsize=fs + 1, fontweight='bold')
    for hl in holes:
        if hl['id'][0] != 'D':                      # ESP32 moves to the electronics zone (mark through its own holes)
            W.cross(ax, hl['x_from_left'], hl['y_from_front_edge_view_bottom'], hl['dia'], hl['id'][0], fs=fs)
    for x in AXLES:
        u = x + w / 2
        for s in (-1, 1):
            face = HY + s * FACE_Y                                    # view y of the bracket's inner face
            base0 = face - s * BRK_BASE
            W.rect(ax, u - BRK_W / 2, min(face, base0), BRK_W, BRK_BASE, fill=False, ec=W.DRILL, lw=1.2, ls='--')
            W.rect(ax, u - BRK_W / 2, min(face, face + s * BRK_T), BRK_W, BRK_T, fc=W.DRILL, ec=W.DRILL, lw=0)
            mo = face - s * MOTOR_L
            W.rect(ax, u - GB_D / 2, min(face, mo), GB_D, MOTOR_L, fill=False, ec='#9e9e9e', lw=.8, ls=':')
            ax.plot([u, u], [face, face + s * 8], color=W.DRILL, lw=.8)
            if full:
                ax.text(u, base0 - s * 3, 'motor bracket\n(under)', ha='center', va='center', fontsize=fs - 1,
                        color=W.DRILL)
    z0, z1 = AXLES[0] + BRK_W / 2 + w / 2, AXLES[1] - BRK_W / 2 + w / 2
    if full:
        ax.text((z0 + z1) / 2, h / 2 - 22, 'under the deck: battery, buck converters,\nPCA9685, ESP32', ha='center',
                va='center', fontsize=fs, color=W.NOTE, style='italic')
    ax.text(w - 3, h / 2, 'FRONT →', ha='right', va='center', fontsize=fs + 2, fontweight='bold', rotation=90)


def section(ax):
    """Front view through the front axle with the 200 mm start-box width."""
    ax.add_patch(Rectangle((-120, -4), 240, 4, fc='#bdbdbd', ec='none'))
    W.rect(ax, -HY, DECK_BOT, DECK[1], W.T, fc=W.WOOD, ec='k', lw=1)
    for s in (-1, 1):
        f = s * FACE_Y
        W.rect(ax, min(f, f + s * BRK_T), AXLE_Z - 20, BRK_T, DECK_BOT - (AXLE_Z - 20), fc=W.DRILL, ec='k', lw=.6)
        W.rect(ax, min(f, f - s * BRK_BASE), DECK_BOT - BRK_T, BRK_BASE, BRK_T, fc=W.DRILL, ec='k', lw=.6)
        W.rect(ax, min(f, f - s * W.GB_L), AXLE_Z - GB_D / 2, W.GB_L, GB_D, fc='#f0c419', ec='k', lw=.8)
        W.rect(ax, min(f - s * W.GB_L, f - s * MOTOR_L), AXLE_Z - W.CAN_D / 2, MOTOR_L - W.GB_L, W.CAN_D, fc='#9e9e9e', ec='k', lw=.8)
        W.rect(ax, min(f + s * BRK_T, f + s * (BRK_T + NUT_T)), AXLE_Z - 6.5, NUT_T, 13, fc='#607d8b', ec='k', lw=.8)
        W.rect(ax, min(f, f + s * COLLAR_L), AXLE_Z - 4, COLLAR_L, 8, fc='#90a4ae', ec='k', lw=.6)
        ce = f + s * COLLAR_L
        W.rect(ax, min(ce, ce + s * SHAFT_L), AXLE_Z - 3, SHAFT_L, 6, fc='#cfd8dc', ec='k', lw=.6)
        W.rect(ax, min(ce + s, ce + s * (1 + WHEEL_W)), 0, WHEEL_W, WHEEL_D, fc='#37474f', ec='k', lw=.8, alpha=.25)
    ax.plot([-BOX[1] / 2] * 2, [0, 150], color=W.CUT, lw=1.2, ls='--')
    ax.plot([BOX[1] / 2] * 2, [0, 150], color=W.CUT, lw=1.2, ls='--')
    ax.text(0, 152, f'start box 200 wide — robot {2 * (FACE_Y + COLLAR_L + 1 + WHEEL_W):.0f}', ha='center',
            color=W.CUT, fontsize=10, fontweight='bold')
    ax.annotate('', xy=(HY, DECK_BOT + 12), xytext=(FACE_Y, DECK_BOT + 12), arrowprops=dict(arrowstyle='<->', lw=1))
    ax.text((HY + FACE_Y) / 2, DECK_BOT + 15, f'{HY - FACE_Y:.0f}', ha='center', fontsize=10, fontweight='bold')
    notes = [((-FACE_Y - 1, AXLE_Z + 15), (-100, 118), 'bracket face 12 mm\nin from the board edge'),
             ((-FACE_Y + 10, AXLE_Z - 10), (-40, 30), 'gearbox'), ((FACE_Y + 5, AXLE_Z - 6), (40, 22), 'M8 nut'),
             ((FACE_Y + COLLAR_L + 15, 30), (105, 22), 'wheel'), ((0, DECK_BOT + W.T), (40, 118), 'deck 3 mm')]
    for xy, xyt, t in notes:
        ax.annotate(t, xy=xy, xytext=xyt, fontsize=9, fontweight='bold', ha='center',
                    arrowprops=dict(arrowstyle='->', lw=.8))
    for z, t in ((DECK_BOT + W.T, f'{DECK_BOT + W.T:.0f} deck top'), (AXLE_Z, f'{AXLE_Z:.0f} axle'),
                 (AXLE_Z - GB_D / 2, f'{AXLE_Z - GB_D / 2:.0f} lowest point')):
        ax.plot([-120, -78], [z, z], color=W.NOTE, lw=.7, ls=':')
        ax.text(-120, z + 1.5, t, fontsize=8.5, color=W.NOTE)
    ax.set_xlim(-122, 122); ax.set_ylim(-6, 160)
    ax.set_aspect('equal'); ax.axis('off')
    ax.set_title('Seen from the front (mm). Wheels must stay inside the red lines.', fontsize=12)


def picture(path, n):
    fig = plt.figure(figsize=(17, 8.6), dpi=120)
    a = fig.add_axes([0.01, 0.12, 0.56, 0.78])
    draw_deck(a, fs=9)
    w, h = DECK
    W.dim(a, 0, 0, w, 0, '250', off=7)
    W.dim(a, 0, 0, 0, h, '138', off=6, vertical=True)
    for x, lab in zip(AXLES, ('rear axle', 'front axle')):
        u = x + w / 2
        a.annotate('', xy=(u, h + 6), xytext=(0, h + 6), arrowprops=dict(arrowstyle='<->', lw=1))
        a.text(u / 2 if x < 0 else u - 30, h + 8, f'{u:.0f} to the {lab}', fontsize=9.5, fontweight='bold', ha='center')
    a.set_xlim(-16, w + 8); a.set_ylim(-20, h + 18)
    a.set_aspect('equal'); a.axis('off')
    a.set_title('DECK 250 × 138 — top view. Dashed blue = motor bracket UNDER the deck (solid bar = its face).',
                fontsize=12, fontweight='bold', loc='left')
    section(fig.add_axes([0.58, 0.12, 0.41, 0.78]))
    txt = ('1  Mark the bracket spots: face line 12 mm in from the long edge, centred 52 mm (rear) and 202 mm (front) from the REAR end.   '
           '2  Hold each bracket there, pencil through its base holes, drill, M3/M4 screw + nut.\n'
           '3  Drill B (arm), C (driver), cut K (kill switch), S1, S2 from the 1:1 PDF.   '
           '4  Battery, bucks, PCA9685, ESP32 go UNDER the deck between the front and rear brackets '
           f'({n["room_under_deck_for_electronics_mm"]:.0f} mm of height before the 50 mm clearance line).')
    fig.text(0.01, 0.02, txt, fontsize=10.5, va='bottom')
    fig.suptitle(f'4 — One board, motors on L-brackets underneath: robot {n["overall_length"]:.0f} × {n["overall_width"]:.0f} mm '
                 '(start box 300 × 200)', fontsize=15, fontweight='bold')
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)


def template(path):
    with PdfPages(path) as pdf:
        fig = plt.figure(figsize=(297 / 25.4, 210 / 25.4))
        ax = fig.add_axes([0, 0, 1, 1])
        ax.set_xlim(-23, 274); ax.set_ylim(-40, 170)
        ax.axis('off')
        draw_deck(ax, fs=6.5, full=False)
        ax.text(-11, 163, 'DECK 250 × 138, top view, FRONT to the right. Blue bars = motor bracket face (under the deck). '
                'Drill B Ø3.4, C Ø3.4; cut K 22.2 × 30.2, S1/S2 12 × 30.', fontsize=8, fontweight='bold', va='top')
        ax.plot([-11, 89], [-28, -28], color='k', lw=1.2)
        for xx in (-11, 89):
            ax.plot([xx, xx], [-30, -26], color='k', lw=1.2)
        ax.text(39, -25.5, 'must measure exactly 100 mm — print at 100 % / "Actual size"', ha='center', fontsize=7)
        pdf.savefig(fig)
        plt.close(fig)


def main():
    os.makedirs(W.OUT, exist_ok=True)
    n = numbers()
    picture(os.path.join(W.OUT, '4_Bracket_Deck.png'), n)
    template(os.path.join(W.OUT, 'Bracket_Deck_Template_1to1.pdf'))
    json.dump({'numbers': n, 'axles_x': AXLES, 'bracket_face_y': FACE_Y,
               'assumed': {'bracket_width': BRK_W, 'bracket_base_depth': BRK_BASE, 'axis_below_board': AXIS_H,
                           'motor_length_from_face': MOTOR_L}},
              open(os.path.join(W.OUT, 'bracket_deck.json'), 'w', encoding='utf-8'), indent=1)
    for k, v in n.items():
        print(f'{k:36s} {v}')


if __name__ == '__main__':
    main()
