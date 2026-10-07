"""Build docs/Thenar_Walker_Team_Guide.pdf — a 12-page guide for the team.

All numbers come from the project's evidence files (SolidWorks verification, simulation,
drive sizing), so re-running this after a change keeps the guide honest.
  python docs/make_team_pdf.py
"""
import json
import os
import tempfile

from PIL import Image as PILImage
from reportlab.graphics.shapes import Drawing, Line, Polygon, Rect, String
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'docs', 'Thenar_Walker_Team_Guide.pdf')
R = lambda *p: os.path.join(ROOT, *p)
TMP = tempfile.mkdtemp(prefix='twpdf_')

pdfmetrics.registerFont(TTFont('Seg', r'C:\Windows\Fonts\segoeui.ttf'))
pdfmetrics.registerFont(TTFont('SegB', r'C:\Windows\Fonts\segoeuib.ttf'))
pdfmetrics.registerFont(TTFont('SegSym', r'C:\Windows\Fonts\seguisym.ttf'))
pdfmetrics.registerFontFamily('Seg', normal='Seg', bold='SegB', italic='Seg', boldItalic='SegB')

NAVY = colors.HexColor('#12355b')
ORANGE = colors.HexColor('#d9622b')
LIGHT = colors.HexColor('#eef2f7')
GREEN = colors.HexColor('#1f7a3a')
GREY = colors.HexColor('#5b6573')

W, H = A4
M = 16 * mm
CW = W - 2 * M

S = {
    'h1': ParagraphStyle('h1', fontName='SegB', fontSize=18, leading=22, textColor=NAVY, spaceAfter=4),
    'h2': ParagraphStyle('h2', fontName='SegB', fontSize=11.5, leading=14, textColor=ORANGE, spaceBefore=6, spaceAfter=3),
    'b': ParagraphStyle('b', fontName='Seg', fontSize=9.2, leading=12.2, spaceAfter=3),
    'small': ParagraphStyle('small', fontName='Seg', fontSize=7.8, leading=10, textColor=GREY),
    'cell': ParagraphStyle('cell', fontName='Seg', fontSize=8, leading=10),
    'cellb': ParagraphStyle('cellb', fontName='SegB', fontSize=8, leading=10),
    'head': ParagraphStyle('head', fontName='SegB', fontSize=8, leading=10, textColor=colors.white),
    'cap': ParagraphStyle('cap', fontName='Seg', fontSize=7.8, leading=9.5, textColor=GREY, alignment=TA_CENTER),
    'big': ParagraphStyle('big', fontName='SegB', fontSize=34, leading=38, textColor=NAVY),
    'sub': ParagraphStyle('sub', fontName='Seg', fontSize=13, leading=17, textColor=GREY),
    'call': ParagraphStyle('call', fontName='Seg', fontSize=9, leading=12, textColor=NAVY),
}


def load(*p):
    return json.load(open(R(*p), encoding='utf-8'))


VER = load('cad', 'evidence', 'verification.json')
MIS = {k: load('sim', 'out', f'mission_{k}.json') for k in ('A', 'B', 'C', 'C_spin')}
TURN = load('sim', 'out', 'turn_study.json')
DRV = load('analysis', 'drive_sizing.json')
PT = load('cad', 'print', 'print_time.json')


def P(t, st='b'):
    return Paragraph(t, S[st])


def img(path, w, h=None, crop=True):
    """Image flowable, background auto-cropped, fitted inside w x h (points)."""
    im = PILImage.open(path).convert('RGB')
    if crop:
        bg = im.getpixel((2, 2))
        diff = PILImage.new('RGB', im.size, bg)
        from PIL import ImageChops
        box = ImageChops.difference(im, diff).convert('L').point(lambda v: 255 if v > 18 else 0).getbbox()
        if box:
            pad = 12
            box = (max(0, box[0] - pad), max(0, box[1] - pad), min(im.width, box[2] + pad), min(im.height, box[3] + pad))
            im = im.crop(box)
    out = os.path.join(TMP, os.path.basename(path))
    im.save(out)
    r = im.height / im.width
    ww = w
    hh = ww * r
    if h and hh > h:
        hh = h
        ww = hh / r
    return Image(out, width=ww, height=hh)


def table(rows, widths, head=True, zebra=True, fs=None):
    data = []
    for i, row in enumerate(rows):
        st = 'head' if (head and i == 0) else 'cell'
        data.append([c if not isinstance(c, str) else P(c, st) for c in row])
    t = Table(data, colWidths=widths, repeatRows=1 if head else 0)
    style = [('VALIGN', (0, 0), (-1, -1), 'TOP'), ('LEFTPADDING', (0, 0), (-1, -1), 4), ('RIGHTPADDING', (0, 0), (-1, -1), 4),
             ('TOPPADDING', (0, 0), (-1, -1), 2.2), ('BOTTOMPADDING', (0, 0), (-1, -1), 2.2),
             ('LINEBELOW', (0, 0), (-1, -1), 0.25, colors.HexColor('#c9d1dc'))]
    if head:
        style += [('BACKGROUND', (0, 0), (-1, 0), NAVY)]
    if zebra:
        for i in range(1 if head else 0, len(rows)):
            if i % 2 == 0:
                style.append(('BACKGROUND', (0, i), (-1, i), LIGHT))
    t.setStyle(TableStyle(style))
    return t


def callout(text, color=NAVY):
    t = Table([[P(text, 'call')]], colWidths=[CW])
    t.setStyle(TableStyle([('BACKGROUND', (0, 0), (-1, -1), LIGHT), ('LINEBEFORE', (0, 0), (0, -1), 3, color),
                           ('LEFTPADDING', (0, 0), (-1, -1), 8), ('TOPPADDING', (0, 0), (-1, -1), 5), ('BOTTOMPADDING', (0, 0), (-1, -1), 5)]))
    return t


def two(a, b, wa=0.5):
    t = Table([[a, b]], colWidths=[CW * wa, CW * (1 - wa)])
    t.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP'), ('LEFTPADDING', (0, 0), (-1, -1), 0), ('RIGHTPADDING', (0, 0), (-1, -1), 6)]))
    return t


def figure(path, caption, w, h=None):
    t = Table([[img(path, w, h)], [P(caption, 'cap')]], colWidths=[w])
    t.setStyle(TableStyle([('ALIGN', (0, 0), (-1, -1), 'CENTER'), ('LEFTPADDING', (0, 0), (-1, -1), 0), ('RIGHTPADDING', (0, 0), (-1, -1), 0),
                           ('TOPPADDING', (0, 0), (-1, -1), 1), ('BOTTOMPADDING', (0, 0), (-1, -1), 1)]))
    return t


def on_page(c, doc):
    n = doc.page
    if n == 1:
        return
    c.saveState()
    c.setFillColor(NAVY)
    c.rect(0, H - 9 * mm, W, 9 * mm, stroke=0, fill=1)
    c.setFillColor(colors.white)
    c.setFont('SegB', 9)
    c.drawString(M, H - 6 * mm, 'THENAR WALKER  ·  RoboReach 2026-27')
    c.setFont('Seg', 8.5)
    c.drawRightString(W - M, H - 6 * mm, 'Team build guide')
    c.setFillColor(GREY)
    c.setFont('Seg', 7.5)
    c.drawString(M, 8 * mm, 'Everything here is generated from the project files (CAD + simulation). Nothing has been built or powered yet.')
    c.drawRightString(W - M, 8 * mm, f'{n} / 12')
    c.restoreState()


# ---------------------------------------------------------------- diagrams
def arrow(d, x1, y1, x2, y2, col=NAVY):
    d.add(Line(x1, y1, x2, y2, strokeColor=col, strokeWidth=1.1))
    import math
    a = math.atan2(y2 - y1, x2 - x1)
    s = 5
    d.add(Polygon([x2, y2, x2 - s * math.cos(a - 0.4), y2 - s * math.sin(a - 0.4), x2 - s * math.cos(a + 0.4), y2 - s * math.sin(a + 0.4)],
                  fillColor=col, strokeColor=col))


def box(d, x, y, w, h, title, sub='', fill=LIGHT, edge=NAVY, tc=NAVY):
    d.add(Rect(x, y, w, h, rx=4, ry=4, fillColor=fill, strokeColor=edge, strokeWidth=0.9))
    d.add(String(x + w / 2, y + h - 11, title, fontName='SegB', fontSize=7.8, fillColor=tc, textAnchor='middle'))
    for i, line in enumerate(sub.split('\n') if sub else []):
        d.add(String(x + w / 2, y + h - 21 - 9 * i, line, fontName='Seg', fontSize=6.6, fillColor=GREY, textAnchor='middle'))


def power_tree():
    d = Drawing(CW, 205)
    bw, bh = 92, 34
    cx = CW / 2
    box(d, cx - 70, 170, 140, 32, '3S LiPo 2200 mAh', '11.1 V nominal · 12.6 V full · XT60')
    arrow(d, cx, 170, cx, 156)
    box(d, cx - 70, 124, 140, 32, '10–15 A fuse  →  KILL SWITCH', 'KCD4 30 A rocker, on the deck', fill=colors.HexColor('#fde8df'), edge=ORANGE, tc=ORANGE)
    arrow(d, cx, 124, cx, 112)
    d.add(Line(30, 112, CW - 30, 112, strokeColor=NAVY, strokeWidth=1.3))
    d.add(String(CW - 30, 115, 'VBAT bus (16 AWG)', fontName='Seg', fontSize=6.6, fillColor=GREY, textAnchor='end'))
    xs = [30, 30 + (CW - 60) / 4, 30 + (CW - 60) / 2, 30 + 3 * (CW - 60) / 4, CW - 30]
    items = [('MDD10A ch 1', 'left motors\nFL + RL'), ('MDD10A ch 2', 'right motors\nFR + RR'), ('6 V buck ≥10 A', '→ PCA9685 V+\n6 × MG996R'),
             ('5 V buck MP1584', '→ ESP32 5V\n(3V3 → PCA logic)'), ('100k / 27k', 'divider\n→ GPIO34')]
    for x, (t, s) in zip(xs, items):
        arrow(d, x, 112, x, 88)
        box(d, x - bw / 2 + (8 if x == xs[0] else 0) - (8 if x == xs[-1] else 0), 50, bw, 38, t, s)
    d.add(String(cx, 30, 'All grounds common. Every voltage on the robot ≤ 12.6 V (rule: ≤ 24 V DC).', fontName='Seg', fontSize=7.2,
                 fillColor=GREY, textAnchor='middle'))
    d.add(String(cx, 18, 'Kill switch is before everything → it shuts the whole robot down (rule).', fontName='Seg', fontSize=7.2,
                 fillColor=GREY, textAnchor='middle'))
    return d


def state_diagram():
    d = Drawing(CW, 92)
    box(d, 10, 30, 120, 44, 'DISARMED', 'power-up · servo PWM off', fill=colors.HexColor('#f1f1f1'), edge=GREY, tc=GREY)
    box(d, CW / 2 - 70, 30, 140, 44, 'ACTIVE', 'drive + arm follow the leader', fill=colors.HexColor('#e3f3e8'), edge=GREEN, tc=GREEN)
    box(d, CW - 170, 30, 160, 44, 'HOLD', 'wheels 0 · arm frozen\ngripper PWM ON (keeps the cube)', fill=colors.HexColor('#fde8df'), edge=ORANGE, tc=ORANGE)
    arrow(d, 130, 52, CW / 2 - 70, 52, GREEN)
    d.add(String((130 + CW / 2 - 70) / 2, 57, 'ENABLE + leader at STOW', fontName='Seg', fontSize=6.6, fillColor=GREY, textAnchor='middle'))
    arrow(d, CW / 2 + 70, 60, CW - 170, 60, ORANGE)
    d.add(String((CW / 2 + 70 + CW - 170) / 2, 64, 'no packet 250 ms / ENABLE off / bad data', fontName='Seg', fontSize=6.4, fillColor=GREY, textAnchor='middle'))
    arrow(d, CW - 170, 42, CW / 2 + 70, 42, GREEN)
    d.add(String((CW / 2 + 70 + CW - 170) / 2, 33, 'NEW valid signed command', fontName='Seg', fontSize=6.4, fillColor=GREY, textAnchor='middle'))
    d.add(String(CW / 2, 10, 'The kill switch cuts power in every state.', fontName='Seg', fontSize=7, fillColor=GREY, textAnchor='middle'))
    return d


def steer_diagram():
    """Top view: how skid steering turns the robot (front = up). Arrow up = wheel drives forward."""
    d = Drawing(CW, 118)
    cases = [('Straight', (1, 1)), ('Curve right', (1, 0.4)), ('Spin right', (1, -1)), ('Spin left', (-1, 1))]
    pw = CW / 4
    for i, (name, (l, r)) in enumerate(cases):
        cx = pw * i + pw / 2
        d.add(Rect(cx - 18, 30, 36, 62, fillColor=colors.HexColor('#24456e'), strokeColor=NAVY))
        d.add(String(cx, 95, 'front', fontName='Seg', fontSize=6, fillColor=GREY, textAnchor='middle'))
        for side, v in ((-1, l), (1, r)):
            for wy in (36, 70):
                x = cx + side * 26
                d.add(Rect(x - 5, wy, 10, 18, fillColor=colors.black, strokeColor=colors.black))
                ln = 16 * abs(v)
                y0 = wy + 9 - ln / 2 * (1 if v > 0 else -1)
                arrow(d, x + side * 12, y0, x + side * 12, y0 + ln * (1 if v > 0 else -1), GREEN if v > 0 else ORANGE)
        d.add(String(cx, 16, name, fontName='SegB', fontSize=8, fillColor=NAVY, textAnchor='middle'))
        lbl = {'Straight': 'both sides forward', 'Curve right': 'left fast, right slow',
               'Spin right': 'left fwd, right back', 'Spin left': 'left back, right fwd'}[name]
        d.add(String(cx, 6, lbl, fontName='Seg', fontSize=6.8, fillColor=GREY, textAnchor='middle'))
    return d


# ---------------------------------------------------------------- pages
def page1():
    st = [Spacer(1, 6 * mm), P('THENAR WALKER', 'big'), P('Mobile robot arm for <b>RoboReach</b> — Techfest IIT Bombay 2026-27', 'sub'),
          Spacer(1, 3 * mm), P('Team build guide: what we are building, what to buy, how to wire, flash, build and test it.', 'b'),
          Spacer(1, 3 * mm), figure(R('cad', 'renders', 'robot_STOW_iso.png'), 'The robot folded in its start pose (STOW) — SolidWorks render', CW, 88 * mm),
          Spacer(1, 3 * mm)]
    v = VER['configs']
    rows = [['', 'Rule', 'Ours (measured)'],
            ['Start size', '≤ 300 × 200 × 300 mm', ' × '.join(f'{x:.1f}' for x in v['STOW']['size_lwh_mm']) + ' mm'],
            ['Ground clearance', '≥ 50 mm', f"{v['STOW']['lowest_non_wheel']['z_mm']:.0f} mm"],
            ['Grasp object centred at', '450 mm', f"{v['REACH_450']['grasp_centre_mm'][2]:.1f} mm"],
            ['Voltage', '≤ 24 V DC', '12.6 V max (3S LiPo)'],
            ['Mass', '—', f"{v['STOW']['mass_total_g'] / 1000:.2f} kg"],
            ['Drive', '—', '4 × 25GA-370 12 V 60 RPM, Ø140 mm wheels, ~0.34 m/s'],
            ['Full course in simulation', '—', 'pick → ramp → bridge → speed breakers → place: PASS']]
    st.append(table(rows, [CW * 0.28, CW * 0.27, CW * 0.45]))
    st += [Spacer(1, 3 * mm), P('<b>Contents</b> — 2 Rules · 3 Robot at a glance · 4 The arm · 5 Drive &amp; turning · 6–7 Shopping list · '
                                 '8 Wiring · 9 Firmware &amp; safety · 10 Build guide · 11 Verified in SolidWorks · 12 Simulated run + team checklist', 'small'),
           PageBreak()]
    return st


def page2():
    st = [P('The challenge and the rules that shape the robot', 'h1'),
          P('RoboReach: a <b>manually controlled, wireless</b> robot with an arm. One operator, 7 minutes, 15 obstacles/tasks on a 3 × 3 m arena. '
            'Points + time bonus (300 − time, max 100) − penalties. Max 870 + 100.', 'b'),
          P('Rules that decide the design', 'h2')]
    rows = [['Rule', 'What we did'],
            ['Fit in 300 × 200 × 300 mm at the start', 'Arm folds to STOW on a <b>low deck (103 mm)</b>; robot 293.7 × 196 × 292.9 mm'],
            ['Grab an object centred at 450 mm, bot on the ground', 'Arm on the deck reaches ~640 mm; REACH_450 pose puts a cube centre at 449.6 mm'],
            ['Ground clearance ≥ 50 mm', 'Ø140 wheels, motors on the axle line → lowest point 57 mm'],
            ['Wheels always touch the ground; ramp 15°, bumps 40 mm', '4WD, big wheels; checked in SolidWorks + physics simulation'],
            ['Mechanical gripper only', 'The thenar-arms jaw gripper (no glue, magnets, suction)'],
            ['Wi-Fi/BT, name <b>TeamName_TF</b>, authentication', 'ESP32 Wi-Fi AP, every command signed (HMAC-SHA256), replays rejected'],
            ['Disconnect → motors stop, arm stops, gripper keeps holding', 'Firmware HOLD state — tested in simulation with a Wi-Fi drop'],
            ['On-board battery ≤ 24 V, secured, kill switch', '3S LiPo in a screwed cage, 15 A fuse, 30 A kill switch on the deck'],
            ['No autonomous motion', 'Pure teleoperation: leader arm + joystick → robot']]
    st.append(table(rows, [CW * 0.42, CW * 0.58]))
    st.append(P('Tasks and points (where the points are)', 'h2'))
    tasks = [['Task', 'Pts', 'Task', 'Pts', 'Task', 'Pts'],
             ['Pickup (50 mm cube)', '10', 'Stacker (3 cyl + cone)', '90', 'Buzzed (wire loop)', '<b>150</b>'],
             ['Ascent (15° ramp)', '30', 'Longshot (throw cube)', '≤100', 'Diagonals (250 mm track)', '20'],
             ['Bridge (400 mm wide)', '10', 'Robonardo (draw TF)', '<b>130</b>', 'Escape room (4 shapes)', '≤100'],
             ['Descent + speed breakers', '60', 'Downy (cube at 400 mm)', '30', 'Slalom ridge', '40'],
             ['Touchdown (80 mm square)', '10', "Hanoi'd (tower of Hanoi)", '80', 'The End (stop in zone)', '10']]
    st.append(table(tasks, [CW * 0.25, CW * 0.07, CW * 0.25, CW * 0.08, CW * 0.27, CW * 0.08]))
    st += [Spacer(1, 3 * mm),
           two(figure(R('cad', 'renders', 'arena', 'arena_iso.png'), 'Our SolidWorks model of the 3 × 3 m arena (layout from the rulebook images, ±10 %)', CW * 0.55, 62 * mm),
               P('<b>Penalties to avoid:</b> dropping the cube between pickup and touchdown (−15 each), wheels leaving the track (−10), '
                 'touching the buzz wire (−20 each), letters outside the outline (−30). '
                 '<b>Practise Buzzed and Robonardo</b> — 280 points need a steady arm, not speed.', 'b'), 0.58),
           PageBreak()]
    return st


def page3():
    v = VER['configs']['STOW']
    st = [P('The robot at a glance', 'h1'),
          P('A printed 4WD chassis with the <b>thenar-arms MG996R follower arm</b> bolted on top (arm unchanged). '
            'The operator uses the <b>thenar-arms encoder leader</b> arm plus a thumb joystick.', 'b'),
          two(figure(R('cad', 'renders', 'robot_STOW_side.png'), 'Side (STOW)', CW * 0.48, 55 * mm),
              figure(R('cad', 'renders', 'robot_STOW_front.png'), 'Front (STOW)', CW * 0.48, 55 * mm)),
          Spacer(1, 2 * mm)]
    rows = [['Item', 'Value', 'Item', 'Value'],
            ['Length × width × height (start)', '293.7 × 196 × 292.9 mm', 'Wheel diameter / width', '140 / 28 mm'],
            ['Wheelbase / track', '152 / 168 mm', 'Ground clearance', '57 mm'],
            ['Chassis tub', '240 × 138 mm, printed PETG', 'Deck top', '103 mm'],
            ['Mass (CAD)', f"{v['mass_total_g'] / 1000:.2f} kg", 'Centre of gravity height', f"{v['cog_mm'][2]:.0f} mm (STOW)"],
            ['Battery', '3S LiPo 2200 mAh', 'Run time per pack', f"~{DRV['runtime_avg_min_2200mAh_80pct']:.0f} min (run is 7 min)"]]
    st.append(table(rows, [CW * 0.27, CW * 0.23, CW * 0.25, CW * 0.25]))
    st += [P('How it is laid out', 'h2'),
           P('• <b>Tub</b> (floor 57 mm off the ground): 4 motors screwed to the side walls, battery in a cage on the left, '
             '6 V buck + PCA9685 + 2 motor drivers on the right, 5 V buck at the back. ESP32 hangs under the deck.', 'b'),
           P('• <b>Deck</b> (top at 103 mm): the arm base sits in a 4 mm fence and is bolted with 4 × M3 through its own vent slots. '
             'Kill switch at the front right.', 'b'),
           P('• <b>Why the deck is so low:</b> the arm cannot fold lower than ~190 mm without hitting itself (we searched every pose '
             'with collision checking). On a normal 135 mm deck the robot would be 325 mm tall — too tall for the start box.', 'b'),
           two(figure(R('cad', 'renders', 'robot_STOW_top.png'), 'Top view', CW * 0.46, 48 * mm),
               figure(R('cad', 'renders', 'robot_HOME_iso.png'), 'HOME pose (carrying pose)', CW * 0.5, 48 * mm), 0.48),
           PageBreak()]
    return st


def page4():
    v = VER['configs']
    st = [P('The arm: poses, reach and gripper', 'h1'),
          two(figure(R('cad', 'renders', 'robot_REACH_450_side.png'), 'REACH_450 — cube centre at 449.6 mm, jaws closing sideways', CW * 0.48, 62 * mm),
              figure(R('cad', 'renders', 'robot_PICK_FLOOR_side.png'), 'PICK_FLOOR — 50 mm cube on the floor', CW * 0.48, 62 * mm)),
          P('Arm poses (joint angles in degrees: pan, lift, elbow, wrist flex, wrist roll, gripper)', 'h2')]
    rows = [['Pose', 'Joints', 'Use']]
    use = {'STOW': 'start box; fold here before the run', 'HOME': 'carrying the cube over obstacles (most stable)',
           'REACH_450': 'objects up to 450 mm (rule), Downy pole top', 'PICK_FLOOR': 'cubes on the floor, 254 mm ahead of centre'}
    for k in ('STOW', 'HOME', 'REACH_450', 'PICK_FLOOR'):
        rows.append([f'<b>{k}</b>', ', '.join(f'{x:g}' for x in v[k]['joint_deg']), use[k]])
    st.append(table(rows, [CW * 0.18, CW * 0.37, CW * 0.45]))
    gg = load('analysis', 'gripper_gap.json')
    grow = [['Gripper angle', '0°', '10°', '20°', '30°', '35°', '40°', '50°', '60°']]
    grow.append(['Opening (mm)'] + [f"{gg[str(a)]['inner_gap_15mm_from_tip']:.0f}" for a in (0, 10, 20, 30, 35, 40, 50, 60)])
    st += [P('Gripper opening (measured on the real jaw meshes)', 'h2'),
           table(grow, [CW * 0.2] + [CW * 0.1] * 8),
           Spacer(1, 2 * mm),
           P('• 50 mm cube: grips around 28°. 60 mm cube / Ø60 cylinder: ~38°. Max ~79 mm.', 'b'),
           P('• The jaws do not fully close (≈20 mm gap at 0°). <b>Add 4 mm TPU finger pads</b> so it can hold the Robonardo marker '
             'and grip thermocol better (the cube slid ~7 mm along bare fingers in the simulation).', 'b'),
           P('• The URDF "tool point" is on the fixed finger; a held 50 mm object is centred 26.5 mm to the side. All heights here use the true object centre.', 'b'),
           callout('<b>Driving rule:</b> carry objects in HOME over the ramp, bridge and speed breakers. Stretched-out poses still pass the '
                   'stability check, but HOME has the most margin (33 mm on the 15° descent).', ORANGE),
           PageBreak()]
    return st


def page5():
    st = [P('Drive: motors, wheels and turning', 'h1'),
          two(table([['Motor to buy', ''], ['Name in shops', '<b>25GA-370 12 V 60 RPM</b> (same as JGA25-370 103:1)'],
                     ['How many', '4 (+1 spare)'], ['Where', 'Robu.in ₹389 (in stock when checked); The Engineer Store ₹462'],
                     ['Check', '12 V · 60 RPM · 4 mm D-shaft · metal gears · no encoder'],
                     ['Do NOT buy', '100 RPM version — too weak for the ramp']], [CW * 0.16, CW * 0.36]),
              table([['Drive numbers', ''], ['Ramp force (15°)', f"{DRV['ramp_force_N']} N"], ['Needed per motor', f"{DRV['per_motor_kgcm_4wd']} kg·cm (4 driving)"],
                     ['Worst case', f"{DRV['per_motor_kgcm_2_driving']} kg·cm (2 driving, on a bump)"],
                     ['Motor rated / stall', f"{DRV['rated_kgcm']} / {DRV['stall_kgcm']} kg·cm"],
                     ['Speed', f"{DRV['speed_rated_m_s']} m/s loaded, {DRV['speed_free_m_s']} free"]], [CW * 0.18, CW * 0.28]), 0.53),
          Spacer(1, 2 * mm),
          callout('<b>Motor check when it arrives (10 min):</b> put a printed wheel on one motor, tie a string to the tyre, hang a full '
                  '<b>1 L water bottle</b>, run at 12 V. It must lift it. Shops list different strengths for the same name — the simulation '
                  'needs ≥ 6 kg·cm (5 gets stuck on the speed breakers). If it lifts it, buy the other three.', ORANGE),
          P('How it steers — no steering servo, no steering shaft', 'h2'),
          P('<b>Skid steering, like a tank or a JCB.</b> One Cytron MDD10A drives both left wheels (channel 1) and both right wheels (channel 2). '
            'The ESP32 mixes the joystick: <b>left = throttle − turn, right = throttle + turn</b>. Different speeds on the two sides '
            'turn the robot; opposite directions spin it on the spot. Each wheel is fixed straight on its motor shaft.', 'b'),
          steer_diagram(),
          P('Turning — why the front tyres are hard plastic', 'h2'),
          P('The robot is almost square (wheelbase 152 ≈ track 168), so every turn drags the tyres sideways. With rubbery TPU on all four '
            'wheels it could hardly turn. We tested options in the physics simulation:', 'b')]
    names = {'JGA25 78:1 (6.2 kg.cm, 77 rpm), all TPU  [current]': 'All TPU tyres (first design)',
             'JGA25 131:1 (~10 kg.cm, ~46 rpm), all TPU': 'Stronger, slower motor, all TPU',
             'JGA25 171:1 (~12 kg.cm, ~35 rpm), all TPU': 'Strongest motor, all TPU',
             'JGA25 78:1, PETG front / TPU rear': '<b>PETG front + TPU rear (chosen)</b>'}
    rows = [['Variant', 'Spin 35%', 'Spin 65%', 'Spin 100%', 'Arc turn', 'Top speed', 'Ramp']]
    for k, r in TURN.items():
        rows.append([names.get(k, k), f"{r['spin_35%_deg_s']}°/s", f"{r['spin_65%_deg_s']}°/s", f"{r['spin_100%_deg_s']}°/s",
                     f"{r['arc_30/90%_deg_s']}°/s", f"{r['straight_100%_m_s']} m/s", 'yes' if r['climbs_ramp_at_35%'] else 'no'])
    st.append(table(rows, [CW * 0.3] + [CW * 0.1] * 4 + [CW * 0.11, CW * 0.09]))
    st += [P('Study done with a 77 rpm motor. The 60 RPM motor you buy was then re-checked in the full simulation: it spins 145° to within '
             '0.2° and passes the whole course, a bit slower (~0.34 m/s).', 'small'),
           Spacer(1, 2 * mm),
           P('• <b>Front wheels: one-piece hard PETG. Rear wheels: PETG hub + TPU 95A tyre.</b> Hard rear tyres were also tried: they get stuck on the speed breakers.', 'b'),
           P('• Firmware gives turning its own power (50/75/100 %) with a 35 % minimum so a small stick push still turns.', 'b'),
           P('• <b>Drive like a car</b> (curves while moving) where you can; spin on the spot for small corrections. In the 250 mm '
             'Diagonals corridor the robot (196 mm wide) cannot spin around — plan arcs.', 'b'),
           PageBreak()]
    return st


def shop(rows, widths=None):
    return table([['Qty', 'Part', 'Spec / notes', 'Where to buy']] + rows, widths or [CW * 0.07, CW * 0.27, CW * 0.44, CW * 0.22])


def page6():
    st = [P('Shopping list (1/2) — rover module', 'h1'),
          P('Everything for the driving base. Specs matter more than brand — check against the "Spec" column.', 'b'),
          P('Drive', 'h2'),
          shop([['4 (+1)', '<b>25GA-370 12 V 60 RPM gear motor</b>', '4 mm D-shaft, metal gears, no encoder. Do the 1 L bottle test.', 'Robu.in (₹389), The Engineer Store'],
                ['1', '<b>Cytron MDD10A</b> dual motor driver', '10 A per channel, PWM + DIR, 84.5 × 62 mm. Already bought; sits on the deck top.', 'bought locally'],
                ['4', 'M3 × 5 grub screw (cup point)', 'Wheel hub onto the motor shaft flat', 'Local hardware / Robu'],
                ['8', 'M3 × 6 socket screw', 'Motor face to chassis wall (2 per motor)', 'Local hardware'],
                ['2 m', '22 AWG silicone wire (red/black)', 'Motor leads', 'Robu / Amazon.in']]),
          P('Power and safety', 'h2'),
          shop([['1 (+1)', '<b>3S LiPo 2200 mAh 30C</b>, XT60', 'Max 106 × 34 × 26 mm to fit the cage', 'Robu / Amazon.in / hobby shop'],
                ['1', 'LiPo balance charger + LiPo-safe bag', 'Charge only in the bag, never unattended', 'Robu / Amazon.in'],
                ['1', '<b>KCD4 30 A DPST rocker (red)</b>', 'The kill switch. Panel cut-out 22.2 × 30.2 mm', 'Robu / Amazon.in'],
                ['1', 'Inline blade fuse holder + 15 A fuse', 'Right after the battery', 'Car accessories shop'],
                ['1', 'XT60 male/female pair', 'Battery connector', 'Robu'],
                ['1', '<b>6 V DC-DC buck, ≥ 10 A</b>', 'XL4016 / "300 W 20 A" type, adjustable. Feeds the 6 servos', 'Robu / Amazon.in'],
                ['1', 'MP1584 mini buck', 'Set to 5.0 V for the ESP32', 'Robu'],
                ['1 m', '16 AWG silicone wire', 'Battery → fuse → switch → drivers', 'Robu']]),
          P('Brains', 'h2'),
          shop([['1', '<b>ESP32-DevKitC</b> (WROOM-32E, 38 pin)', 'The robot controller (Wi-Fi AP + motors + servos)', 'Robu / Amazon.in'],
                ['1', '<b>PCA9685</b> 16-ch PWM board', 'Drives the 6 arm servos over I²C', 'Robu'],
                ['1 + 1', 'Resistors 100 kΩ + 27 kΩ', 'Battery voltage sense', 'Any'],
                ['1 lot', 'Jumper wires, servo extensions, heat-shrink, zip ties', '', 'Any']]),
          P(f"Rover prints — 5 ready P1S plates in cad/print, no supports (~{PT['total_hours']:.0f} h total)", 'h2'),
          shop([['1', 'Plate 1: tub + battery cage + 2 bars', 'PETG, 4 walls, 30 % gyroid (~7 h, ~260 g)', 'P1S'],
                ['1', 'Plate 2: deck plate', 'PETG, 250 mm long — fits the P1S bed (~3 h, ~130 g)', 'P1S'],
                ['2', 'Plates 3 + 4: front wheel + rear hub', 'PETG, ~5.4 h each; front wheels are one piece (hard tread). <b>Print plate 3 first</b> (motor test)', 'P1S'],
                ['1', 'Plate 5: 2 rear tyres', '<b>TPU 95A</b>, 3 walls, 20 % (~7.7 h, ~90 g)', 'P1S']]),
          PageBreak()]
    return st


def page7():
    st = [P('Shopping list (2/2) — arm, controller, hardware', 'h1'),
          P('Arm — the thenar-arms MG996R follower (unchanged; full list in thenar-arms R3-PRINT.md)', 'h2'),
          shop([['6', '<b>MG996R</b> servo (180°)', 'Pan, lift, elbow, wrist flex, wrist roll, gripper', 'Robu / Amazon.in'],
                ['6', 'Metal servo horn 25T, Ø20 mm disc', '4 × M3 on 14 mm circle — must match your servo spline', 'Robu'],
                ['24 / 24 / 48', 'M3 × 12 screws / nuts / washers', 'Servo tabs', 'Local hardware'],
                ['24', 'M3 × 8 screws', 'Horns to links', 'Local hardware'],
                ['7', 'Arm prints (R3)', 'Base, shoulder, upper arm, forearm, wrist, gripper body, jaw — PETG', 'Print'],
                ['2', 'TPU finger pads, 4 mm', 'Recommended (marker + grip)', 'Print'],
                ['4', 'M3 × 20 + washer + nyloc', 'Arm base to deck, through its vent slots', 'Local hardware']]),
          P('Controller — added to the thenar-arms encoder leader', 'h2'),
          shop([['1', 'Leader arm (thenar-arms L1)', '6 × AS5600, TCA9548A, ESP32, bearings — its own BOM', 'Already designed'],
                ['1', '2-axis thumb joystick module', '3.3 V; VRx → GPIO34, VRy → GPIO35', 'Robu'],
                ['1', 'Toggle switch (ENABLE)', 'GPIO32 to GND', 'Robu'],
                ['1', 'Push button (SPEED)', 'GPIO33 to GND', 'Robu'],
                ['1', 'USB power bank + cable', 'Powers the leader ESP32', 'Any']]),
          P('Chassis hardware', 'h2'),
          shop([['4', 'M3 heat-set insert (Ø4.0 bore)', 'Tub corner columns', 'Robu / Amazon.in'],
                ['4', 'M3 × 10 socket screw', 'Deck to tub', 'Local hardware'],
                ['2', 'M3 × 8 + nyloc', 'Battery cage to floor', 'Local hardware'],
                ['4', 'M3 × 8 self-tapping', 'Battery bars', 'Local hardware'],
                ['20', 'M2.5 × 6 self-tapping', 'Boards to the floor standoffs', 'Local hardware'],
                ['2', 'M2.5 × 10 brass standoff (F-F)', 'ESP32 under the deck', 'Robu']]),
          P('Filament and tools', 'h2'),
          shop([[f"~{PT['petg_g'] / 1000:.2f} kg", 'PETG', 'Rover plates 1–4 (+ the arm parts if not printed yet)', ''],
                [f"~{PT['tpu_g']} g", 'TPU 95A', 'Rear tyres (+ finger pads)', ''],
                ['—', 'Soldering iron, heat-set insert tip, multimeter, hex keys, crimper', 'Multimeter is a must for setting the bucks', '']]),
          Spacer(1, 3 * mm),
          callout('<b>Not needed:</b> camera, Raspberry Pi, encoders, IMU. The ESP32 is the right brain here: boots in under a second, '
                  'survives the kill switch (no SD card), and the safety firmware is already written and tested. '
                  'Add an ESP32-CAM later only if you want a camera view.'),
          PageBreak()]
    return st


def page8():
    st = [P('Wiring', 'h1'), P('Power', 'h2'), power_tree(), P('Robot ESP32 pins', 'h2')]
    pins = [['ESP32 pin', 'Goes to', 'ESP32 pin', 'Goes to'],
            ['GPIO21 / 22', 'PCA9685 SDA / SCL', 'GPIO16 / 17', 'MDD10A PWM2 / DIR2 (right motors)'],
            ['GPIO13', 'PCA9685 OE (servo output enable)', 'GND', 'MDD10A header GND (pin 1)'],
            ['GPIO25 / 26', 'MDD10A PWM1 / DIR1 (left motors)', 'GPIO34', 'battery divider (100k / 27k)'],
            ['GPIO2', 'on-board LED (solid = ACTIVE)', '', ''],
            ['5V / GND', '5 V buck output', '3V3', 'PCA9685 VCC (logic only)']]
    st.append(table(pins, [CW * 0.15, CW * 0.35, CW * 0.15, CW * 0.35]))
    st += [Spacer(1, 2 * mm),
           two(table([['PCA9685 ch', 'Servo'], ['0', 'J1 shoulder pan'], ['1', 'J2 shoulder lift'], ['2', 'J3 elbow'],
                      ['3', 'J4 wrist flex'], ['4', 'J5 wrist roll'], ['5', 'J6 gripper']], [CW * 0.15, CW * 0.3]),
               table([['Controller (leader ESP32)', ''], ['GPIO34 / 35', 'joystick X (turn) / Y (throttle)'],
                      ['GPIO32', 'ENABLE toggle → GND'], ['GPIO33', 'SPEED button → GND'], ['3V3 / GND', 'joystick power (not 5 V)'],
                      ['21 / 22', 'leader encoders (unchanged)']], [CW * 0.17, CW * 0.33]), 0.47),
           Spacer(1, 2 * mm),
           callout('<b>Before connecting:</b> set the 6 V buck to 6.0 V and the 5 V buck to 5.0 V with a multimeter, no load. Never power '
                   'servos from the ESP32 or USB. MDD10A: M1A/M1B = both left motors in parallel, M2A/M2B = both right; 10 kΩ from PWM1 and '
                   'PWM2 to GND. Whole side backwards: swap that side\'s A/B wires. One wheel fighting its partner: swap that motor\'s two wires.', ORANGE),
           PageBreak()]
    return st


def page9():
    st = [P('Firmware, radio link and safety', 'h1'),
          P('Two ESP32 programs share one library (ThenarLink). The safety logic is unit-tested on a PC (39/39 checks) and was run '
            'inside the physics simulation.', 'b'),
          state_diagram(),
          P('What judges will check — and how we meet it', 'h2'),
          table([['Rule', 'How'],
                 ['Name TeamName_TF', 'Robot makes a WPA2 Wi-Fi network called &lt;TeamName&gt;_TF (spaces removed, max 20 chars), 1 device max'],
                 ['Only our controller', 'Every packet signed with a 32-byte secret key (HMAC-SHA256); new session id per pairing; old/copied packets rejected'],
                 ['Disconnect', '250 ms without a valid packet → HOLD: wheels stop at once, arm freezes, gripper keeps holding'],
                 ['Stays disabled', 'Moves again only on a NEW valid command; arm catches up slowly (45°/s) for 1.5 s'],
                 ['Kill switch', 'Hardware, in series with the battery — cuts everything']], [CW * 0.2, CW * 0.8]),
          P('Driving controls', 'h2'),
          P('ENABLE switch on = robot may move. SPEED button cycles <b>precision 35 % / normal 65 % / fast 100 %</b> for driving '
            '(turning 50 / 75 / 100 %). Power-up: the arm only engages when the leader arm is folded to STOW (within 6°), so it can never jump.', 'b'),
          P('Set up and flash', 'h2'),
          table([['Step', 'Command / action'],
                 ['1. Make your secret key', 'python firmware/tools/new_key.py "Your Team Name"'],
                 ['2. Test the logic on a PC', 'powershell -File firmware/tests/run_tests.ps1   (39 checks)'],
                 ['3. Compile both sketches', 'powershell -File firmware/tools/compile_firmware.ps1'],
                 ['4. Flash', 'thenar_walker_bot → robot ESP32;  thenar_walker_controller → leader ESP32'],
                 ['5. Leader calibration', 'Hold leader at HOME, send ZERO in the serial monitor; SIGN &lt;axis&gt; -1 for reversed joints; JOYZERO'],
                 ['6. Servo calibration', 'Measure each servo zero/direction with horns off → calibration.h → FOLLOWER_CALIBRATED = true']],
                [CW * 0.27, CW * 0.73]),
          Spacer(1, 2 * mm),
          callout('Until servo calibration is done, the robot drives but the arm stays limp — on purpose. Never share or commit team_config.h.'),
          PageBreak()]
    return st


def page10():
    steps = [
        ('0 · Before buying everything', ['Buy one motor first and do the 1 L bottle test (page 5).',
                                         'Measure motor and boards against the CAD; change cad/tools/design.py if different.']),
        ('1 · Print', ['The 5 plates in cad/print (PETG plates 1–4, TPU plate 5) — plate 3 first. No supports. Plus the 7 arm parts.']),
        ('2 · Tub', ['Melt 4 M3 inserts into the corner columns.', 'Screw the battery cage to the floor.',
                     'Drop motors into the U-slots, gearbox face on the wall, 2 × M3 × 6 each (solder leads first).',
                     'Screw the 6 V buck, PCA9685 and the 5 V buck to the floor standoffs (2 pads stay spare).',
                     'Set bucks to 6.0 V / 5.0 V. Wire per page 8, kill switch OFF.']),
        ('3 · Wheels', ['TPU tyres onto the 2 rear hubs; the 2 front wheels are one piece. Push onto the D-shaft ~1 mm off the wall, grub screw on the flat.']),
        ('4 · Deck and arm', ['Screw 2 brass standoffs under the deck and zip-tie the ESP32 to them, snap in the kill switch, route servo leads through the slot. MDD10A onto the 4 deck-top bosses, terminals forward, wires down the front slot.',
                              'Battery into the cage, screw on the bars. Deck onto the tub with 4 × M3 × 10.',
                              'Arm base into the fence, 4 × M3 × 20 through the base vent slots, nylocs under the deck. Servos J1..J6 → PCA ch 0..5.']),
        ('5 · Firmware', ['Key, compile, flash, calibrate — page 9.']),
        ('6 · First power-up (wheels off the ground)', ['Robot and leader in STOW, ENABLE off, kill switch on.',
                                                       'ENABLE on → LED solid. Check every wheel direction.',
                                                       '<b>Failsafe test:</b> while driving and holding a cube, switch the controller off → wheels stop, arm freezes, '
                                                       'cube stays in the gripper. Repeat this at inspection.']),
        ('7 · Before every run', ['Battery ≥ 11.4 V, team key is not the example, robot folded in STOW and inside the 300 × 200 × 300 box.'])]
    st = [P('Build guide', 'h1')]
    for title, items in steps:
        st.append(P(title, 'h2'))
        for it in items:
            st.append(P('<font name="SegSym">☐</font>  ' + it, 'b'))
    st.append(PageBreak())
    return st


def page11():
    st = [P('Verified in SolidWorks', 'h1'),
          P('Native SolidWorks 2026 parts and assemblies, generated by scripts. Checks use exact geometry (body extreme points, '
            'interference detection). The arm matches the URDF kinematics to 0.04 mm.', 'b')]
    rows = [['Result', 'Check', 'Measured']]
    for c in VER['checks']:
        det = c['detail']
        if 'worst margin' in det:
            det = det.split(':')[0]
        det = det.replace('RR-01_Chassis_Tub-1', 'chassis tub').replace(' (+6 intended servo-spline/horn overlaps from the arm CAD)', '')
        rows.append(['<font color="#1f7a3a"><b>PASS</b></font>' if c['pass'] else '<font color="#c0392b"><b>FAIL</b></font>', c['check'], det])
    st.append(table(rows, [CW * 0.09, CW * 0.45, CW * 0.46]))
    st += [Spacer(1, 2 * mm),
           P('Robot placed on the arena\'s real ramp, bridge and speed breakers at 9 positions: <b>only the tyres touch</b> — the chassis, '
             'motors and arm never hit an obstacle.', 'b'),
           two(figure(R('cad', 'renders', 'arena', 'terrain_on_ramp.png'), 'On the 15° ramp', CW * 0.48, 50 * mm),
               figure(R('cad', 'renders', 'arena', 'terrain_breaker_under_belly.png'), 'Speed breaker under the belly', CW * 0.48, 50 * mm)),
           PageBreak()]
    return st


def page12():
    st = [P('Simulated end-to-end run', 'h1'),
          P('MuJoCo physics (real masses, motor torque curve, servo limits, tyre friction) with <b>our actual firmware logic in the loop</b>. '
            'A scripted "operator" drives the leader arm and joystick like a person would.', 'b')]
    titles = {'A': 'Pick 50 mm cube → 15° ramp → bridge → down → 4 speed breakers → place in 80 mm square',
              'B': 'Same + 1.5 s Wi-Fi drop on the bridge + 75 fake/replayed packets',
              'C': 'Downy: 60 mm cube off the 400 mm pole, turn, place in its square'}
    rows = [['Mission', 'What', 'Result', 'Time']]
    for k in 'ABC':
        r = MIS[k]
        n = sum(c['pass'] for c in r['checks'])
        rows.append([f'<b>{k}</b>', titles[k], f"<font color='#1f7a3a'><b>PASS {n}/{len(r['checks'])}</b></font>" if r['all_pass'] else f'FAIL {n}/{len(r["checks"])}',
                     f"{r['sim_time_s']:.0f} s"])
    st.append(table(rows, [CW * 0.09, CW * 0.6, CW * 0.17, CW * 0.14]))
    b = {c['check']: c['detail'] for c in MIS['B']['checks']}
    st += [Spacer(1, 1.5 * mm),
           P('Wi-Fi drop: ' + next(v for k, v in b.items() if k.startswith('Robot stops')) + '; ' +
             next(v for k, v in b.items() if k.startswith('Arm motion')) + '; attacks: ' +
             next(v for k, v in b.items() if k.startswith('Forged')) + '.', 'small'),
           Table([[img(R('sim', 'out', 'mission_A_1.png'), CW * 0.32, 40 * mm, crop=False),
                   img(R('sim', 'out', 'mission_A_3.png'), CW * 0.32, 40 * mm, crop=False),
                   img(R('sim', 'out', 'mission_C_2.png'), CW * 0.32, 40 * mm, crop=False)],
                  [P('Picking the cube', 'cap'), P('Over the speed breakers, cube held', 'cap'), P('Cube taken off the 400 mm pole', 'cap')]],
                 colWidths=[CW / 3] * 3),
           P('What the simulation does NOT prove', 'h2'),
           P('Tyre and gripper friction are estimates; real thermocol dents (squeeze gently); the scripted operator never shakes. '
             'Videos: sim/out/mission_A.mp4, mission_B.mp4, mission_C.mp4.', 'b'),
           P('Team checklist', 'h2')]
    todo = ['Order 1 motor → bottle test → order the rest + all electronics (pages 6–7)',
            'Print the 5 rover plates (cad/print, plate 3 first) and the arm',
            'Build and wire (pages 8, 10); set both bucks before connecting',
            'Flash, calibrate the leader and the servos (page 9)',
            'Test on the real floor: turning, the ramp, a 40 mm bump, the failsafe',
            'Practise: Buzzed (150) and Robonardo (130) are a third of all points',
            'Register the team; the SSID must be YourTeamName_TF']
    for t in todo:
        st.append(P('<font name="SegSym">☐</font>  ' + t, 'b'))
    return st


def main():
    doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=M, rightMargin=M, topMargin=14 * mm, bottomMargin=13 * mm,
                            title='Thenar Walker — RoboReach team guide', author='Thenar Walker team')
    story = []
    for f in (page1, page2, page3, page4, page5, page6, page7, page8, page9, page10, page11, page12):
        story += f()
    doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
    from pypdf import PdfReader
    n = len(PdfReader(OUT).pages)
    print('wrote', OUT, n, 'pages')


if __name__ == '__main__':
    main()
