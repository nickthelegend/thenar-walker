"""Build docs/Thenar_Walker_Build_Book.pdf — sizes of every part (hole by hole), layouts, assembly, wiring, BOM.

Part drawings and hole tables come from cad/tools/make_drawings.py (sections of the exported STLs), so run
that first after any CAD change:
  python cad/tools/make_drawings.py
  python docs/make_build_book.py
"""
import json
import os
import sys

from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.platypus import KeepTogether, PageBreak, SimpleDocTemplate, Spacer, Table, TableStyle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'cad', 'tools'))
import make_team_pdf as T          # noqa: E402  (fonts, styles, table/figure helpers)
import design as D                 # noqa: E402

P, table, callout, two, figure, img = T.P, T.table, T.callout, T.two, T.figure, T.img
NAVY, ORANGE, GREY, GREEN, LIGHT, CW, M, W, H = T.NAVY, T.ORANGE, T.GREY, T.GREEN, T.LIGHT, T.CW, T.M, T.W, T.H
R = T.R
OUT = R('docs', 'Thenar_Walker_Build_Book.pdf')
DR = json.load(open(R('docs', 'drawings', 'drawings.json'), encoding='utf-8'))
VIEWS = {v['name']: v for v in DR['views']}
RED = colors.HexColor('#b42318')
AMBER = colors.HexColor('#9a6312')

# what each hole size is for, per view (matched on the start of the size label)
USE = {
    'RR-01_floor': {'Ø2.2': 'Board standoff pilots (Ø6 × 3 posts): M2.5 × 6 self-tapping. 5 V buck, PCA9685, 6 V buck, 2 spare pads.',
                    '28.0': 'Motor floor window (motor can drops in)', '30.7': 'Motor floor window + half of the battery-cage screw hole'},
    'RR-01_wall_left': {'Ø8.0': 'Motor shaft + gearbox boss', 'Ø3.4': 'M3 × 6: gearbox face to wall (2 per motor, 17 apart)'},
    'RR-01_wall_front': {'8.0': 'Ventilation slot'},
    'RR-01_wall_rear': {'14.0': 'ESP32 USB cable window'},
    'RR-01_top': {'Ø4.0': 'M3 heat-set insert bore, 8 deep'},
    'RR-02_deck': {'Ø2.6': 'Motor driver MDD10A: M3 × 10 self-tapping (pilot in a Ø7 × 6 mm boss)',
                   'Ø2.8': 'ESP32: M2.5 × 10 brass standoffs (hangs under the deck)',
                   'Ø3.4': '4 corners: M3 × 10 deck → tub inserts (counterbored). Inner 4: arm base M3 × 20 + washer + nyloc',
                   '12.0': 'Slots: servo + signal wires (by the arm), motor + power wires (in front of the driver)',
                   '22.2': 'Kill switch cut-out (KCD4 snaps in)'},
    'RR-03_cage_base': {'Ø3.4': 'M3 × 8 + nyloc through the tub floor'},
    'RR-03_cage_top': {'Ø2.6': 'M3 × 8 self-tapping for the bars'},
    'RR-04_bar': {'Ø3.4': 'M3 × 8 clearance (into the cage bosses)'},
    'RR-05_hub_6mm': {'Ø6.3': 'Motor shaft (6 mm round)', 'Ø26.0': 'Lightening holes'},
    'RR-07_front_6mm': {'Ø6.3': 'Motor shaft (6 mm round)', 'Ø26.0': 'Lightening holes'},
}


def use_for(view, size):
    for k, v in USE.get(view, {}).items():
        if size.startswith(k):
            return v
    return ''


def styled(rows, widths, colour_col=None):
    t = table(rows, widths)
    if colour_col is not None:
        cmap = {'HAVE': GREEN, 'PRINTED': GREEN, 'PRINTING': NAVY, 'READY': NAVY, 'ON HOLD': AMBER, 'DECIDE': AMBER,
                'TO BUY': RED, 'CONFIRM': GREY, 'NOT USED': GREY}
        st = []
        for i, r in enumerate(rows[1:], 1):
            c = cmap.get(str(r[colour_col]).split(' ·')[0].upper())
            if c:
                st.append(('TEXTCOLOR', (colour_col, i), (colour_col, i), c))
        t.setStyle(TableStyle(st))
    return t


def hole_block(name):
    v = VIEWS[name]
    groups = {}
    for h in v['holes']:
        groups.setdefault(h['id'][0], []).append(h)
    g_rows = [['ID', 'Size (mm)', 'Qty', 'What it is for']]
    for L, hs in groups.items():
        g_rows.append([f'<b>{L}</b>', hs[0]['size'], str(len(hs)), use_for(name, hs[0]['size'])])
    org = 'from the wheel centre' if v['origin'] == 'center' else 'from the 0,0 corner'
    c_rows = [['ID', 'X', 'Y']] + [[h['id'], f"{h['x']:.1f}", f"{h['y']:.1f}"] for h in v['holes']]
    out = [figure(v['png'], f"Section of the real part · overall {v['size'][0]:.1f} × {v['size'][1]:.1f} mm in this view · "
                              f"hole centres {org} (X to the right, Y up)", CW, 105 * mm)]
    if len(v['holes']):
        # coordinates in up to 3 side-by-side columns; the size table gets the rest of the width
        n = len(c_rows) - 1
        k = 1 if n <= 8 else (2 if n <= 16 else 3)
        per = -(-n // k)
        gw = CW - k * 37 * mm - 3 * mm
        gt = table(g_rows, [10 * mm, 22 * mm, 9 * mm, gw - 41 * mm])
        cols = []
        for j in range(k):
            chunk = [c_rows[0]] + c_rows[1 + j * per: 1 + (j + 1) * per]
            cols.append(table(chunk, [9 * mm, 13 * mm, 13 * mm]))
        ct = Table([cols], colWidths=[37 * mm] * k)
        ct.setStyle(TableStyle([('VALIGN', (0, 0), (-1, -1), 'TOP'), ('LEFTPADDING', (0, 0), (-1, -1), 0), ('RIGHTPADDING', (0, 0), (-1, -1), 2)]))
        out.append(Spacer(1, 2 * mm))
        out.append(two(gt, ct, (gw + 3 * mm) / CW))
    for n_ in v['notes']:
        out.append(P('• ' + n_, 'small'))
    return out


DRAW_ORDER = [('RR-01_floor', 'RR-01 Chassis tub — floor', True), ('RR-01_wall_left', 'RR-01 Chassis tub — side wall', True),
              ('RR-01_top', 'RR-01 Chassis tub — top (insert columns)', False), ('RR-01_wall_front', 'RR-01 Chassis tub — front wall', False),
              ('RR-01_wall_rear', 'RR-01 Chassis tub — rear wall', True), ('RR-02_deck', 'RR-02 Deck plate', True),
              ('RR-03_cage_base', 'RR-03 Battery cage — base', False), ('RR-03_cage_top', 'RR-03 Battery cage — top', False),
              ('RR-04_bar', 'RR-04 Battery bar (× 2)', True), ('RR-05_hub_6mm', 'RR-05 Rear wheel hub (6 mm shaft)', False),
              ('RR-07_front_6mm', 'RR-07 Front wheel (6 mm shaft)', True), ('RR-06F_strip', 'RR-06F TPU tyre half-strip', True)]
SECTIONS = ['1 · Overall dimensions', '2 · Parts and their sizes', '3 · Part drawings with every hole', '4 · Layout: where everything sits',
            '5 · Assembly, step by step', '6 · Wiring', '7 · Bill of materials and status', '8 · Print plates and checklist']
PG = {}                             # heading -> page number, filled from a first build pass


def pg(key):
    return str(PG.get(key, '?'))


def on_page(c, doc):
    if doc.page == 1:
        return
    c.saveState()
    c.setFillColor(NAVY)
    c.rect(0, H - 9 * mm, W, 9 * mm, stroke=0, fill=1)
    c.setFillColor(colors.white)
    c.setFont('SegB', 9)
    c.drawString(M, H - 6 * mm, 'THENAR WALKER  ·  RoboReach 2026-27')
    c.setFont('Seg', 8.5)
    c.drawRightString(W - M, H - 6 * mm, 'Build book: sizes, holes, assembly')
    c.setFillColor(GREY)
    c.setFont('Seg', 7.5)
    c.drawString(M, 8 * mm, 'All sizes in mm. Drawings are not to scale — use the numbers. Generated from the project CAD.')
    c.drawRightString(W - M, 8 * mm, f'page {doc.page}')
    c.restoreState()


# ------------------------------------------------------------------------------------------------ pages
def cover():
    return [Spacer(1, 22 * mm), P('THENAR WALKER', 'big'), P('Build book — RoboReach 2026-27 (Techfest IIT Bombay)', 'sub'),
            Spacer(1, 4 * mm),
            P('Every printed part with its sizes and every hole (size, position, what screw goes in it), the overall robot '
              'dimensions, where each board sits, the assembly steps, wiring, and the full parts list with what the team already has.'),
            Spacer(1, 4 * mm), figure(R('cad', 'renders', 'robot_STOW_iso.png'), 'SolidWorks assembly in the STOW start pose', CW, 95 * mm),
            Spacer(1, 4 * mm),
            table([['Section', 'Page']] + [[h, pg(h)] for h in SECTIONS], [CW * 0.8, CW * 0.2]),
            Spacer(1, 3 * mm),
            callout('<b>Two open items change some drawings:</b> (1) the tub motor slots are drawn for a 25 mm motor; the motors bought are '
                    '35 mm (6 mm shaft, M8 collar) — the tub will be redrawn once the motor is fully measured. (2) The deck bosses fit the '
                    'Cytron MDD10A; if the SHIELD-MDD10 is used instead, they move.', ORANGE),
            PageBreak()]


def dims_page():
    st = [P('1 · Overall dimensions', 'h1'), figure(DR['extra']['robot_dimensions'], 'Robot in the STOW start pose (from design.py and SolidWorks)', CW, 95 * mm),
          Spacer(1, 3 * mm)]
    rows = [['Item', 'Size', 'Rule / note'],
            ['Overall L × W × H (STOW)', '293.7 × 196.0 × 292.9', 'must fit 300 × 200 × 300 at the start'],
            ['Ground clearance (lowest point except wheels)', '57.0 (tub floor)', 'rule ≥ 50. With the 35 mm motor the motor bottom is ~52.5'],
            ['Wheels', 'Ø140 × 28 wide', 'front: one-piece PETG; rear: PETG hub + TPU tyre'],
            ['Wheelbase / track', '152 / 168', 'axle 70 above the ground'],
            ['Chassis tub', '240 × 138 × 41', 'bottom at 57, top at 98'],
            ['Deck plate', '250 × 138 × 5 (+ 6 bosses, 4 fence)', 'top surface at 103'],
            ['Arm base position', 'x = −87 from the centre', 'bolted through 4 of the arm base vent slots'],
            ['Grasp height (REACH_450)', 'object centre at 449.6', 'rule: object centred at 450'],
            ['Floor pick (PICK_FLOOR)', 'grasp centre 25.1 above the ground', '50 mm cube ~254 ahead of the centre'],
            ['Mass (CAD estimate)', '2.40 kg', 'arm included'],
            ['Battery voltage', '11.1 V nominal, 12.6 V full', 'rule ≤ 24 V DC']]
    st.append(table(rows, [CW * 0.38, CW * 0.27, CW * 0.35]))
    st.append(PageBreak())
    return st


def overview_page():
    st = [P('2 · Parts and their sizes', 'h1'), P('Printed parts (PETG unless marked)', 'h2')]
    pr = [['Part', 'Qty', 'Size (L × W × H)', 'Key holes', 'Drawing'],
          ['RR-01 Chassis tub', '1', '240 × 138 × 41', '4 × Ø4 insert bores, motor slots, 20 × Ø2.2 pilots', f"p. {pg('RR-01 Chassis tub — floor')}–{pg('RR-01 Chassis tub — rear wall')}"],
          ['RR-02 Deck plate', '1', '250 × 138 × 11', '8 × Ø3.4, 4 × Ø2.6, 2 × Ø2.8, 2 slots, kill switch cut-out', f"p. {pg('RR-02 Deck plate')}"],
          ['RR-03 Battery cage', '1', '128 × 44 × 27 (inside 108 × 36)', '2 × Ø3.4 tabs, 4 × Ø2.6 bar pilots', f"p. {pg('RR-03 Battery cage — base')}"],
          ['RR-04 Battery bar', '2', '10 × 44 × 3', '2 × Ø3.4', f"p. {pg('RR-04 Battery bar (× 2)')}"],
          ['RR-05 Rear hub (6 mm)', '2', 'Ø131 × 28 (rim Ø128)', 'Ø6.3 bore, 6 × Ø26, M3 grub', f"p. {pg('RR-05 Rear wheel hub (6 mm shaft)')}"],
          ['RR-07 Front wheel (6 mm)', '2', 'Ø140 × 28', 'Ø6.3 bore, 6 × Ø26, M3 grub', f"p. {pg('RR-07 Front wheel (6 mm shaft)')}"],
          ['RR-06F Tyre half-strip (TPU)', '4', '224.3 × 25 × 6', 'Ø4.4 key notch, click joint', f"p. {pg('RR-06F TPU tyre half-strip')}"]]
    st.append(table(pr, [CW * 0.25, CW * 0.06, CW * 0.22, CW * 0.37, CW * 0.10]))
    st.append(P('Bought parts', 'h2'))
    bo = [['Part', 'Size', 'Mounting', 'Note'],
          ['Gear motor 370, plastic gearbox, 12 V ~100 RPM', 'gearbox Ø35; shaft Ø6 round with cross-hole', 'M8 threaded collar + 13 mm nut',
           '<b>To measure:</b> collar length, shaft length, gearbox length, can Ø × length'],
          ['Cytron MDD10A Rev2.0 motor driver', '84.5 × 62 × ~13', '4 × Ø3 holes on 78.74 × 55.88', 'deck top, terminals forward'],
          ['Cytron SHIELD-MDD10 (alternative)', 'Arduino-Uno shield size (~69 × 54)', 'Uno hole pattern', 'measure before use'],
          ['ESP32-DevKitC 38-pin', '55 × 28', '2 × M2.5 standoffs, zip tie', 'under the deck'],
          ['PCA9685 servo driver', '62.2 × 25.4', '4 × M2.5 posts', 'tub floor'],
          ['6 V buck ≥ 10 A (XL4016 class)', '66 × 48 × 18', '4 × M2.5 posts', 'tub floor'],
          ['MP1584 5 V buck', '22 × 17', '4 × M2.5 posts', 'tub rear corner'],
          ['3S LiPo 2200 mAh, XT60', 'max 106 × 34 × 26', 'in the cage, 2 bars on top', ''],
          ['KCD4 rocker (kill switch)', 'cut-out 30.2 × 22.2, body 32 deep', 'snaps into the deck', 'front right'],
          ['MG996R servo (arm, ×6)', '40.7 × 19.7 × 42.9', 'arm parts', 'thenar-arms R3']]
    st.append(table(bo, [CW * 0.27, CW * 0.25, CW * 0.22, CW * 0.26]))
    st.append(PageBreak())
    return st


def drawings_pages():
    st = [P('3 · Part drawings with every hole', 'h1'),
          P('Each drawing is a slice through the real 3-D part. Holes are lettered by size (A = smallest); the table says what each is '
            'for, and the X / Y list gives every hole centre measured from the orange 0,0 corner (or from the centre for wheels). '
            'Check a printed part with a caliper against these numbers.')]
    order = DRAW_ORDER
    for name, title, brk in order:
        block = [P(title, 'h2')] + hole_block(name)
        if name == 'RR-01_floor':
            block.insert(1, callout('<b>On hold:</b> the tub is drawn for the 25 mm motor. The motor slots, side-wall holes and floor windows '
                                    'will change for the 35 mm motor, and the battery-cage screw holes (half inside the motor windows, '
                                    'marked C1 / C2 notch) will move inward. Do not print plate 1 until the redesign.', ORANGE))
        st.append(KeepTogether(block))
        st.append(PageBreak() if brk else Spacer(1, 4 * mm))
    return st


def layout_pages():
    st = [P('4 · Layout: where everything sits', 'h1'), P('Tub floor (deck removed)', 'h2'),
          figure(DR['extra']['layout_floor'], 'Top view. Positions are board centres from the tub centre; +X forward, +Y to the robot left.', CW, 100 * mm)]
    rows = [['Item', 'Centre X', 'Centre Y', 'Size', 'Mounted on']]
    names = {'BUCK6V': '6 V buck', 'PCA9685': 'PCA9685', 'BUCK5V': '5 V buck (MP1584)', 'ESP32': 'ESP32', 'MDD10A': 'Motor driver MDD10A'}
    mounts = {'floor': 'floor posts (3 mm)', 'deck': 'under the deck (10 mm standoffs)', 'deck_top': 'deck top (6 mm bosses)'}
    for k, (x, y, rot, mnt) in D.BOARDS.items():
        L_, W_ = D.BOARD_SIZE[k]
        rows.append([names[k], f'{x:.1f}', f'{y:.1f}', f'{L_} × {W_}' + (' (turned 90°)' if rot % 180 == 90 else ''), mounts[mnt]])
    rows.append(['Battery (in the cage)', f'{D.BATT_C[0]:.1f}', f'{D.BATT_C[1]:.1f}', f'{D.BATT_L:.0f} × {D.BATT_W:.0f}', 'cage on the floor'])
    rows.append(['Kill switch', f'{D.KILL_XY[0]:.1f}', f'{D.KILL_XY[1]:.1f}', '30.2 × 22.2 cut-out', 'deck'])
    rows.append(['Arm base origin', f'{D.ARM_X:.1f}', '0.0', 'fence inside 88 × 112', 'deck'])
    st.append(table(rows, [CW * 0.28, CW * 0.12, CW * 0.12, CW * 0.22, CW * 0.26]))
    st += [PageBreak(), P('Deck top', 'h2'),
           figure(DR['extra']['layout_deck'], 'Top view of the deck. The driver terminals face forward over the wire slot.', CW, 100 * mm),
           P('Motor wheels: front-left, rear-left, front-right, rear-right motors sit at X = ±76 (wheelbase 152), axle 70 above the ground, '
             'shafts pointing outward through the side walls.', 'b'),
           PageBreak()]
    return st


def assembly_pages():
    steps = [
        ('0 · Before you start', ['Measure one motor: collar length, shaft length past the collar, gearbox length, can Ø × length. The tub is redrawn from these.',
                                  'Decide the driver: MDD10A (deck ready) or SHIELD-MDD10 (deck bosses move).',
                                  'Bottle test done: one motor lifted 750 ml at 7 cm (≥ ~5.2 kg·cm), enough for the course.'],
         'Caliper, the motor, the drivers'),
        ('1 · Print', ['PETG: plate 1 (tub, cage, bars — after the redesign), plate 2 (deck), plates 3 + 4b (wheels). TPU: plate 5F (tyre strips) ×1, ×2 for spares.',
                       '0.2 mm layers, 4 walls, 30 % gyroid (TPU: 3 walls, 20 %), no supports. TPU from the external spool, not the AMS.'],
         'P1S, ~0.8 kg PETG, ~85 g TPU per tyre plate'),
        ('2 · Clean up the prints', ['Run a 6 mm drill through each wheel bore by hand; drill the 2 older 4 mm-bore wheels out to 6 mm.',
                                     'Tap or self-thread the Ø2.6 grub holes with an M3 grub screw.',
                                     'Check holes against the drawings (section 3) with a caliper.'],
         '6 mm drill, M3 tap (optional)'),
        ('3 · Tub', ['Melt 4 × M3 heat-set inserts into the corner bores (Ø4.0, 8 deep).',
                     'Screw the battery cage to the floor: 2 × M3 × 8 + nyloc from below.',
                     'Mount the 4 motors in the side walls, shafts out (new mount for the 35 mm motor). Solder leads first.',
                     'Screw the 6 V buck, PCA9685 and 5 V buck to their floor posts (M2.5 × 6 self-tapping).',
                     'Set the 6 V buck to 6.0 V and the 5 V buck to 5.0 V with a multimeter, before connecting anything.'],
         '4 inserts, 2 × M3×8 + nyloc, ~12 × M2.5×6, motor mounts'),
        ('4 · Wheels and tyres', ['Rear: click two TPU half-strips together, superglue the joint, wrap around a rear hub between the lips '
                                  '(grooves out, notch over the grub screw), pull the second joint closed and glue.',
                                  'Push each wheel onto its shaft ~1 mm off the wall; turn it so the grub screw lines up with the shaft cross-hole; tighten the M3 × 5 grub.',
                                  'Front wheels (hard, one piece) at the front, TPU-tyred wheels at the rear. Spin each wheel by hand: no rubbing.'],
         '4 × M3×5 grub, superglue'),
        ('5 · Deck', ['Screw 2 × M2.5 × 10 brass standoffs under the deck, zip-tie the ESP32 to them (USB toward the rear window).',
                      'Snap the kill switch into its cut-out (front right) and wire it in series with battery +.',
                      'Motor driver on the 4 bosses (M3 × 10 self-tapping), blue terminals facing forward.',
                      'Route: servo leads + 5 driver signal wires up through the slot by the arm; motor + power wires up through the slot in front of the driver.'],
         '2 standoffs, 4 × M3×10 self-tap, zip ties'),
        ('6 · Close up and fit the arm', ['Battery into the cage, lead out of the front window, screw on the two bars (M3 × 8 self-tapping).',
                                          'Deck onto the tub: 4 × M3 × 10 into the inserts.',
                                          'Arm base inside the fence, 4 × M3 × 20 + washers through the base vent slots, nyloc nuts under the deck.',
                                          'Servos J1…J6 into PCA9685 channels 0…5.'],
         '4 × M3×8 self-tap, 4 × M3×10, 4 × M3×20 + nyloc'),
        ('7 · Firmware and first power-up', ['python firmware/tools/new_key.py "Team Name" → flash thenar_walker_bot (robot) and thenar_walker_controller (leader).',
                                             'Wheels off the ground, kill switch in reach. Use the driver test buttons (M1A/M1B/M2A/M2B) first.',
                                             'Joystick check: if a whole side runs backwards swap that side\'s two driver wires.',
                                             'Calibrate the servos (calibration.h), set FOLLOWER_CALIBRATED = true, reflash.',
                                             'Failsafe test: switch the controller off while holding a cube → wheels stop, arm freezes, gripper holds.'],
         'USB cable, multimeter'),
    ]
    st = [P('5 · Assembly, step by step', 'h1')]
    for title, items, tools in steps:
        rows = [[P(f'<b>{title}</b>', 'b'), P('<i>' + tools + '</i>', 'small')]] + [[P('• ' + i, 'b'), ''] for i in items]
        t = Table(rows, colWidths=[CW * 0.74, CW * 0.26])
        t.setStyle(TableStyle([('SPAN', (0, k), (1, k)) for k in range(1, len(rows))] +
                              [('BACKGROUND', (0, 0), (-1, 0), LIGHT), ('VALIGN', (0, 0), (-1, -1), 'TOP'),
                               ('LEFTPADDING', (0, 0), (-1, -1), 5), ('TOPPADDING', (0, 0), (-1, -1), 1.5), ('BOTTOMPADDING', (0, 0), (-1, -1), 1.5),
                               ('BOX', (0, 0), (-1, -1), 0.4, colors.HexColor('#c9d1dc'))]))
        st += [KeepTogether([t]), Spacer(1, 2.5 * mm)]
    st += [PageBreak()]
    return st


def wiring_page():
    st = [P('6 · Wiring', 'h1'), T.power_tree(), Spacer(1, 2 * mm),
          P('Motors: both left motors in parallel on channel 1, both right motors on channel 2 — same wire of each motor into the same terminal.', 'b')]
    rows = [['ESP32 pin', 'Signal', 'MDD10A (white 5-pin plug)', 'SHIELD-MDD10 (jumper-selected)'],
            ['GPIO25', 'left speed', 'PWM1 (pin 4)', 'PWM1 pin, e.g. D3'], ['GPIO26', 'left direction', 'DIR1 (pin 5)', 'DIR1 pin, e.g. D2'],
            ['GPIO16', 'right speed', 'PWM2 (pin 2)', 'PWM2 pin, e.g. D6'], ['GPIO17', 'right direction', 'DIR2 (pin 3)', 'DIR2 pin, e.g. D7'],
            ['GND', 'ground', 'GND (pin 1)', 'any GND'], ['GPIO21 / 22', 'I²C', 'PCA9685 SDA / SCL', ''],
            ['GPIO13', 'servo enable', 'PCA9685 OE', ''], ['GPIO34', 'battery sense', '100 kΩ / 27 kΩ divider', ''],
            ['GPIO2', 'status LED', 'on-board', 'solid = ACTIVE']]
    st.append(table(rows, [CW * 0.16, CW * 0.18, CW * 0.33, CW * 0.33]))
    st.append(Spacer(1, 2 * mm))
    st.append(callout('10 kΩ from PWM1 to GND and from PWM2 to GND (no twitch while the ESP32 boots). Fuse 10 A, close to the XT60. '
                      'Kill switch before everything. Never power the servos from the ESP32 or USB.', ORANGE))
    st.append(PageBreak())
    return st


def bom_pages():
    rows = [['Part', 'Qty', 'Spec', 'Status'],
            ['370 gear motor, plastic gearbox, 12 V ~100 RPM', '4 (+1)', 'Ø35 gearbox, 6 mm shaft, M8 collar; bottle test passed', 'Have'],
            ['M3 × 5 cup-point grub screw', '4', 'wheel → shaft', 'To buy'],
            ['22 AWG silicone wire red + black', '2 m each', 'motor leads', 'Confirm'],
            ['Cytron MDD10A Rev2.0', '1', 'dual 10 A, PWM + DIR', 'Decide · have'],
            ['Cytron SHIELD-MDD10', '1', 'same driver as an Arduino shield', 'Decide · have'],
            ['10 kΩ resistor', '2', 'PWM pull-downs', 'To buy'],
            ['3S LiPo 2200 mAh 30C XT60', '1 (+1)', 'max 106 × 34 × 26', 'Confirm'],
            ['LiPo balance charger + safe bag', '1', '3S balance', 'Confirm'],
            ['XT60 pair', '2', '', 'Have'],
            ['Glass fuse holder 5 × 20 + 10 A fuse', '1', 'inline at the XT60', 'Have'],
            ['LiPo cell tester / buzzer', '1', 'on the balance lead', 'Have'],
            ['KCD4 rocker 16 A+ (kill switch)', '1', 'cut-out 30.2 × 22.2', 'Confirm'],
            ['6 V buck ≥ 10 A', '1', 'servo rail', 'Confirm'],
            ['MP1584 5 V buck', '1', 'ESP32 supply', 'Confirm'],
            ['16 AWG silicone wire red + black', '1 m each', 'power', 'Confirm'],
            ['100 kΩ + 27 kΩ resistors', '1 each', 'battery sense', 'To buy'],
            ['ESP32-DevKitC 38-pin', '1', 'robot', 'Confirm'],
            ['PCA9685', '1', 'servo driver', 'Confirm'],
            ['Jumper wires, servo extensions, heat-shrink, zip ties', '1 lot', '', 'Confirm'],
            ['thenar-arms follower R3 (6 × MG996R)', '1', 'the arm', 'Have'],
            ['thenar-arms L1 leader (controller)', '1', '', 'Have'],
            ['Joystick module, ENABLE toggle, SPEED button, power bank', '1 each', 'controller add-ons', 'Confirm'],
            ['M3 heat-set inserts', '4', 'tub corners', 'Confirm'],
            ['M3 × 10 socket screws', '4', 'deck → tub', 'Confirm'],
            ['M3 × 10 self-tapping screws', '4', 'driver → deck', 'To buy'],
            ['M3 × 8 screws + nyloc', '2 + 2', 'cage → floor', 'Confirm'],
            ['M3 × 8 self-tapping screws', '4', 'bars → cage', 'Confirm'],
            ['M3 × 20 screws + washers + nyloc', '4', 'arm → deck', 'Confirm'],
            ['M2.5 × 10 brass standoffs + M2.5 × 6 screws', '2 + 2', 'ESP32', 'Confirm'],
            ['M2.5 × 6 self-tapping screws', '~12', 'boards → floor', 'Confirm'],
            ['Superglue (CA)', '1', 'tyre strip joints', 'To buy'],
            ['Plate 1: tub + cage + bars', '1', 'PETG', 'On hold'],
            ['Plate 2: deck', '1', 'PETG', 'Ready'],
            ['Plate 3: front wheel + rear hub', '1', 'PETG (drill to 6 mm)', 'Printed'],
            ['Plate 4b: front wheel + rear hub, 6 mm', '1', 'PETG', 'Printed'],
            ['Plate 5F: 4 TPU strips = 2 tyres', '1 (+1 spare)', 'TPU 95A', 'Printing'],
            ['25GA-370 motor, MDD3A, TB6612, white plastic wheel', '–', 'earlier plan / unsuitable', 'Not used']]
    return [P('7 · Bill of materials and status', 'h1'),
            P('Have = bought / built. To buy = still needed. Confirm = not reported yet. Decide = two options, only one needed.', 'small'),
            styled(rows, [CW * 0.42, CW * 0.11, CW * 0.31, CW * 0.16], colour_col=3), PageBreak()]


def print_page():
    rows = [['Plate', 'Parts', 'Material', 'Time', 'Filament', 'Status'],
            ['1', 'tub, cage, 2 bars', 'PETG', '~7.1 h', '~259 g', 'On hold'],
            ['2', 'deck', 'PETG', '3 h 27 min', '117 g', 'Ready'],
            ['3', 'front wheel + rear hub (4 mm)', 'PETG', '6 h 08 min', '202 g', 'Printed'],
            ['4b', 'front wheel + rear hub (6 mm)', 'PETG', '6 h 07 min', '202 g', 'Printed'],
            ['5F', '4 tyre strips = 2 tyres', 'TPU 95A', '5 h 40 min', '84 g', 'Printing']]
    chk = [['Before every run', ''], ['Battery ≥ 11.4 V (controller shows batt)', '☐'], ['Kill switch reachable and working', '☐'],
           ['team_config.h generated (not the example)', '☐'], ['Arm folded to STOW, robot inside 300 × 200 × 300', '☐'],
           ['All 4 grub screws tight, tyre joints intact', '☐'], ['Failsafe test: controller off → robot stops, gripper holds', '☐']]
    t = table(chk, [CW * 0.85, CW * 0.15])
    t.setStyle(TableStyle([('FONTNAME', (1, 1), (1, -1), 'SegSym')]))
    return [P('8 · Print plates and checklist', 'h1'), styled(rows, [CW * 0.08, CW * 0.36, CW * 0.12, CW * 0.14, CW * 0.12, CW * 0.18], colour_col=5),
            P('PETG: 0.2 mm, 4 walls, 30 % gyroid, textured PEI, no supports. TPU: 3 walls, 20 %, external spool, dried. Sliced files: cad/print/sliced/.', 'small'),
            Spacer(1, 4 * mm), t]


def build():
    doc = SimpleDocTemplate(OUT, pagesize=T.A4, leftMargin=M, rightMargin=M, topMargin=14 * mm, bottomMargin=14 * mm,
                            title='Thenar Walker — build book', author='Thenar Walker team')
    story = cover() + dims_page() + overview_page() + drawings_pages() + layout_pages() + assembly_pages() + wiring_page() + bom_pages() + print_page()
    doc.build(story, onFirstPage=on_page, onLaterPages=on_page)


def find_pages():
    import fitz
    with fitz.open(OUT) as d:
        texts = [' '.join(p.get_text().split()) for p in d]
    for k in SECTIONS:
        for i, t in enumerate(texts[1:], 2):          # skip the cover: its contents table repeats the names
            if k in t:
                PG[k] = i
                break
    start = PG.get(SECTIONS[2], 2)                    # part titles also appear in the overview table
    for _, k, _ in DRAW_ORDER:
        for i, t in enumerate(texts[start - 1:], start):
            if k in t:
                PG[k] = i
                break
    return len(texts)


def main():
    build()
    find_pages()
    build()                                            # second pass prints the real page numbers
    print('wrote', OUT, find_pages(), 'pages')


if __name__ == '__main__':
    main()
