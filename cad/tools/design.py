"""Thenar Walker — single source of truth for every dimension (mm).

Robot frame: origin on the ground under the chassis centre, +X forward,
+Y left, +Z up. SolidWorks Front Plane = XY, so the CAD uses this frame as is.

Every rule from the Techfest 2026-27 RoboReach statement that drives geometry
is listed in RULES and re-checked by verify_robot.py against the SolidWorks
assembly, not against these numbers.
"""
import os

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
CAD = os.path.join(ROOT, 'cad')
PARTS = os.path.join(CAD, 'parts')
ASM = os.path.join(CAD, 'assemblies')
ARENA = os.path.join(CAD, 'arena')
VENDOR = os.path.join(CAD, 'vendor', 'thenar-arms')
RENDERS = os.path.join(CAD, 'renders')
EVIDENCE = os.path.join(CAD, 'evidence')

RULES = {
    'start_box_lwh': (300.0, 200.0, 300.0),   # bot must fit at the start
    'min_ground_clearance': 50.0,             # lowest non-wheel point
    'grasp_centre_height': 450.0,             # arm must grasp an object centred here
    'max_voltage': 24.0,
    'ramp_deg': 15.0,
    'speed_breaker_peak': 40.0,
    'bridge_width': 400.0,
    'diagonal_track_width': 250.0,
}

# ---------------------------------------------------------------- drive
WHEEL_D = 140.0           # printed PETG hub + TPU tyre
WHEEL_W = 28.0
TYRE_T = 6.0              # radial TPU thickness -> hub rim OD = 128
AXLE_Z = WHEEL_D / 2      # 70
AXLE_X = 76.0             # wheelbase 152
WHEEL_YC = 84.0           # wheel centre plane; outer face at 98 -> overall width 196
WHEEL_INNER = WHEEL_YC - WHEEL_W / 2   # 70

# 25GA-370 (= JGA25-370) 12 V 103:1, 60 rpm no-load: rated 2.7 kg.cm @ 46 rpm, stall 8.2 kg.cm / 1.3 A (analysis/drive_sizing.py)
# gearbox of the 103:1 version is ~23 mm (CAD envelope 22): facing motors still keep a ~20 mm gap
MOTOR_GB_D = 25.0
MOTOR_GB_L = 22.0
MOTOR_CAN_D = 24.4
MOTOR_CAN_L = 31.0
MOTOR_CAP_L = 3.0
MOTOR_SHAFT_D = 4.0
MOTOR_SHAFT_L = 10.0
SHAFT6_BORE = 6.3         # wheel variant for the local 370 plastic-gearbox motor (6 mm round shaft, cross-hole)
MOTOR_BOSS_D = 7.0
MOTOR_BOSS_L = 2.0
MOTOR_HOLE_PITCH = 17.0   # 2 x M3 on the gearbox face
MOTOR_FACE_Y = 67.0       # gearbox face plane (|y|)

# ---------------------------------------------------------------- chassis tub (RR-01)
TUB_X = 120.0             # half length -> 240 (fits Bambu P1S 256 bed)
TUB_Y = 69.0              # half width outer -> 138 (1 mm to the wheel)
FLOOR_Z0 = 57.0           # underside = ground clearance (rule >= 50)
FLOOR_T = 3.0
WALL_SIDE_T = 5.0         # side walls carry the motors
WALL_END_T = 4.0
TUB_TOP_Z = 98.0          # low deck: arm stows inside the 300 mm start height
POCKET_D = 26.0           # motor gearbox pocket in side wall
BOSS_D = 9.0              # heat-set insert columns
BOSS_XY = [(-112, 63), (-112, -63), (112, 63), (112, -63)]
MOTOR_WINDOW = (62.0, 90.0, 9.0, 64.5)    # |x| range, |y| range of floor windows for motors
# engraved logos on the printed tub (outer faces of the end walls, read from outside): text, wall, centre y, centre z, cap height
LOGOS = [('THENAR.IO', 'front', 0.0, 78.0, 13.0), ('THENARLABS', 'rear', 0.0, 68.0, 9.0)]
LOGO_DEPTH = 1.0

# ---------------------------------------------------------------- deck (RR-02)
DECK_X = 125.0            # half length -> 250
DECK_Y = 69.0
DECK_Z0 = TUB_TOP_Z
DECK_T = 5.0
DECK_TOP = DECK_Z0 + DECK_T          # 103

# ---------------------------------------------------------------- arm mount
ARM_X = -87.0                         # arm base_link origin, robot frame (collision-searched, analysis/stow_search4.py)
ARM_BASE_DROP = 2.4                   # base underside is 2.4 below base_link z=0
ARM_Z = DECK_TOP + ARM_BASE_DROP      # 105.4
ARM_FOOT = (-22.4, 64.7, -55.5, 55.5)  # base extents within the fence height, base_link frame (x0,x1,y0,y1)
ARM_SLOTS = [(-14.2, 31.7), (-14.2, -31.7), (55.5, 27.8), (55.5, -27.8)]  # M3 through base vent slots
FENCE_H = 4.0
FENCE_T = 3.0

# Arm poses (degrees, thenar-arms logical joint angles; URDF/firmware convention)
POSES = {   # collision-free vs chassis + self (python-fcl), re-verified in SolidWorks
    # grasp centre = URDF tool point - (w/2 + 1.5 mm) along the closing axis (tool point is on the fixed finger)
    'STOW':       [0, -80, 80, 50, 85, 0],
    'HOME':       [0, -25, 35, 0, 0, 20],
    'REACH_450':  [-3.4, 12.0, -73.8, 61.3, -85.0, 35],   # 50 mm object centred at 449.8 mm, jaws closing sideways
    'PICK_FLOOR': [0, 80, -68.3, 76.7, 3.4, 35],          # 50 mm floor cube, grasp centre z 25.1 mm
}

# ---------------------------------------------------------------- electronics on the tub floor
STANDOFF = 3.0
# name: (cx, cy, rot_deg about Z, mount) ; mount 'floor' = on standoffs on the floor, 'deck' = hung under the deck,
# 'deck_top' = on printed bosses on top of the deck
BOARDS = {
    'BUCK6V':  (-28.0, -38.0, 0, 'floor'),
    # Cytron MDD10A Rev2.0 (84.5 x 62, too big for the floor next to the 6 V buck): deck top, front-left, outside the
    # stowed arm. Rotated 180 deg so the screw terminals face forward, over the wire slot in front of the board.
    'MDD10A':  (52.5, 35.0, 180, 'deck_top'),
    'PCA9685': (-30.0, 3.0, 0, 'floor'),
    'BUCK5V':  (-102.0, 40.0, 90, 'floor'),
    'ESP32':   (-30.0, -30.0, 0, 'deck'),   # hung under the deck on 10 mm posts, above the 6 V buck
}
BOARD_SIZE = {'BUCK6V': (66, 48), 'MDD10A': (84.5, 62), 'PCA9685': (62.2, 25.4), 'BUCK5V': (22, 17), 'ESP32': (55, 28)}

MDD10A_HOLES = (78.74, 55.88)          # mounting-hole pitch, Cytron MDD10A user manual V2.0 p.6 (holes ~3 mm)
DECK_BOSS_H = 6.0                      # printed bosses on the deck top: solder joints clear the deck
MOTOR_WIRE_SLOT = (98.0, 110.0, 22.0, 52.0)   # deck cut (x0, x1, y0, y1) in front of the MDD10A terminals
# floor pads where the two MDD3A boards were: kept so the already-shared tub (plate 1) stays identical
SPARE_FLOOR_PADS = {'PAD_A': (37.0, -42.0, 45, 40), 'PAD_B': (37.0, -1.0, 45, 40)}

# ---------------------------------------------------------------- battery (3S LiPo 2200 mAh, nominal envelope)
BATT_L, BATT_W, BATT_H = 106.0, 34.0, 26.0
BATT_C = (0.0, 42.0)                   # centre x,y (left half of the floor)
CRADLE_T = 4.0
CRADLE_H = 27.0                        # walls 60..87; battery top 86; bars 87..90; deck underside 98

# ---------------------------------------------------------------- kill switch on the deck front overhang
KILL_XY = (-9.0, 47.0)          # v2: moved off the front-right motor (tub_v2.py)
KILL_CUT = (30.2, 22.2)                # KCD4-style 30 A rocker panel cut-out

# ---------------------------------------------------------------- masses (g) — datasheet / estimate, used for COG + tipping
MASS = {
    'motor_jga25': 95, 'wheel_hub': 55, 'wheel_tyre': 35, 'battery_3s2200': 185,
    'esp32': 10, 'pca9685': 9, 'mdd10a': 45, 'buck_6v': 60, 'buck_5v': 4, 'kill_switch': 18,
    'wiring_misc': 120, 'fasteners': 80,
    'mg996r': 55, 'horn': 6,
}
PETG_G_CC = 1.27
PRINT_FILL = 0.55                      # effective solid fraction for 4 walls + 30 % infill
GRASP_OFFSET_50 = 26.5                 # mm, 50 mm object centre from the tool point along -x_tool
