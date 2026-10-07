# Design notes — Thenar Walker

A 12 V four-wheel skid-steer base that carries the **thenar-arms MG996R follower R3** unchanged,
teleoperated by the **thenar-arms L1 encoder leader** plus a thumb joystick, for Techfest 2026-27
RoboReach. Every number below is produced by a script in this repo; `docs/VERIFICATION.md` holds the
measured results.

## Rules that set the geometry

| Rule (RoboReach PS) | Consequence |
|---|---|
| Fits 300 L × 200 W × 300 H at the start | Arm must fold; base ≤ 300 × 200 including wheels |
| Arm grasps an object centred at 450 mm with the bot on the ground | Arm base height + reach |
| Ground clearance ≥ 50 mm (lowest point to ground) | Big wheels; motors on the axle line stay above 50 mm |
| Wheels in continuous contact | No legs/tracks over obstacles; Ø140 wheels roll over 40 mm breakers |
| ≤ 24 V, on-board battery, secured battery, kill switch | 3S LiPo (12.6 V max), printed battery cage, 30 A rocker in series |
| Wi-Fi/BT, SSID `TeamName_TF`, authentication, failsafe on disconnect | ESP32 AP + HMAC-signed UDP + HOLD state (see WIRING.md) |
| Mechanical gripper only | thenar-arms jaw gripper, no adhesive/magnet/suction |

## Layout (robot frame: X forward, Y left, Z up, origin on the ground at the chassis centre)

* **Wheels:** Ø140 (printed PETG hub + TPU tyre), 28 wide, centres at x = ±76, y = ±84.
  Overall 293.7 × 196 mm. A Ø140 wheel puts the axle at 70 mm, so a Ø25 JGA25 motor mounted
  coaxially bottoms out at 57.5 mm — above the 50 mm clearance rule without any belts or gears.
* **Motors:** gearbox faces screw to the inside of the tub side walls (U-slot pockets that open
  through the floor so the cans drop in). Left and right motors face each other with a 24 mm gap.
* **Tub (RR-01):** 240 × 138 mm, floor underside at **57 mm** = lowest point of the robot.
  Floor windows let the motor cans sit lower than the floor top.
* **Deck (RR-02):** top at **103 mm**. Electronics sit on the tub floor in one layer
  (battery cage on the left half; 6 V buck and PCA9685 on the right half; 5 V buck
  in the rear end bay; ESP32 hung under the deck above the 6 V buck). The Cytron MDD10A motor
  driver (84.5 × 62 mm, too big for the floor) sits on 4 bosses on the deck top, front-left, outside
  the stowed arm's path, with a wire slot in front of its terminals.
* **Arm:** base bolted to the deck at x = −87 through four of its own vent slots (no change to
  the thenar-arms parts), inside a 4 mm locating fence.

### Why the deck is so low (the 300 mm start height)

The first layout put the deck at 135 mm with an electronics tray. SolidWorks interference
detection then showed the folded arm colliding with itself (up to 9.6 cm³). A collision-aware
search over the arm's joint limits (`analysis/stow_search*.py`, python-fcl on the URDF meshes,
servo-horn spline overlap allowed) proved the lowest collision-free fold inside the ±80° software
limits is 187.5 mm tall above the arm's base frame (about 190 mm above the base underside). With a 135 mm deck that is 325 mm — over the rule. So the deck
came down to 103 mm and the electronics moved to the floor:

* **STOW pose:** pan 0, lift −80, elbow 80, wrist flex 50, wrist roll 85, gripper 0.
  Robot = 293.7 × 196 × 292.9 mm (6 / 4 / 7 mm margin), no self- or chassis collision.
  Wrist flex 45–55 are all clear, so ±5° of servo error is tolerated.
* The thenar-arms joint limits were **not** changed.

## Arm poses on the base (all collision-checked, then verified in SolidWorks)

The URDF `gripper_frame_link` ("tool") sits on the **fixed finger**, not between the jaws: a held
50 mm object is centred 26.5 mm from it along the closing axis. All grasp heights below are that true
object centre (`analysis/poses3.py`); an earlier draft quoted the tool point and was 25 mm off.

| Pose | Joints (deg) | What it shows |
|---|---|---|
| STOW | 0, −80, 80, 50, 85, 0 | start box |
| HOME | 0, −25, 35, 0, 0, 20 | thenar-arms home; carrying pose |
| REACH_450 | −3.4, 12, −73.8, 61.3, −85, 35 | 50 mm object centred at 449.6 mm (SolidWorks), gripper horizontal, jaws closing sideways, 260 mm ahead of centre |
| PICK_FLOOR | 0, 80, −68.3, 76.7, 3.4, 35 | 50 mm floor cube (centre z 25.1 mm) 254 mm ahead of centre |

The 400 mm Downy pole, the ≤ 400 mm buzz wire and the 340 mm stacker tower are inside the workspace;
the simulation (docs/SIMULATION.md) actually takes the 60 mm cube off the 400 mm pole.

## Drive sizing (`analysis/drive_sizing.py`)

Mass 2.37 kg from the CAD, sized at 2.61 kg. On the 15° ramp with 0.5 m/s² launch:
force 9.2 N → 0.64 N·m at the wheels → **1.63 kg·cm per motor** with four driving,
**3.27 kg·cm** if a breaker lifts one axle. The motor to buy in India is the **25GA-370 12 V 60 RPM (103:1)**:
rated 2.7, stall 8.2 kg·cm → margins 1.6× rated (4WD) and 2.5× to stall (2 wheels). The 100 rpm version
(stall ~3.6 kg·cm) would stall on the ramp — don't substitute it. Speed ~0.34 m/s loaded. Firmware speed modes: 35 % precision / 65 % / 100 % for driving.

**Steering:** skid steer (tank / JCB style) — no steering servo or shaft. One Cytron MDD10A drives both left wheels on
channel 1 and both right wheels on channel 2; the ESP32 mixes the joystick (left = throttle − turn, right = throttle + turn).
Different side speeds curve the robot, opposite directions spin it on the spot.

**Turning:** wheelbase 152 ≈ track 168, so every turn scrubs tyres sideways. With TPU on all four
wheels the simulation showed no spin below ~55 % power and arc turns of only 2.8°/s (it reversed 2 m
while turning 26°) — this is why the first video's smooth spin looked wrong. Stronger, slower motors
barely help. Fix: **PETG (hard) tyres at the front, TPU at the rear** → spin 20/77/136°/s at 35/65/100 %,
arcs 14°/s, ramp and speed breakers still pass (PETG at the rear stalls on the breakers). Firmware gives
rotation its own 50/75/100 % scale with a 35 % breakaway floor.

## Stability

COG at 104–149 mm height depending on pose. Worst static margin is 19 mm (PICK_FLOOR on a 15°
descent); carrying the cube in HOME on the descent leaves 33 mm. Drive rule: **carry the cube in
HOME (arm tucked) over the ramp, bridge and breakers**, don't descend with the arm stretched out.
The firmware ramps acceleration (full speed in 0.4 s) but stops immediately.

## Obstacles (verified in SolidWorks on the arena's own geometry, and driven in MuJoCo)

Ramp 15° to 80 mm, 400 mm bridge, ramp down, then four R40 half-cylinder breakers at 100 mm pitch.
Nine solved poses (ramp foot, on ramp, crest, bridge, bridge exit, down-ramp foot, front wheels on
the first breaker, breaker under the belly, rear wheels on the last breaker): only the tyres
touch the obstacles. Breakover angle is 2·atan(57/76) ≈ 74°; the tub ends lie inside the wheel
silhouettes, so approach/departure angles are effectively 90°.

The diagonals corridor is 250 mm and the robot is 196 mm wide; it cannot spin on the spot inside it
(needs a 352 mm circle), so drive the corridor in arcs.

## Electronics and failsafe

ESP32 robot controller → PCA9685 (servos) + Cytron MDD10A (motors, on the deck top), powered from a 3S LiPo through a
15 A fuse and the kill switch. The controller is the leader ESP32. Link: WPA2 AP `<Team>_TF`,
one station, HMAC-SHA256 on every packet, random session id per pairing, monotonic sequence numbers.
HOLD on 250 ms of silence, operator disable, bad target or encoder fault: wheels 0, arm frozen,
gripper PWM kept on. The original thenar-arms follower turned servo outputs OFF on watchdog
timeout (OE high), which would drop the cube and break the RoboReach disconnect rule — this
firmware keeps them on and only freezes motion.

## Known limits / to measure on the real parts

* Motor shaft ≥ 10 mm and flat orientation; hub grub screw lands on the flat.
* Gripper does not fully close (19.7 mm at 0°): add TPU finger pads for the marker task.
* Board footprints and mounting holes are nominal — check before printing the tub.
* Arena positions are reconstructed from the rulebook renders; dimensions of the obstacles
  themselves follow the rulebook.
