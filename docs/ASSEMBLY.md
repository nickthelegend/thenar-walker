# Build guide — Thenar Walker

Open `cad/assemblies/Thenar_Walker.SLDASM` (configuration **STOW**) beside this guide.
Part numbers (RR-xx printed, P-xx purchased) match the CAD and `docs/BOM.md`.

## 0. Before printing
1. Measure your JGA25 motor: gearbox Ø25, shaft 4 mm D, shaft length from the face ≥ 9 mm,
   face holes 2 × M3 at 17 mm. If different, change `cad/tools/design.py` and rebuild
   (`python cad/tools/build_parts.py`).
2. Lay your boards on the CAD footprints (`design.BOARDS`) — move any that differ.
3. **Motor torque test (before buying all four).** Sellers' torque figures for the "25GA-370 12 V 60 RPM"
   disagree, and the robot needs ≥ 6 kg·cm stall. Print one wheel, put it on the motor, tie a string to
   the tyre (7 cm radius) and hang a filled 1 L water bottle (~1 kg = 7 kg·cm). At 12 V the motor must
   lift it. If it stalls with a 500 ml bottle, it's the weak version — return it.
   **100 RPM version** (simulated 2026-10-05, `sim/out/run_100rpm_*.txt`): it passes the whole course only if it is
   at least ~4.9 kg·cm at stall (4.5 gets stuck on the speed breakers). Test: it must lift a **750 ml** bottle on the
   7 cm string at 12 V. Drive the speed breakers in normal/fast mode (momentum helps) and turn near the poles in
   precision mode (in normal mode it is fast enough to brush the corridor wall).
   Easiest with the MDD10A: battery → B+/B−, motor → M1A/M1B, press the **M1A** test button (no ESP32 needed).

## 1. Print
Print the 5 ready plates in `cad/print/` (see `cad/print/PRINT.md`): tub + cage + bars, deck, 2 × (front wheel + rear hub),
and the 2 TPU rear tyres. Plus the seven thenar-arms R3 arm parts.

## 2. Tub
1. Melt 4 × M3 heat-set inserts into the corner columns.
2. Screw the battery cage (RR-03) to the floor: 2 × M3 × 8 + nyloc from below.
3. Drop each motor into its U-slot from above, gearbox face against the wall, terminals inward.
   2 × M3 × 6 through the wall into the gearbox face. Solder leads first.
4. Screw the 6 V buck, PCA9685 and the 5 V buck to their floor standoffs (M2.5 self-tap).
   The two empty standoff pads in front of the buck (where the old MDD3A boards were) are spare:
   use them for the fuse holder or a wire tie-down.
5. **Set the 6 V buck to 6.0 V and the 5 V buck to 5.0 V on the bench before connecting anything.**
6. Wire per `docs/WIRING.md`: XT60 → fuse → kill switch → VBAT bus. Leave the kill switch OFF.

## 3. Wheels
1. Rear: stretch a TPU tyre over each RR-05 hub between the bead lips. Front: the RR-07 wheels are one piece.
2. Push each wheel on the D-shaft until it is ~1 mm off the wall; tighten the M3 grub screw onto the
   flat through the hole in the tread.

## 4. Deck and arm
1. Screw two M2.5 × 10 brass standoffs under the deck and zip-tie the robot ESP32 to them (USB towards the rear).
2. Snap the kill switch into the deck cut-out, wire it, route the six servo leads up through the
   slot in front of the fence.
3. **Motor driver (Cytron MDD10A)** on the deck top, front-left: blue terminals facing forward, 4 × M3 × 10
   self-tapping screws into the printed bosses. Feed B+/B− and the 4 motor-pair wires up through the slot
   in front of the terminals; the 5 signal wires (GND, PWM2, DIR2, PWM1, DIR1) come up through the servo slot
   to the white plug at the back of the board. Details: WIRING.md → "Motor driver".
4. Put the battery in its cage, lead out of the +X window, screw on the two bars.
5. Deck onto the tub: 4 × M3 × 10 into the inserts.
6. Assemble the thenar-arms R3 follower (its own guide). Sit the arm base inside the deck fence
   and bolt it with 4 × M3 × 20 + washers through the base vent slots at (−14.2, ±31.7) and
   (55.5, ±27.8) mm (arm frame) — nyloc nuts under the deck. Plug servos J1..J6 into PCA9685 ch 0..5.

## 5. Firmware
1. `python firmware/tools/new_key.py "Your Team Name"` — creates the secret `team_config.h`.
2. Compile: `powershell -File firmware/tools/compile_firmware.ps1` (or Arduino IDE, board
   "ESP32 Dev Module", add `firmware/libraries` as a sketchbook library folder).
3. Flash `thenar_walker_bot` to the robot ESP32 and `thenar_walker_controller` to the leader ESP32.
4. Leader calibration (once): hold the leader at the thenar-arms HOME pose, send `ZERO` on the
   serial monitor; fix any reversed axis with `SIGN <axis> -1`; centre the joystick with `JOYZERO`.
5. Servo calibration (once, arm unloaded, horns off): measure each MG996R's zero and direction,
   put them in `firmware/thenar_walker_bot/calibration.h`, set `FOLLOWER_CALIBRATED = true`,
   reflash. Until then the robot drives but the arm stays limp (by design).

## 6. First power-up (wheels off the ground)
1. Robot in STOW, leader folded to STOW, controller ENABLE **off**. Kill switch ON.
2. Controller LED stops fast-blinking when it joins `<Team>_TF`; robot LED blinks (DISARMED/HOLD).
3. ENABLE on → robot LED solid = ACTIVE (arm engages only if the leader matches STOW within 6°).
4. Check each wheel direction with the joystick; swap a motor's two leads if one is backwards.
5. **Failsafe test:** while driving and holding a cube, switch the controller off → wheels stop
   at once, arm freezes, gripper keeps the cube, and nothing moves until the controller is back
   and sends new commands. Repeat at inspection.

## 7. Before every run
Battery ≥ 11.4 V (controller prints `batt` mV), kill switch reachable, team_config not the example,
robot folded to STOW and inside the 300 × 200 × 300 box.
