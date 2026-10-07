# Bill of materials — Thenar Walker (RoboReach 2026-27)

Quantities are for **one robot + one controller**. Printed masses come from the SolidWorks volumes
(PETG at 1.27 g/cm³ × 0.55 effective fill for 4 walls + 30 % gyroid, TPU at 1.21 g/cm³ × 0.6).
Robot total from the CAD: **2.37 kg** (sized with +10 % → 2.61 kg).
Purchased dimensions are nominal envelopes — check your actual parts against the CAD before printing.

---

## 1. Moving parts

### 1a. Drive train — 4 identical wheel stations (skid steer, 12 V)

| # | Part | Qty | Spec / why | CAD |
|---|---|---:|---|---|
| D1 | **25GA-370 (= JGA25-370) 12 V 60 RPM gear motor, 103:1, 4 mm D-shaft, no encoder** — e.g. [The Engineer Store](https://www.theengineerstore.in/products/25ga-370-12v-60rpm-dc-gear-motor) (in stock when checked, ₹462); also listed by [Robu](https://robu.in/product/25ga-370-12v-60rpm-dc-gear-motor/), [Kitsguru](https://kitsguru.com/products/25ga370-dc-12v-60rpm-reduction-motor-high-torque), [Robotics DNA](https://roboticsdna.in/product/25ga-370-12v-60rpm-dc-gear-motor/), [Zbotic](https://zbotic.in/product/25ga-370-12v-60rpm-dc-gear-motor/) | 4 (+1 spare) | Rated 2.7 kg·cm @ 46 rpm, stall 8.2 kg·cm / 1.3 A. 15° ramp needs 1.63 kg·cm per motor (1.6× margin) and 3.27 kg·cm when only 2 wheels drive. Simulated end-to-end with these numbers (docs/SIMULATION.md). **Avoid the 100 RPM version** (stall ~3.6 kg·cm). Check the shaft is D-shaped (Robu's 9.4 mm shaft length is OK). **Listings disagree on torque** (Robu: stall 4 kg·cm; Kitsguru: 8.2) and the sim needs **≥ 6 kg·cm stall** to cross the speed breakers (5 fails, 6 passes) — buy one, do the bottle test in ASSEMBLY.md §0, then buy the rest. | P-01 |
| D2 | **Front wheel, one piece** Ø140 | 2 | Print **PETG** (RR-07): hub + hard tread together (a rigid PETG tread can't be stretched over the bead lips). The hard front lets the skid steer turn. | RR-07 |
| D3 | **Rear wheel** = hub + tyre | 2 + 2 | Hub RR-05 in PETG; tyre RR-06 in **TPU 95A**, stretched over the hub between the bead lips (grip for the ramp and speed breakers). | RR-05 / RR-06 |
| D4 | M3 × 5 cup-point grub screw | 4 | Hub → shaft flat. Hex-key access through the hole in the tyre and rim | — |
| D5 | M3 × 6 socket cap screw | 8 | Motor gearbox face → tub side wall (2 per motor) | — |
| D6 | **Cytron MDD10A Rev2.0** dual motor driver (already bought) | 1 | 5–30 V, 10 A continuous / 30 A peak per channel, 3.3 V logic, PWM + DIR inputs, 84.5 × 62 mm. Channel 1 = both left motors in parallel, channel 2 = both right. On the deck top, 4 × M3 × 10 self-tapping screws into printed bosses | P-05 |
| D6b | 10 kΩ resistor | 2 | PWM1 / PWM2 pull-downs (no twitch while the ESP32 boots) | — |
| D7 | 22 AWG silicone wire, red/black | 2 m | Motor leads | — |

Result: Ø140 wheels, wheelbase 152 mm, track 168 mm, overall 293.7 × 196 mm.
~0.34 m/s loaded, 0.44 m/s free.

### 1b. Robot arm — the thenar-arms MG996R follower R3 (unchanged)

Full part list and print notes: thenar-arms `so101-mg996r/R3-PRINT.md`.

| # | Part | Qty | Spec | CAD |
|---|---|---:|---|---|
| A1 | MG996R positional servo (180°) | 6 | J1 pan, J2 lift, J3 elbow, J4 wrist flex, J5 wrist roll, J6 gripper | MG996R_Nominal_R3 |
| A2 | 25T metal disc horn Ø20, 4 × M3 on 14 mm PCD | 6 | Must match your servo spline | Metal_Horn_D20_PCD14_R3 |
| A3 | Horn-to-link screws M3 × 8 | 24 | | — |
| A4 | Servo tab screws M3 × 12 + nut + 2 washers | 24 / 24 / 48 | | — |
| A5 | Arm prints (R3): Base, Shoulder, Upper arm, Forearm, Wrist pitch/roll, Gripper body, Moving jaw | 7 | PETG. CAD mass 368 g | thenar-arms STLs |
| A6 | PCA9685 16-channel PWM driver | 1 | Servo PWM, I²C 0x40 | P-04 |
| A7 | 6 V DC-DC buck, ≥ 10 A (XL4016 / "300 W 20 A" class), adjustable | 1 | 6 × MG996R; TowerPro stall 1.4 A each → 8.4 A worst case | P-06 |
| A8 | Servo extension leads 15–30 cm | 6 | Through the deck slot to the PCA9685 | — |
| A9 | **TPU finger pads**, 4 mm (recommended) | 2 | The jaws stop 19.7 mm apart at the 0° gripper limit: pads let it hold a thin marker (Robonardo, 130 pts) and grip thermocol better | — |
| A10 | Arm base → deck: M3 × 20 SHCS + washer + nyloc nut | 4 | Through four of the base's vent slots, nut under the deck | — |

Gripper opening (from the CAD): 50 mm cube at ~28°, 60 mm cube / Ø60 cylinder at ~38°,
79 mm maximum inner gap at 60°.

---

## 2. Electronics and power

| # | Part | Qty | Spec |
|---|---|---:|---|
| E1 | ESP32-DevKitC (ESP32-WROOM-32E, 38-pin) | 1 robot + 1 controller | Robot: Wi-Fi AP + control. Controller: the leader's existing ESP32 |
| E2 | 3S LiPo 2200 mAh 30C, XT60 | 1 (+1 spare) | 106 × 34 × 26 mm max. Peak 10.6 A, ~28 min per pack at average load |
| E3 | KCD4 DPST rocker 30 A, red (kill switch) | 1 | Panel cut-out 22.2 × 30.2 mm; in series with battery + |
| E4 | Fuse holder + fuse, 10–15 A | 1 | Blade or 5 × 20 mm glass (the glass panel-mount holder you bought is fine — use a 10 A fuse) |
| E5 | XT60 male/female pair | 1 | |
| E6 | MP1584 mini buck, set to 5.0 V | 1 | ESP32 supply |
| E7 | Resistors 100 kΩ + 27 kΩ | 1 each | Battery voltage sense on GPIO34 |
| E8 | 16 AWG silicone wire | 1 m | Battery → fuse → switch → drivers/bucks |
| E9 | Dupont / JST-XH leads, heat-shrink, zip ties | 1 lot | |
| E10 | M2.5 × 6 self-tapping screws | 20 | Boards to the floor standoffs |
| E11 | LiPo balance charger + LiPo safe bag | 1 | |

### Controller additions (on the thenar-arms L1 leader)

| # | Part | Qty | Spec |
|---|---|---:|---|
| C1 | 2-axis analog thumb joystick module | 1 | 3.3 V supply; VRx → GPIO34, VRy → GPIO35 |
| C2 | SPST toggle switch (ENABLE) | 1 | GPIO32 to GND |
| C3 | Momentary push button (SPEED) | 1 | GPIO33 to GND |
| C4 | USB power bank + cable | 1 | Powers the leader ESP32 |

The leader itself (6 × AS5600, TCA9548A, 12 × 688ZZ bearings, magnets, prints) is the
thenar-arms L1 BOM — no changes.

---

## 3. Chassis prints (PETG unless noted)

| # | Part | Qty | Bed size | Mass | Notes |
|---|---|---:|---|---:|---|
| RR-01 | Chassis tub | 1 | 240 × 138 × 41 mm | 144 g | Print floor-down, 4 walls; the motor U-slots' round tops print as short arches (no support needed below 26 mm) |
| RR-02 | Deck plate | 1 | 250 × 138 × 11 mm | 119 g | Arm fence, MDD10A bosses + wire slot, ESP32 standoff holes; fits the P1S 256 mm bed |
| RR-03 | Battery cage | 1 | 128 × 44 × 27 mm | 22 g | |
| RR-04 | Battery retaining bar | 2 | 10 × 44 × 3 mm | 1 g | |
| RR-05 | Rear wheel hub | 2 | Ø131 × 28 mm | 68 g | PETG, spoke disc down |
| RR-06 | Rear tyre | 2 | Ø140 × 25 mm | 44 g | TPU 95A |
| RR-07 | Front wheel (one piece) | 2 | Ø140 × 28 mm | ~115 g | PETG, inner face down |

**Ready-to-print P1S plates: `cad/print/` (5 plates, see cad/print/PRINT.md).** Single STLs: `cad/stl/`. Filament for the rover plates: ≈ 0.81 kg PETG + ≈ 87 g TPU, ≈ 28.6 h on a P1S (estimate ±25 %).

### Chassis fasteners

| Item | Qty | Use |
|---|---:|---|
| M3 heat-set insert (Ø4.0 bore, ~5.7 mm long) | 4 | Tub corner columns |
| M3 × 10 SHCS | 4 | Deck → tub (counterbored) |
| M3 × 8 SHCS + nyloc | 2 | Battery cage → floor |
| M3 × 8 self-tapping (or M3 into 2.6 mm pilot) | 4 | Battery bars → cage bosses |
| M2.5 × 10 brass standoff (F-F) + 2 × M2.5 × 6 screws | 2 | ESP32 under the deck (zip-tie the board to them) |
| M3 × 10 self-tapping screw (or M3 × 10 machine screw into the 2.6 mm pilot) | 4 | Cytron MDD10A → deck-top bosses |

---

## 4. Not in the BOM on purpose

* No adhesives, magnets, hooks or suction on the gripper (rule) — the jaws are purely mechanical.
* No ready-made Lego. The JGA25 gearboxes are ready-made gear assemblies, which the rules allow.
* No camera — not needed for manual control and saves weight/power.
