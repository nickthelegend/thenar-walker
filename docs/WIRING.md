# Wiring — Thenar Walker

Pin numbers here are the ones in `firmware/thenar_walker_bot/thenar_walker_bot.ino` and
`firmware/thenar_walker_controller/thenar_walker_controller.ino`. If you change one, change both.

## Power tree (robot)

```
3S LiPo 2200 mAh (11.1 V nom, 12.6 V full)  ── XT60 ──┐
                                                      │
                            fuse 10–15 A (blade or 5×20 glass, inline, close to the XT60)
                                                      │
                          KCD4 30 A rocker = KILL SWITCH (both poles in parallel on +)
                                                      │  battery +  ("VBAT bus", 16 AWG)
        ┌─────────────────────────────────────┬───────┴─────────┬────────────────────┐
   MDD10A  B+  (B− to battery −)          6 V buck IN+        5 V buck IN+      100k ─┬─ 27k ─ GND
   M1A/M1B: FL + RL motors in parallel     (>= 10 A)           (MP1584)               │
   M2A/M2B: FR + RR motors in parallel      │ 6.0 V              │ 5.0 V            GPIO34 (battery sense)
                                     PCA9685 V+ terminal    ESP32 5V pin
                                     (servo rail, 6 x MG996R)   ESP32 3V3 -> PCA9685 VCC (logic)
All grounds common: battery −, MDD10A B− and input-header GND, both buck GND, PCA9685 GND, ESP32 GND.
```

* Every voltage on the robot is <= 12.6 V (rule: <= 24 V DC between any two points).
* The kill switch is in series with the battery positive *before* everything, so it shuts the
  whole robot down (motors, servos, ESP32). It is on the deck, front right, reachable from above.
* Set the 6 V buck to **6.0 V with no load before** connecting the PCA9685 V+ (MG996R: 4.8–6.6 V).
* Never power servos from the ESP32 or USB.
* Battery: inside the printed cage (RR-03) with two screwed bars (RR-04) — it cannot fall out if
  the robot rolls over (rule). Leads leave through the cage's +X end window.

## Robot ESP32 (ESP32-DevKitC, hung under the deck)

| ESP32 pin | Goes to | Notes |
|---|---|---|
| GPIO21 | PCA9685 SDA | I²C 400 kHz |
| GPIO22 | PCA9685 SCL | |
| GPIO13 | PCA9685 OE | HIGH = servo outputs off (boot state); firmware drives LOW once engaged |
| GPIO25 | MDD10A **PWM1** (header pin 4) | Left motors speed |
| GPIO26 | MDD10A **DIR1** (header pin 5) | Left motors direction |
| GPIO16 | MDD10A **PWM2** (header pin 2) | Right motors speed |
| GPIO17 | MDD10A **DIR2** (header pin 3) | Right motors direction |
| GND | MDD10A **GND** (header pin 1) | must be connected, or the inputs float |
| GPIO34 | battery divider mid-point | 100 kΩ (to VBAT) / 27 kΩ (to GND) |
| GPIO2 | on-board LED | solid = ACTIVE, blinking = HOLD / waiting |
| 5V / GND | 5 V buck out | |
| 3V3 | PCA9685 VCC | logic only |

### Motor driver: Cytron MDD10A Rev2.0 (one board, both sides)

* Mounted on top of the deck, front-left, on 4 printed bosses (4 × M3 × 10 self-tapping screws).
  The blue screw terminals face forward; all power and motor wires go down through the slot in front of them.
* Terminal block (left to right in the manual): **M1B, M1A, B+, B−, M2A, M2B**.
  * M1A / M1B: **both left motors in parallel** (front-left and rear-left: both red wires in M1A,
    both black wires in M1B).
  * M2A / M2B: **both right motors in parallel**, same way (red → M2A, black → M2B).
  * B+ from the kill switch output, B− to battery −.
* The 5-pin input header uses the supplied white 2510 plug: **1 GND, 2 PWM2, 3 DIR2, 4 PWM1, 5 DIR1**.
  Run the 5 jumper wires up through the arm's servo-cable slot to the header.
* Logic: 3.3 V is fine (high ≥ 3 V). PWM 20 kHz (the board's maximum). PWM low → both outputs low (brake);
  DIR low → current A→B, DIR high → B→A.
* Recommended: a **10 kΩ resistor from PWM1 to GND and from PWM2 to GND** (at the plug), so the
  wheels cannot twitch while the ESP32 is booting and its pins are still floating.
* Current: each 25GA-370 stalls at about 1.3 A, so 2 motors on one channel draw 2.6 A worst case
  (MDD10A: 10 A continuous per channel). No heat sink needed.
* **Test buttons**: M1A / M1B / M2A / M2B on the board spin the motors with no ESP32 connected. Use
  them for the first power-up and the bottle test (ASSEMBLY.md §0).
* The right-side motors face the other way, so the firmware inverts the right channel (`SIDE_SIGN`).
  If **both** wheels on one side spin backwards, swap that side's M_A / M_B wires. If only **one** wheel
  of a side spins backwards, that motor's two wires are swapped relative to its partner: fix them so
  both motors on that channel match.

### PCA9685 servo channels (same order as the thenar-arms follower)

| Channel | Joint | MG996R location |
|---|---|---|
| 0 | J1 shoulder pan | in the base |
| 1 | J2 shoulder lift | shoulder |
| 2 | J3 elbow flex | upper arm |
| 3 | J4 wrist flex | forearm |
| 4 | J5 wrist roll | wrist |
| 5 | J6 gripper | gripper body |

Servo leads run down through the deck slot in front of the arm fence.
`calibration.h` ships with `FOLLOWER_CALIBRATED = false`: the robot drives, but the arm PWM stays
off until you measure each servo's zero/direction unloaded and set it to `true`.

## Controller (the thenar-arms L1 encoder leader)

Keep the leader's existing ESP32 + TCA9548A (0x70) + 6 × AS5600 wiring from thenar-arms
(`so101-mg996r/ENCODER-LEADER.md`, SDA 21 / SCL 22). Add:

| ESP32 pin | Goes to |
|---|---|
| GPIO34 | joystick VRx (turn) |
| GPIO35 | joystick VRy (throttle) |
| 3V3 / GND | joystick VCC / GND (3.3 V, not 5 V — the ADC tops out at ~3.1 V) |
| GPIO32 | ENABLE toggle switch to GND (on = robot may move) |
| GPIO33 | SPEED push button to GND (cycles precision 35 % / normal 65 % / fast 100 % drive; turning 50 / 75 / 100 %) |

Power the controller from a USB power bank. Only one person operates: right hand on the leader
arm handle, left hand on the joystick (rule: one operator).

## Link and failsafe behaviour (what the judges will ask)

* Robot runs a WPA2 access point named **`<TeamName>_TF`** (spaces removed, first 20 characters),
  max 1 connected station.
* Every packet is HMAC-SHA256 signed with a 32-byte key only your two ESP32s know
  (`firmware/tools/new_key.py`); a random per-pairing session id and increasing sequence
  numbers reject replays. Packets from any other IP/port are ignored.
* No valid command for 250 ms, or ENABLE switched off, or a leader encoder fault → **HOLD**:
  wheels stop immediately, the arm stops where it is, the gripper keeps its PWM (keeps holding),
  and nothing moves again until a *new* valid command arrives.
* Power-up: the robot stays DISARMED (servo PWM off) until the leader arm is within 6° of the
  robot's STOW pose, so the arm cannot jump when it engages.
* Demonstrate at inspection: switch the controller off → wheels stop, arm freezes, gripper holds.
