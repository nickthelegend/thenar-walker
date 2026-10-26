# Thenar Walker — project brief for Claude

Techfest IIT Bombay **RoboReach 2026-27** rover (finals 15-18 Dec 2026). A 4-wheel skid-steer base carrying the team's
**thenar-arms** 6-DoF MG996R arm (github.com/nickthelegend/thenar-arms, SO-ARM100 derived). Owner: nickthelegend
(India, builds with a friend; local parts; Bambu P1S + Creality SparkX i7 printers). Reusable methods and pitfalls for
new projects: [docs/PLAYBOOK.md](docs/PLAYBOOK.md).

## Rules that drive the design
Start inside 300 × 200 × 300 mm (L × W × H) · ground clearance ≥ 50 mm · grasp at 450 mm reach · wireless Wi-Fi/BT
**with authentication**, SSID `<TeamName>_TF`, one operator, phones allowed, no internet · on link loss: wheels stop,
arm stops, gripper keeps holding, stays disabled until a new valid command · hardware kill switch · ≤ 24 V DC.

## The robot as built (Oct 2026)
| Part | What it is |
|---|---|
| Chassis | Printed PETG tub v2 (`cad/tools/tub_v2.py`, mesh CAD) 240 × 138, walls to z 100, THENAR.IO / THENARLABS engraved on the tub walls. Deck = 3 mm wood 250 × 138, top at z 103 |
| Wheels | Ø140 printed PETG hubs + flat TPU tyre strips, 6.3 mm round bore. Wheelbase 154 (AXLE_X 77) |
| Drive motors | 4 × local **370 plastic-gearbox** motors: gearbox Ø37.5 × 20, M8 × 13 collar + 13 mm nut through the tub wall, Ø6 shaft with cross-hole, 22 mm long. Passed the 750 ml bottle @ 7 cm lift test. Clearance 51.25 mm at the gearbox bottom, 57 mm at the tub floor (shaft = wheel radius = 70 mm) |
| Motor driver | **Cytron MDD10A** Rev2, sign-magnitude PWM + DIR, 20 kHz. Ch1 = both left motors in parallel, ch2 = both right |
| Arm | 6 × MG996R on a **PCA9685** (0x40, 50 Hz, OE active-low). Channels: 0 base pan, 1 shoulder lift, 2 elbow, 3 wrist flex, 4 wrist roll, 5 gripper (= J1..J6) |
| Brain | **Raspberry Pi** (moving from the ESP32-S3, Oct 26): `pi/` runs the same protocol and rules in Python and adds a camera for the Hasami brain. The **ESP32-S3** build (CP210x, COM5 on the owner's PC) still works and stays in `firmware/` |
| Kill switch | KCD4 rocker (`cad/tools/make_kill_switch.py`), cuts all power |

**Raspberry Pi pins (BCM, planned — confirm on the real wiring):** PCA9685 on I2C-1 (SDA GPIO2, SCL GPIO3), OE GPIO17 ·
MDD10A PWM1 GPIO12, PWM2 GPIO13 (hardware PWM, 20 kHz, `dtoverlay=pwm-2chan`), DIR1 GPIO5, DIR2 GPIO6 · LED GPIO27 ·
battery sense needs an ADS1115 (0x48), optional. Full table: `docs/WIRING_PI.md`. The Pi needs its own 5 V 5 A buck.

**ESP32-S3 pins:** PCA9685 SDA 8, SCL 9, OE 10 (confirmed) · MDD10A PWM1 4, DIR1 5, PWM2 6, DIR2 7 (**assumed — not yet
confirmed by the team; verify before trusting wheel directions**) · battery sense GPIO1 (100k/27k) · LED 2.
`docs/WIRING.md` still shows the old classic-ESP32 pins (21/22/13, 25/26/16/17) — it is out of date for the S3.

**Arm calibration** (saved in the robot's NVS `tw-armcal` and in `firmware/thenar_walker_bot/calibration.h`):
pulse_us = zero + q_deg · us_per_deg · sign, clamped to the safe ends.

| Joint | zero µs | µs/° | sign | safe ends µs | allowed q |
|---|---|---|---|---|---|
| J1 pan | 2360.0 | 10.000 | −1 | 1250–2500 | −14 … 111° |
| J2 shoulder | 755.7 | 8.222 | −1 | 500–1920 | −142 … 31° |
| J3 elbow | 1669.2 | 11.778 | +1 | 500–1860 | −99 … 16° |
| J4 wrist flex | 796.2 | 14.889 | +1 | 500–1660 | −20 … 58° |
| J5 wrist roll | 1920.0 | 10.556 | −1 | 500–2450 | −50 … 134° |
| J6 gripper | 1460.0 | 10.000 | −1 | 500–1640 | −18 … 96° |

Joint angle meaning (reference poses from MuJoCo): pan 0 = arm forward, + = clockwise from above · shoulder −13.9 =
upper arm vertical, + leans forward · elbow −73.8 = straight, + bends down · wrist −2.3 = in line, + down ·
roll 0 = jaws one above the other · gripper 0 = tips 20 mm apart, ~1.35 mm/°. Known issue: J2, J4, J6 (odd PCA
channels) jumped/oscillated on small steps — do the servo-swap test (good servo on a bad channel) before blaming code.

## Control: phone only (current)
On the Pi: `python -m thenar_pi` (folder `pi/`, see `pi/README.md`), a line-for-line port of the firmware below:
same page, same port-81 WebSocket, same rules; the calibrator talks to it over a WebSocket on port 82 (**Connect Pi
(Wi-Fi)**), and `GET /snap.jpg` serves a camera frame. `pi/tools/extract_page.py` regenerates its pages from the .ino
and `docs/calibrator`, so there is one source for each.

`firmware/thenar_phone/thenar_phone.ino` is the ESP32 robot firmware. Robot = Wi-Fi AP `ThenarWalker_TF` (WPA2, one client);
phone opens `http://192.168.4.1` (page is inside the firmware), then everything runs over one **WebSocket on port 81**:
- phone → robot: `D thr turn` (−100..100, turn > 0 = left) · `S 0..2` speed · `X` STOP · `A 1|0` arm on/limp ·
  `M mask` · `Q joint deg` target · `V joint dps` hold-to-move speed (0 = stop) · `P n` ping · `G` config
- robot → phone: `C{json}` config · `T bat on speed latch q0..q5` at 10 Hz · `P n` pong
- Latency: no polling, sent on touch (≤ 1 per frame) + 40 ms heartbeat, Nagle off, `esp_wifi_set_ps(WIFI_PS_NONE)`,
  100 Hz loop. Page shows the round trip in ms. Phone tip: mobile data OFF.
- Safety: 300 ms without a message → wheels stop, hold-moves stop, arm holds (PWM stays on). STOP latches the wheels
  until the stick is centred (the latch must keep the last stick input — never zero it, or it clears itself).
  Joints start limp; ARM ON jumps each joint to its target; targets clamped to the calibrated ends.
- The same firmware answers the calibrator's serial protocol (`HELLO P REL OFF RATE STATE CAL CALSAVE CALGET`), so
  `docs/calibrator` (Web Serial, Chrome/Edge, served from localhost) works without reflashing.

Build + flash (standard core, NOT Bluepad32):
```
F:\Projects\thenar-bin\tools\arduino-cli\arduino-cli.exe compile --fqbn esp32:esp32:esp32s3 --libraries firmware/libraries -u -p COM5 firmware/thenar_phone
```
Secrets: `firmware/libraries/ThenarLink/src/team_config.h` (Wi-Fi password + link key) is git-ignored — make it with
`python firmware/tools/new_key.py "Thenar Walker" <password>`. Never commit it or paste the password into tracked files.

## Repo map
| Path | Status | What |
|---|---|---|
| `pi/` | **active** | Raspberry Pi runtime `thenar_pi` (robot rules, server, camera, calibration record, AP setup, systemd unit), 26 tests |
| `firmware/thenar_phone/` | **active** | phone control firmware for the ESP32-S3 (above) |
| `docs/calibrator/` | **active** | Web Serial arm calibrator (two-reference method, safe ends, Save to robot) |
| `firmware/libraries/ThenarLink/` | active | shared headers: `tw_armcal.h` (NVS calibration record, CRC, validation), `tw_robot.h` (drive mixing constants, failsafe state machine), `tw_link.h`/`tw_sha256.h` (HMAC UDP link) |
| `firmware/arm_calibration/` | superseded | standalone calibration bridge (thenar_phone now speaks the same protocol) |
| `firmware/thenar_walker_bot/`, `thenar_walker_controller/`, `firmware/tests/` | dormant | original leader-arm + authenticated UDP design (classic ESP32 pins), host unit tests |
| `mobile/` | **stale** | Expo Android app; speaks the OLD HTTP `/c` protocol — won't work with the WebSocket firmware until ported |
| `docs/gamepad-tester/` | stale for driving | "Pad Check" controller tester + 3D rover sim; its robot link uses the old HTTP protocol |
| `cad/tools/` | active | `tub_v2.py` (mesh CAD + fit checks), `make_plates.py` (print plates), `engrave_logos.py`, `make_flat_tyres.py`, `make_kill_switch.py`, `design.py` (all dimensions); older SolidWorks COM scripts (`build_*.py`, `swlib.py`) built v1 |
| `cad/print/` | active | sliced plates for Bambu P1S (+ `creality/`), `PRINT.md` |
| `analysis/` | reference | arm FK, stow search (why the deck is low), drive sizing, gripper gap |
| `sim/` | reference | MuJoCo model + firmware-in-the-loop bridge, missions |
| `videos/thenar-assembly/` | done | HyperFrames assembly film (render in GitHub release v1) |
| `docs/` | mixed | BOM, ASSEMBLY, DESIGN, VERIFICATION, build book PDF; `WIRING_PI.md` is the Pi wiring; WIRING is out of date (classic ESP32 pins) |

## Owner's machine (Windows 11)
arduino-cli + cores `esp32:esp32` 3.3.10 (and unused `esp32-bluepad32` 4.1.0) · JDK 17 in
`F:\Projects\thenar-bin\tools\jdk17` · Android SDK in `%LOCALAPPDATA%\Android\Sdk` · Python 3.13 with trimesh,
manifold3d, mujoco, pyserial, cryptography · SolidWorks 2026 (the owner keeps other projects open, e.g. THENAR_Hexapod —
never close or kill it; close only our own docs with `cad/tools/close_mine.py`) · browser: Brave (use Chrome/Edge for
Web Serial and gamepads).

## Working with this owner
- Wants fast, end-to-end results: build it, test it, flash it, push it, then report in plain words. Commit + push
  to `main` of the public repo is expected; GitHub releases for big files (videos).
- Short casual messages; often a screenshot. Answer the actual question first.
- Never show fake/demo motion where real input is expected. When asked for a local server, don't publish an artifact.
- Ask before opening a COM port only if nothing in the request implies flashing/testing; a Web Serial tab holding
  COM5 blocks uploads ("port busy") — ask them to Disconnect it.
- India: suggest parts sold locally (Robu, The Engineer Store, Kitsguru), prices in ₹.

## Hard-won lessons (details in docs/PLAYBOOK.md)
- Git Bash on this machine mangles backslashes in heredocs (`\n` becomes a real newline): write source files with the
  editor tool, not shell heredocs. Windows "Restrictions" policy blocks `.bat` launchers: give `python ...` commands.
- Flashing keeps NVS (calibration survives). Browser localStorage is per origin **including the port** — the
  calibrator state was once recovered from Brave's LevelDB (`localhost:8790`) when the owner thought it was lost.
- ESP32-S3 has BLE only: Bluepad32 can't pair classic-BT pads (EvoFox, most clones; Xbox One S needs fw 5.x).
  That is why control is phone-only now.
- Text "seen from outside" a wall: check reading direction with a camera projection; logos shipped mirrored once.
- Bambu Studio CLI exit −64 = parts in the P1S front-left exclusion zone (keep x ≥ 20). Don't promise print times
  without slicing; more walls made a Creality print slower.
- Mesh CAD (trimesh + manifold) replaced SolidWorks for the tub because SolidWorks hung on text sketches; the
  SolidWorks model/assembly is still v1 and needs resyncing.

## Open items
1. Bring up the Pi build on the robot: Raspberry Pi OS Bookworm, I2C on, `pwm-2chan` overlay, `i2cdetect` shows 40,
   `thenar_pi.armcal seed-known`, wheels off the ground, then the phone. Confirm the Pi model and the camera.
2. Confirm the MDD10A pins and wheel directions on the real robot (wheels off the ground).
3. Servo-swap test for J2/J4/J6; replace bad MG996R clones.
4. Update `docs/WIRING.md` to the ESP32-S3 pins.
5. Port `mobile/` (or delete it) and Pad Check's robot link to the WebSocket protocol, if still wanted.
6. Resync the SolidWorks model to tub v2; re-check arm vs kill switch clearance.
