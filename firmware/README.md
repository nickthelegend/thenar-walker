# Firmware

| Sketch | Board | Role |
|---|---|---|
| `thenar_walker_bot/` | ESP32 on the robot | Wi-Fi AP `<Team>_TF`, 4 × 12 V motors (1 × Cytron MDD10A, PWM + DIR), 6 × MG996R (PCA9685), failsafe |
| `thenar_phone/` | ESP32-S3 on the robot | **Phone control**: Wi-Fi AP `<Team>_TF`, page at http://192.168.4.1, WebSocket for low latency, calibrator over USB |
| `arm_calibration/` | ESP32-S3 on the robot | Serial bridge for the browser arm calibrator (`docs/calibrator/`), saves the calibration to NVS |
| `thenar_walker_controller/` | ESP32 in the thenar-arms L1 leader | Reads the 6 AS5600 joints + joystick, sends signed commands at 50 Hz |
| `libraries/ThenarLink/` | both + host tests | SHA-256/HMAC, packet format, replay gate, robot state machine, drive mixing |

Everything safety-relevant is in `libraries/ThenarLink/src/tw_robot.h` and is unit-tested on the PC
(`tests/run_tests.ps1`, uses PlatformIO's bundled MinGW g++). The sketches are thin hardware glue.

States: **DISARMED** (boot, servo PWM off) → **ACTIVE** (leader within 6° of STOW + ENABLE) ⇄
**HOLD** (link silent 250 ms / ENABLE off / bad target / encoder fault: wheels 0, arm frozen,
servo PWM kept on so the gripper holds). HOLD → ACTIVE only on a new authenticated command, with
a 45°/s catch-up limit for 1.5 s.

Driving: throttle is scaled 35/65/100 % by the SPEED button; turning in place uses its own 50/75/100 %
scale with a 35 % breakaway floor (`TURN_SCALE`, `SPIN_MIN`) — tuned for PETG front / TPU rear tyres
(sim/turn_study.py).

Credentials: `python tools/new_key.py "Team Name"` writes `libraries/ThenarLink/src/team_config.h`
(git-ignored). Flash both boards after generating it. The example config compiles with a warning
so you notice.

Servo calibration values live in `thenar_walker_bot/calibration.h` (same meaning as thenar-arms).
Until `FOLLOWER_CALIBRATED = true` the arm stays unpowered while the wheels work.

## Phone control (`thenar_phone`) — the way to drive the robot

The robot is a Wi-Fi hotspot `<Team>_TF`. Join it with the phone (turn mobile data off), open **http://192.168.4.1**:
joystick for the wheels, Slow/Mid/Fast, STOP, and per joint a slider, hold-to-move − / + buttons and ON/OFF.

Low latency: after the page loads, everything goes over one WebSocket (port 81) that stays open. A touch is sent the
moment it happens (once per screen frame at most), Nagle is off on both ends, the ESP32's Wi-Fi power save is off,
and the control loop runs at 100 Hz. The top bar shows the measured round trip in ms.

Safety: the phone sends at least every 40 ms; 300 ms of silence (tab closed, screen off, Wi-Fi lost) stops the wheels
and the hold buttons, and the arm holds where it is. Calibration: `docs/calibrator` talks to this same firmware over USB.

```
arduino-cli compile --fqbn esp32:esp32:esp32s3 --libraries firmware/libraries -u -p COM5 firmware/thenar_phone
```
