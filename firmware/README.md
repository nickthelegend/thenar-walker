# Firmware

| Sketch | Board | Role |
|---|---|---|
| `thenar_walker_bot/` | ESP32 on the robot | Wi-Fi AP `<Team>_TF`, 4 × 12 V motors (1 × Cytron MDD10A, PWM + DIR), 6 × MG996R (PCA9685), failsafe |
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
