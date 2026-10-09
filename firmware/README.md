# Firmware

| Sketch | Board | Role |
|---|---|---|
| `thenar_walker_bot/` | ESP32 on the robot | Wi-Fi AP `<Team>_TF`, 4 × 12 V motors (1 × Cytron MDD10A, PWM + DIR), 6 × MG996R (PCA9685), failsafe |
| `thenar_phone/` | ESP32-S3 on the robot | Phone + gamepad control: Wi-Fi AP `<Team>_TF`, web page at http://192.168.4.1 (joystick, calibrated arm sliders), gamepad paired to the phone or (Bluepad32 build) straight to the robot |
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

## Phone + gamepad control (`thenar_phone`)

Two ways to use a game controller, same button layout (shown on the page):

- **Gamepad → phone → robot.** Pair the controller with the phone, open http://192.168.4.1. Works with any controller
  the phone accepts. Chrome only exposes gamepads on secure pages: on the phone open `chrome://flags`, enable
  *Insecure origins treated as secure*, add `http://192.168.4.1`, relaunch.
- **Gamepad → robot.** Build with the Bluepad32 core. The ESP32-S3 radio is BLE only, so this needs a BLE controller
  (Xbox Series / Xbox One with BLE firmware, 8BitDo in BLE mode...). Bluetooth-Classic-only pads (most Switch-style
  clones) use the phone route. The first pad that connects is remembered; "Forget robot gamepad" on the page resets it.

```
arduino-cli core install esp32-bluepad32:esp32@4.1.0 --additional-urls https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32s3 --libraries firmware/libraries -u -p COM5 firmware/thenar_phone   # both routes
arduino-cli compile --fqbn esp32:esp32:esp32s3          --libraries firmware/libraries -u -p COM5 firmware/thenar_phone   # phone route only
```
The arm uses the calibration saved with the calibrator's **Save to robot**; joints not marked done stay off.
