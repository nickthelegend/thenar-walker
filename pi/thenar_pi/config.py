"""Pins and constants for the Raspberry Pi robot runtime.

Every number that firmware/thenar_phone/thenar_phone.ino and ThenarLink/tw_robot.h use is copied here unchanged, so
the phone page, the calibrator and the drive feel are identical to the ESP32 build. Pins are BCM numbers; any of
them can be overridden with an environment variable of the same name (THENAR_<NAME>).
"""
from __future__ import annotations

import os


def _env_int(name: str, default: int) -> int:
    v = os.environ.get(f"THENAR_{name}")
    return int(v) if v not in (None, "") else default


# ----------------------------------------------------------------- pins (BCM; planned, confirm on the real wiring)
I2C_BUS = _env_int("I2C_BUS", 1)          # GPIO2 SDA, GPIO3 SCL on the 40-pin header
OE_PIN = _env_int("OE_PIN", 17)           # PCA9685 OE, active low: HIGH = every servo output off
PWM_PIN = (_env_int("PWM1_PIN", 12), _env_int("PWM2_PIN", 13))   # MDD10A PWM1 (left), PWM2 (right): hardware PWM0/1
DIR_PIN = (_env_int("DIR1_PIN", 5), _env_int("DIR2_PIN", 6))     # MDD10A DIR1, DIR2
PWM_CHANNEL = (0, 1)                       # sysfs PWM channels on the pwm-2chan overlay (GPIO12 -> 0, GPIO13 -> 1)
LED_PIN = _env_int("LED_PIN", 27)
SIDE_SIGN = (1, -1)                        # right-side motors face the other way
PWM_HZ = 20000                             # MDD10A takes up to 20 kHz; above hearing
ADS1115_ADDR = 0x48                        # optional battery ADC; without it the battery reads "-- V"
BATT_DIVIDER = (100.0 + 27.0) / 27.0       # same 100k / 27k divider as the ESP32 build

# ----------------------------------------------------------------- PCA9685
PCA = 0x40
PCA_CLOCK_HZ = 25_000_000                  # same constant as the calibrator, so calibrated pulses match
SERVO_HZ = 50

# ----------------------------------------------------------------- timing and limits (thenar_phone.ino)
LINK_TIMEOUT_S = 0.300
TICK_S = 0.010                             # 100 Hz control loop
TELE_S = 0.100                             # status to the phone
ARM_DPS = 120.0                            # slew limit toward a slider target
VEL_MAX = 90.0                             # hold-button speed cap

# ----------------------------------------------------------------- drive (ThenarLink/tw_robot.h)
DRIVE_ACCEL_PER_S = 2.5                    # full scale in 0.4 s
DRIVE_DEADBAND = 0.05
SPEED_SCALE = (0.35, 0.65, 1.0)
TURN_SCALE = (0.5, 0.75, 1.0)
SPIN_MIN = 0.35                            # spin-in-place breakaway floor

# ----------------------------------------------------------------- network
HTTP_PORT = _env_int("HTTP_PORT", 80)      # the phone page
WS_PORT = _env_int("WS_PORT", 81)          # the phone's WebSocket (the page connects to :81)
CAL_PORT = _env_int("CAL_PORT", 82)        # the calibrator's line protocol over a WebSocket
BRAIN_PORT = _env_int("BRAIN_PORT", 83)    # the brain (Hasami): same messages as the phone, lower priority
MANUAL_HOLD_S = 2.0                        # real phone input takes control from the brain for this long
STATE_DIR = os.environ.get("THENAR_STATE_DIR", "/var/lib/thenar")
