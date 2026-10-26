"""Hardware on the Raspberry Pi: PCA9685 servo board (I2C), MDD10A motor driver (hardware PWM + DIR), OE, LED, battery.

Same register sequence as thenar_phone.ino: MODE1 sleep -> prescale -> MODE2 totem-pole -> wake + auto-increment,
then ALL_LED full-off and OE high, so every servo is limp at boot. Pulse write: LEDn_ON = 0, LEDn_OFF = count.
Limp one channel = the full-off bit (0x10 in LEDn_OFF_H).

The I2C bus and the pins are injected, so tests can record every write (tests/fakes.py) and a dry run can show the
page without a robot attached.
"""
from __future__ import annotations

import glob
import logging
import os
import time
from typing import Protocol

from . import config

log = logging.getLogger("thenar_pi.hw")


class I2CBus(Protocol):
    def write_byte_data(self, addr: int, reg: int, value: int) -> None: ...
    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None: ...


class PCA9685:
    def __init__(self, bus: I2CBus, addr: int = config.PCA, clock_hz: float = config.PCA_CLOCK_HZ):
        self.bus, self.addr, self.clock_hz = bus, addr, clock_hz
        self.prescale = round(clock_hz / (4096.0 * config.SERVO_HZ)) - 1   # 121 at 25 MHz / 50 Hz
        self.ok = False

    def _reg(self, r: int, v: int) -> bool:
        try:
            self.bus.write_byte_data(self.addr, r, v)
            return True
        except OSError as e:
            log.warning("PCA9685 write %02X failed: %s", r, e)
            return False

    def init(self) -> bool:
        ok = self._reg(0x00, 0x10) and self._reg(0xFE, self.prescale) and self._reg(0x01, 0x04) and self._reg(0x00, 0x20)
        time.sleep(0.002)   # oscillator restart after wake
        self.ok = ok and self._reg(0xFC, 0) and self._reg(0xFD, 0x10)   # ALL_LED_OFF: every channel limp
        return self.ok

    def count(self, us: float) -> int:
        return round(us * self.clock_hz / (1e6 * (self.prescale + 1)))

    def chan_off(self, ch: int) -> None:
        try:
            self.bus.write_i2c_block_data(self.addr, 6 + 4 * ch, [0, 0, 0, 0x10])
        except OSError as e:
            log.warning("PCA9685 channel %d off failed: %s", ch, e)

    def chan_pulse(self, ch: int, us: float) -> None:
        n = self.count(us)
        try:
            self.bus.write_i2c_block_data(self.addr, 6 + 4 * ch, [0, 0, n & 0xFF, (n >> 8) & 0x0F])
        except OSError as e:
            log.warning("PCA9685 channel %d pulse failed: %s", ch, e)


class Pins(Protocol):
    """Everything the robot drives besides the PCA9685."""
    def set_oe(self, enabled: bool) -> None: ...          # True = servo outputs on (OE pin LOW)
    def motor(self, side: int, v: float) -> None: ...     # side 0 left / 1 right, v -1..1 after SIDE_SIGN
    def led(self, on: bool) -> None: ...
    def battery_v(self) -> float | None: ...
    def oe_enabled(self) -> bool: ...
    def close(self) -> None: ...


class SysfsPwm:
    """One hardware PWM channel through /sys/class/pwm (dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4)."""

    def __init__(self, chip: str, channel: int, hz: int):
        self.base = f"{chip}/pwm{channel}"
        if not os.path.isdir(self.base):
            with open(f"{chip}/export", "w") as f:
                f.write(str(channel))
            for _ in range(50):   # udev sets the permissions a moment after export
                if os.access(f"{self.base}/period", os.W_OK):
                    break
                time.sleep(0.02)
        self.period_ns = round(1e9 / hz)
        self._w("enable", 0)
        self._w("duty_cycle", 0)
        self._w("period", self.period_ns)
        self._w("enable", 1)

    def _w(self, name: str, v: int) -> None:
        with open(f"{self.base}/{name}", "w") as f:
            f.write(str(v))

    def duty(self, frac: float) -> None:
        self._w("duty_cycle", round(max(0.0, min(1.0, frac)) * self.period_ns))

    def close(self) -> None:
        try:
            self._w("duty_cycle", 0)
            self._w("enable", 0)
        except OSError:
            pass


def find_pwm_chip() -> str | None:
    """The pwmchip with two channels that drives the header (the RP1 one on a Pi 5, the BCM one on a Pi 4)."""
    want = os.environ.get("THENAR_PWM_CHIP")
    if want:
        return want
    for chip in sorted(glob.glob("/sys/class/pwm/pwmchip*")):
        try:
            if int(open(f"{chip}/npwm").read()) >= 2:
                return chip
        except (OSError, ValueError):
            continue
    return None


class PiPins:
    """Real pins: gpiozero for DIR / OE / LED (works on Pi 4 and Pi 5), sysfs hardware PWM for the MDD10A."""

    def __init__(self):
        from gpiozero import DigitalOutputDevice   # imported here so tests and dry runs don't need it
        self._oe = DigitalOutputDevice(config.OE_PIN, active_high=False, initial_value=False)   # off = OE HIGH
        self._dir = [DigitalOutputDevice(p, initial_value=False) for p in config.DIR_PIN]
        self._led = DigitalOutputDevice(config.LED_PIN, initial_value=False)
        chip = find_pwm_chip()
        if chip:
            self._pwm = [SysfsPwm(chip, ch, config.PWM_HZ) for ch in config.PWM_CHANNEL]
            self._soft = None
            log.info("MDD10A on hardware PWM %s at %d Hz", chip, config.PWM_HZ)
        else:
            from gpiozero import PWMOutputDevice
            self._pwm = None
            self._soft = [PWMOutputDevice(p, frequency=1000, initial_value=0) for p in config.PWM_PIN]
            log.warning("no hardware PWM (add dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4 to config.txt): "
                        "software PWM at 1 kHz, audible and less smooth")
        self._ads = _ads1115()

    def set_oe(self, enabled: bool) -> None:
        self._oe.value = enabled

    def oe_enabled(self) -> bool:
        return bool(self._oe.value)

    def motor(self, side: int, v: float) -> None:
        self._dir[side].value = v < 0
        if self._pwm:
            self._pwm[side].duty(abs(v))
        else:
            self._soft[side].value = min(1.0, abs(v))

    def led(self, on: bool) -> None:
        self._led.value = on

    def battery_v(self) -> float | None:
        return self._ads() if self._ads else None

    def close(self) -> None:
        for k in range(2):
            self.motor(k, 0.0)
        self.set_oe(False)
        for p in self._pwm or []:
            p.close()


def _ads1115():
    """Battery on an ADS1115 (AIN0, single shot, +-4.096 V) when one answers at 0x48; otherwise None."""
    try:
        from smbus2 import SMBus
        bus = SMBus(config.I2C_BUS)
        bus.read_byte(config.ADS1115_ADDR)
    except Exception:
        return None

    def read() -> float | None:
        try:
            bus.write_i2c_block_data(config.ADS1115_ADDR, 0x01, [0xC3, 0x83])   # OS, AIN0, 4.096 V, single, 128 SPS
            time.sleep(0.009)
            hi, lo = bus.read_i2c_block_data(config.ADS1115_ADDR, 0x00, 2)
            raw = int.from_bytes(bytes([hi, lo]), "big", signed=True)
            return raw * 4.096 / 32768 * config.BATT_DIVIDER
        except OSError:
            return None
    return read


def open_i2c() -> I2CBus:
    from smbus2 import SMBus
    return SMBus(config.I2C_BUS)


class DryBus:
    """No robot attached (`python -m thenar_pi --dry-run`): accepts writes and logs nothing; the page says DRY RUN."""

    def write_byte_data(self, addr: int, reg: int, value: int) -> None:
        pass

    def write_i2c_block_data(self, addr: int, reg: int, data: list[int]) -> None:
        pass


class DryPins:
    def __init__(self):
        self._oe = False

    def set_oe(self, enabled: bool) -> None:
        self._oe = enabled

    def oe_enabled(self) -> bool:
        return self._oe

    def motor(self, side: int, v: float) -> None:
        pass

    def led(self, on: bool) -> None:
        pass

    def battery_v(self) -> float | None:
        return None

    def close(self) -> None:
        pass
