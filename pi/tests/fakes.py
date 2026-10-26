"""Stand-in hardware for the tests: it records every write so a test can check exactly what reached the bus and pins.

These replace the I2C bus and GPIO only; everything above them (the robot's rules, the protocol, the server) is the
real code. Same idea as mobile/test/mock_robot.py, one level lower.
"""


class FakeBus:
    def __init__(self, fail: bool = False):
        self.writes: list[tuple] = []   # ("byte", addr, reg, value) | ("block", addr, reg, [bytes])
        self.fail = fail

    def write_byte_data(self, addr, reg, value):
        if self.fail:
            raise OSError(121, "Remote I/O error")
        self.writes.append(("byte", addr, reg, value))

    def write_i2c_block_data(self, addr, reg, data):
        if self.fail:
            raise OSError(121, "Remote I/O error")
        self.writes.append(("block", addr, reg, list(data)))

    def channel(self, ch):
        """The last 4 bytes written to channel ch (LEDn_ON_L .. LEDn_OFF_H), or None."""
        for w in reversed(self.writes):
            if w[0] == "block" and w[2] == 6 + 4 * ch:
                return w[3]
        return None


class FakePins:
    def __init__(self, battery=None):
        self.oe = False
        self.motors = [0.0, 0.0]
        self.motor_log: list[tuple[int, float]] = []
        self.led_on = False
        self.battery = battery

    def set_oe(self, enabled):
        self.oe = enabled

    def oe_enabled(self):
        return self.oe

    def motor(self, side, v):
        self.motors[side] = v
        self.motor_log.append((side, v))

    def led(self, on):
        self.led_on = on

    def battery_v(self):
        return self.battery

    def close(self):
        self.motors = [0.0, 0.0]
        self.oe = False


class Clock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt: float):
        self.t += dt
        return self.t
