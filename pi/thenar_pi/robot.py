"""The robot's rules: a line-for-line port of firmware/thenar_phone/thenar_phone.ino to Python.

Phone -> robot (one text message each):  D thr turn (-100..100, turn > 0 = left)   S speed(0..2)   X (STOP)
  A 1|0 (arm on/limp)   M mask   Q joint deg (target)   V joint deg_per_s (hold buttons, 0 = stop)   P n (ping)   G (config)
Robot -> phone:  C{json config}   T bat on speed latch q0..q5   P n (pong)
Calibrator (one line in, one reply line starting OK or ERR):  HELLO  RATE n  OFF  P ch us  REL ch  STATE
  CAL ch zero slope sign min max done  CALSAVE  CALGET

Safety, unchanged from the ESP32 build: nothing from the phone for 300 ms -> wheels stop, hold buttons stop, the arm
stays where it is with PWM on (the gripper keeps its grip). STOP latches the wheels until the stick is centred; the
latch keeps the last stick input (zeroing it would release the latch on the next tick). Every joint starts limp;
switching a joint on makes it go to its target at once. Angles are clamped to the calibrated safe ends; joints not
marked done in the calibrator can't be switched on.

The brain (Hasami) is new on the Pi: it sends the phone's messages on its own WebSocket (port 83) and the phone always
has priority. Real phone input (stick out of the deadband, STOP, arm or speed buttons) takes control for 2 s and the
brain's motion commands are refused with "E manual"; a phone STOP also pauses the brain ("E paused") until a human
at the brain's side sends "R". The brain can always send STOP. Each source has its own 300 ms link timeout; with no
brain connected the robot behaves exactly like the ESP32 build.
"""
from __future__ import annotations

import json
import logging
import math
import time
from typing import Callable

from . import armcal, config
from .armcal import ArmCal
from .hw import PCA9685, Pins

log = logging.getLogger("thenar_pi.robot")


def clampf(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def drive_targets(t: float, r: float, speed: int) -> tuple[float, float]:
    """Wheel targets (left, right) from throttle and turn, same mixing as setDrive() / tw::Robot (turn > 0 = left)."""
    t, r = clampf(t, -1, 1), clampf(r, -1, 1)
    if abs(t) < config.DRIVE_DEADBAND:
        t = 0.0
    if abs(r) < config.DRIVE_DEADBAND:
        r = 0.0
    if t == 0 and r != 0:   # spin in place: breakaway floor + turn scale
        mag = config.SPIN_MIN + (config.TURN_SCALE[speed] - config.SPIN_MIN) * abs(r)
        left = -mag if r > 0 else mag
        return left, -left
    left = t * config.SPEED_SCALE[speed] - r * config.TURN_SCALE[speed]
    right = t * config.SPEED_SCALE[speed] + r * config.TURN_SCALE[speed]
    m = max(1.0, abs(left), abs(right))
    return left / m, right / m


def ramp(target: float, current: float, max_step: float) -> float:
    """Reversing stops first; slowing or stopping is immediate; speeding up is ramped."""
    if target * current < 0:
        return 0.0
    if abs(target) <= abs(current):
        return target
    return current + clampf(target - current, -max_step, max_step)


def _scan(s: str, types: tuple) -> tuple | None:
    """sscanf for 'D 10 -20' style messages: exactly len(types) leading fields, else None."""
    parts = s.split()
    if len(parts) < len(types):
        return None
    try:
        return tuple(t(p) for t, p in zip(types, parts))
    except ValueError:
        return None


def _int(s: str) -> int:
    return int(float(s)) if "." in s else int(s)


class Robot:
    def __init__(self, pca: PCA9685, pins: Pins, state_dir: str = config.STATE_DIR, ssid: str = "ThenarWalker_TF",
                 clock: Callable[[], float] = time.monotonic, dry_run: bool = False):
        self.pca, self.pins, self.state_dir, self.ssid, self.clock, self.dry_run = pca, pins, state_dir, ssid, clock, dry_run
        self.cal: ArmCal = ArmCal.defaults()
        self.cal_ok = self.pca_ok = self.link_up = self.stop_latch = False
        self.speed = 1
        self.lo, self.hi = [0.0] * 6, [0.0] * 6
        self.goal, self.cur, self.vel = [0.0] * 6, [0.0] * 6, [0.0] * 6
        self.on, self.raw = [False] * 6, [False] * 6
        self.raw_cur, self.raw_tgt, self.raw_rate = [1500.0] * 6, [1500.0] * 6, 300.0
        self.thr = self.trn = 0.0          # the phone's stick
        self.bthr = self.btrn = 0.0        # the brain's stick
        self.wheel_t, self.wheel = [0.0, 0.0], [0.0, 0.0]
        self.last_msg = self.last_tick = 0.0
        self.brain_last = 0.0
        self.brain_up = False              # a brain is connected and talking
        self.brain_paused = False          # the phone pressed STOP: the brain waits for a human "R"
        self.manual_until = 0.0            # real phone input holds control until then
        self.battery: float | None = None

    # ------------------------------------------------------------- setup
    def boot(self) -> None:
        self.pins.set_oe(False)   # servo outputs off until a joint is switched on
        for k in range(2):
            self.pins.motor(k, 0.0)
        stored = armcal.load(self.state_dir)
        self.cal_ok = stored is not None
        self.cal = stored or ArmCal.defaults()
        self.apply_cal()
        self.pca_ok = self.pca.init()
        self.last_tick = self.clock()
        log.info("THENAR PI  %s  PCA9685 %s  arm calibration %s (done mask %02X)%s", self.ssid,
                 "ok" if self.pca_ok else "MISSING", "ok" if self.cal_ok else "MISSING",
                 self.cal.done_mask if self.cal_ok else 0, "  DRY RUN (no robot attached)" if self.dry_run else "")

    def apply_cal(self) -> None:
        """Angle range of every joint = its calibrated safe ends."""
        for i in range(6):
            self.lo[i], self.hi[i] = self.cal.limits_deg(i)
            self.goal[i] = self.cur[i] = clampf(0.0, self.lo[i], self.hi[i])
            self.vel[i] = 0.0

    # ------------------------------------------------------------- hardware writes
    def servo_write(self, ch: int, q: float) -> None:
        us = self.cal.pulse(ch, q)
        if not self.cal.pulse_ok(ch, us):   # never outside the safe ends
            self.pca.chan_off(ch)
            self.on[ch] = False
            return
        self.pca.chan_pulse(ch, us)

    def update_oe(self) -> None:
        self.pins.set_oe(any(self.on[i] or self.raw[i] for i in range(6)))

    def motor_write(self, side: int, v: float) -> None:
        self.pins.motor(side, clampf(v * config.SIDE_SIGN[side], -1, 1))

    # ------------------------------------------------------------- joints
    def joint_usable(self, i: int) -> bool:
        return self.cal_ok and bool((self.cal.done_mask >> i) & 1) and self.lo[i] < self.hi[i]

    def usable_mask(self) -> int:
        return sum(1 << i for i in range(6) if self.joint_usable(i))

    def on_mask(self) -> int:
        return sum(1 << i for i in range(6) if self.on[i])

    def set_joint(self, i: int, want: bool) -> None:
        want = want and self.pca_ok and self.joint_usable(i)
        if want and not self.on[i]:   # jumps to the target
            self.raw[i] = False
            self.vel[i] = 0.0
            self.cur[i] = self.goal[i]
            self.on[i] = True
            self.servo_write(i, self.cur[i])
        elif not want and self.on[i]:
            self.on[i] = False
            self.vel[i] = 0.0
            self.pca.chan_off(i)

    def set_mask(self, m: int) -> None:
        for i in range(6):
            self.set_joint(i, bool((m >> i) & 1))
        self.update_oe()

    def all_limp(self) -> None:
        for i in range(6):
            self.on[i] = self.raw[i] = False
            self.vel[i] = 0.0
            if self.pca_ok:
                self.pca.chan_off(i)
        self.update_oe()

    def freeze_arm(self) -> None:
        for i in range(6):
            self.goal[i] = self.cur[i]
            self.vel[i] = 0.0

    def stop_all(self) -> None:
        """STOP: wheels stop now and stay stopped until the stick is centred; the arm holds where it is."""
        self.wheel_t = [0.0, 0.0]
        self.wheel = [0.0, 0.0]
        self.stop_latch = True
        self.freeze_arm()

    # ------------------------------------------------------------- phone protocol
    def connected(self) -> str:
        self.last_msg = self.clock()
        self.link_up = True
        log.info("phone connected")
        return self.config_msg()

    def config_msg(self) -> str:
        r1 = lambda xs: [round(x, 1) for x in xs]
        cfg = {"ssid": self.ssid, "cal": int(self.cal_ok), "pca": int(self.pca_ok), "usable": self.usable_mask(),
               "on": self.on_mask(), "sp": self.speed, "lo": r1(self.lo), "hi": r1(self.hi), "g": r1(self.goal)}
        return "C" + json.dumps(cfg, separators=(",", ":"))

    def telemetry(self) -> str:
        bat = self.battery if self.battery is not None else 0.0   # the page shows "-- V" for anything <= 1
        q = " ".join(f"{c:.1f}" for c in self.cur)
        return f"T {bat:.2f} {self.on_mask()} {self.speed} {int(self.stop_latch)} {q}"

    def handle(self, c: str, source: str = "phone") -> list[str]:
        """One message from the phone (or, with source="brain", from the brain); returns the replies to send back."""
        now = self.clock()
        if source == "brain":
            return self._brain(c, now)
        self.last_msg = now
        if not self.link_up:
            self.link_up = True
            log.info("phone link back")
        if not c:
            return []
        if self._is_input(c):   # a person is driving: the brain waits
            self.manual_until = now + config.MANUAL_HOLD_S
            if c[0] == "X" and self.brain_up and not self.brain_paused:
                self.brain_paused = True
                log.info("phone STOP -> brain paused until a human resumes it")
        return self._apply(c, brain=False)

    @staticmethod
    def _is_input(c: str) -> bool:
        """Real phone input, as opposed to its idle heartbeat (D 0 0), pings and config requests."""
        if c[0] in "XAMQVS":
            return True
        if c[0] == "D":
            v = _scan(c[1:], (int, int))
            return bool(v) and (abs(v[0]) >= 5 or abs(v[1]) >= 5)
        return False

    def _brain(self, c: str, now: float) -> list[str]:
        """The brain speaks the phone's messages; it can always STOP, otherwise the phone has priority."""
        self.brain_last = now
        if not self.brain_up:
            self.brain_up = True
            log.info("brain connected")
        if not c:
            return []
        k = c[0]
        if k == "X":
            self.stop_all()
            return []
        if k == "R":   # a human at the brain's side resumes autonomy after a phone STOP
            if now < self.manual_until:
                return ["E manual"]
            self.brain_paused = False
            log.info("brain resumed")
            return ["E resumed"]
        if k in "DSAMQV":
            if self.brain_paused:
                return ["E paused"]
            if now < self.manual_until:
                return ["E manual"]
        return self._apply(c, brain=True)

    def brain_in_control(self, now: float) -> bool:
        return self.brain_up and not self.brain_paused and now >= self.manual_until

    def brain_disconnected(self) -> None:
        self.brain_up = False
        self.bthr = self.btrn = 0.0
        self.freeze_arm()
        log.info("brain disconnected -> its wheels stopped, arm holding")

    def _apply(self, c: str, brain: bool) -> list[str]:
        k, rest = c[0], c[1:]
        if k == "D":
            v = _scan(rest, (int, int))
            if v:
                if brain:
                    self.bthr, self.btrn = v[0] / 100.0, v[1] / 100.0
                else:
                    self.thr, self.trn = v[0] / 100.0, v[1] / 100.0
        elif k == "S":
            v = _scan(rest, (_int,))
            if v:
                self.speed = int(clampf(v[0], 0, 2))
        elif k == "X":
            self.stop_all()
        elif k == "A":
            v = _scan(rest, (_int,))
            if v:
                self.set_mask(self.usable_mask() if v[0] else 0)
        elif k == "M":
            v = _scan(rest, (_int,))
            if v:
                self.set_mask(v[0])
        elif k == "Q":
            v = _scan(rest, (_int, float))
            if v and 0 <= v[0] < 6 and math.isfinite(v[1]):
                self.goal[v[0]] = clampf(v[1], self.lo[v[0]], self.hi[v[0]])
                self.vel[v[0]] = 0.0
        elif k == "V":
            v = _scan(rest, (_int, float))
            if v and 0 <= v[0] < 6 and math.isfinite(v[1]):
                self.vel[v[0]] = clampf(v[1], -config.VEL_MAX, config.VEL_MAX)
        elif k == "P":
            return [c]   # ping: straight back so the phone can show the round trip
        elif k == "G":
            return [self.config_msg()]
        return []

    def disconnected(self) -> None:
        log.info("phone disconnected")

    # ------------------------------------------------------------- calibrator protocol
    def calibrator(self, line: str) -> list[str]:
        s = line.strip().upper()
        if s == "HELLO":
            return [f"OK HELLO tw-armcal 1 sda=2 scl=3 oe={config.OE_PIN} pca={int(self.pca_ok)} chip=rpi app=thenar_pi"]
        if s.startswith("RATE "):
            v = _scan(s[5:], (_int,))
            if v:
                self.raw_rate = clampf(v[0], 20, 2000)
                return [f"OK RATE {int(self.raw_rate)}"]
        if s == "OFF":
            self.all_limp()
            return ["OK OFF"]
        if s.startswith("P "):
            v = _scan(s[2:], (_int, _int))
            if v:
                a, b = v
                if not self.pca_ok:
                    return ["ERR P no_pca9685"]
                if not 0 <= a <= 5:
                    return ["ERR P channel"]
                us = clampf(b, 500, 2500)
                self.on[a] = False   # the phone lets go of this joint
                self.raw_tgt[a] = us
                if not self.raw[a]:   # first pulse: the servo jumps there
                    self.raw[a] = True
                    self.raw_cur[a] = us
                    self.pca.chan_pulse(a, us)
                self.update_oe()
                return [f"OK P {a} {int(us)}"]
        if s.startswith("REL "):
            v = _scan(s[4:], (_int,))
            if v:
                a = v[0]
                if not 0 <= a <= 5:
                    return ["ERR REL channel"]
                self.on[a] = self.raw[a] = False
                if self.pca_ok:
                    self.pca.chan_off(a)
                self.update_oe()
                return [f"OK REL {a}"]
        if s == "STATE":
            vals = " ".join(str(round(self.raw_cur[i])) if self.raw[i] else "-" for i in range(6))
            return [f"OK STATE oe={int(self.pins.oe_enabled())} {vals}"]
        if s.startswith("CAL "):
            v = _scan(s[4:], (_int, float, float, _int, float, float, _int))
            if v:
                a, z, sl, b, mn, mx, d = v
                if not 0 <= a <= 5:
                    return ["ERR CAL channel"]
                t = self.cal.copy()   # checked on a copy: invalid numbers are refused
                t.zero_us[a], t.us_per_deg[a], t.sign[a] = z, sl, 1 if b >= 0 else -1
                t.min_us[a], t.max_us[a] = mn, mx
                if not t.joint_ok(a):
                    return [f"ERR CAL {a} bad_values"]
                t.done_mask = t.done_mask | (1 << a) if d else t.done_mask & ~(1 << a)
                self.cal = t
                return [f"OK CAL {a} done={1 if d else 0}"]
        if s == "CALSAVE":
            ok = armcal.save(self.state_dir, self.cal)
            self.cal_ok = ok and self.cal.valid()
            self.all_limp()
            self.apply_cal()   # new numbers: every joint starts limp again
            return [f"{'OK CALSAVE' if ok else 'ERR CALSAVE disk'} mask={self.cal.done_mask:02X}"]
        if s == "CALGET":
            c = self.cal
            out = [f"CAL {i} {c.zero_us[i]:.1f} {c.us_per_deg[i]:.5f} {c.sign[i]} {c.min_us[i]:.1f} {c.max_us[i]:.1f} "
                   f"{(c.done_mask >> i) & 1}" for i in range(6)]
            return out + [f"OK CALGET mask={c.done_mask:02X} complete={int(c.complete())}"]
        return ["ERR unknown (HELLO RATE OFF P REL STATE CAL CALSAVE CALGET)"]

    # ------------------------------------------------------------- control loop
    def tick(self, now: float | None = None) -> bool:
        """Run one 100 Hz step if it is due. Returns True when a step ran."""
        now = self.clock() if now is None else now
        if now - self.last_tick < config.TICK_S:
            return False
        dt = min(now - self.last_tick, 0.1)
        self.last_tick = now

        brain = self.brain_in_control(now)
        if self.link_up and now - self.last_msg > config.LINK_TIMEOUT_S:   # phone silent: stop wheels, hold the arm
            self.link_up = False
            self.thr = self.trn = 0.0
            if not brain:
                self.freeze_arm()
            log.info("phone silent -> wheels stopped, arm holding")
        if self.brain_up and now - self.brain_last > config.LINK_TIMEOUT_S:   # brain silent: same rule for the brain
            self.brain_up = False
            self.bthr = self.btrn = 0.0
            if brain:
                self.freeze_arm()
            log.info("brain silent -> its wheels stopped, arm holding")
            brain = False

        thr, trn = (self.bthr, self.btrn) if brain else (self.thr, self.trn)   # whoever is in control drives
        centred = abs(thr) < config.DRIVE_DEADBAND and abs(trn) < config.DRIVE_DEADBAND
        if self.stop_latch and centred:
            self.stop_latch = False
        self.wheel_t = list(drive_targets(0 if self.stop_latch else thr, 0 if self.stop_latch else trn, self.speed))
        step = config.DRIVE_ACCEL_PER_S * dt
        for k in range(2):
            self.wheel[k] = ramp(self.wheel_t[k], self.wheel[k], step)
            self.motor_write(k, self.wheel[k])

        astep = config.ARM_DPS * dt
        for i in range(6):
            if not self.on[i]:
                continue
            if self.vel[i] != 0:
                self.goal[i] = clampf(self.goal[i] + self.vel[i] * dt, self.lo[i], self.hi[i])
            if self.cur[i] == self.goal[i]:
                continue
            self.cur[i] += clampf(self.goal[i] - self.cur[i], -astep, astep)
            self.servo_write(i, self.cur[i])
        rstep = self.raw_rate * dt
        for i in range(6):
            if not self.raw[i] or self.raw_cur[i] == self.raw_tgt[i]:
                continue
            self.raw_cur[i] += clampf(self.raw_tgt[i] - self.raw_cur[i], -rstep, rstep)
            self.pca.chan_pulse(i, self.raw_cur[i])
        self.update_oe()
        self.pins.led(self.link_up or int(now / 0.25) % 2 == 1)
        return True

    def shutdown(self) -> None:
        for k in range(2):
            self.pins.motor(k, 0.0)
        self.all_limp()
