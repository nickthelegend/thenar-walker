"""The robot's rules, tick by tick: the same checks the firmware's behaviour was tested against on the ESP32."""
import pytest
from fakes import Clock, FakeBus, FakePins

from thenar_pi import armcal, config
from thenar_pi.hw import PCA9685
from thenar_pi.robot import Robot, drive_targets, ramp


@pytest.fixture
def rig(tmp_path):
    def make(calibrated=True, pca_ok=True):
        if calibrated:
            armcal.save(tmp_path, armcal.known())
        bus, pins, clock = FakeBus(fail=not pca_ok), FakePins(), Clock()
        r = Robot(PCA9685(bus), pins, state_dir=str(tmp_path), clock=clock)
        r.boot()
        return r, bus, pins, clock
    return make


def run(r, clock, seconds, msg=None, every=0.04):
    """Tick at 100 Hz for `seconds`, the phone resending `msg` every 40 ms (its heartbeat) when given."""
    t_end, next_msg = clock.t + seconds, clock.t
    while clock.t < t_end - 1e-9:
        clock.advance(config.TICK_S)
        if msg is not None and clock.t >= next_msg:
            r.handle(msg)
            next_msg += every
        r.tick()


# ------------------------------------------------------------------ drive mixing (tw_robot.h numbers)
def test_drive_mixing():
    assert drive_targets(1, 0, 1) == pytest.approx((0.65, 0.65))
    assert drive_targets(1, 0, 2) == pytest.approx((1.0, 1.0))
    assert drive_targets(0.03, 0.02, 1) == (0, 0)                     # deadband
    left, right = drive_targets(0, 1, 1)                              # spin left: floor + turn scale
    assert left == pytest.approx(-0.75) and right == pytest.approx(0.75)
    left, right = drive_targets(0, 0.1, 1)
    assert left == pytest.approx(-(0.35 + 0.4 * 0.1))                # gentle spin still breaks away
    l2, r2 = drive_targets(1, 1, 2)
    assert max(abs(l2), abs(r2)) == pytest.approx(1.0)                # normalised


def test_ramp_rules():
    assert ramp(1.0, 0.0, 0.025) == pytest.approx(0.025)              # speeding up: ramped
    assert ramp(0.0, 0.8, 0.025) == 0.0                               # stopping: immediate
    assert ramp(-0.5, 0.5, 0.025) == 0.0                              # reversing: through zero first


# ------------------------------------------------------------------ wheels, link loss, STOP latch
def test_throttle_ramps_and_right_side_is_mirrored(rig):
    r, bus, pins, clock = rig()
    r.connected()
    run(r, clock, 0.1, "D 100 0")
    assert pins.motors[0] == pytest.approx(0.25, abs=0.03)            # 2.5 / s for 0.1 s
    assert pins.motors[1] == pytest.approx(-0.25, abs=0.03)           # SIDE_SIGN -1
    run(r, clock, 0.5, "D 100 0")
    assert pins.motors[0] == pytest.approx(0.65)                      # speed 1 cap


def test_phone_silence_stops_wheels_and_holds_the_arm(rig):
    r, bus, pins, clock = rig()
    r.connected()
    r.handle("A 1")
    run(r, clock, 0.5, "D 100 0")
    r.handle("V 2 30")                                                 # elbow hold-to-move
    run(r, clock, 0.1)
    assert r.link_up
    run(r, clock, 0.35)                                                # nothing for > 300 ms
    assert not r.link_up and pins.motors == [0.0, 0.0]
    assert r.vel == [0.0] * 6 and r.goal == r.cur                      # arm stops where it is
    assert pins.oe and bus.channel(5)[3] != 0x10                       # PWM stays on: the gripper keeps its grip


def test_stop_latch_keeps_the_last_stick_and_releases_only_when_centred(rig):
    r, bus, pins, clock = rig()
    r.connected()
    run(r, clock, 0.5, "D 80 0")
    r.handle("X")
    run(r, clock, 0.5, "D 80 0")                                       # stick still pushed: stays stopped
    assert r.stop_latch and pins.motors == [0.0, 0.0]
    run(r, clock, 0.05, "D 0 0")                                       # centred: latch releases
    assert not r.stop_latch
    run(r, clock, 0.3, "D 80 0")
    assert pins.motors[0] > 0.3


# ------------------------------------------------------------------ arm
def test_uncalibrated_arm_cannot_power_up(rig):
    r, bus, pins, clock = rig(calibrated=False)
    r.handle("A 1")
    assert r.on_mask() == 0 and not pins.oe and r.usable_mask() == 0


def test_arm_on_slews_and_clamps(rig):
    r, bus, pins, clock = rig()
    r.connected()
    r.handle("A 1")
    assert r.on_mask() == 0x3F and pins.oe
    r.handle("Q 0 200")                                               # J1 pan: clamped to its safe end (111 deg)
    assert r.goal[0] == pytest.approx(r.hi[0])
    run(r, clock, 0.1, "D 0 0")
    assert r.cur[0] == pytest.approx(12.0, abs=1.3)                   # 120 deg/s slew
    run(r, clock, 1.5, "D 0 0")
    assert r.cur[0] == pytest.approx(r.hi[0])
    us = r.cal.pulse(0, r.cur[0])
    assert r.cal.pulse_ok(0, us) and bus.channel(0)[2:] == [r.pca.count(us) & 0xFF, r.pca.count(us) >> 8]


def test_mask_and_limp(rig):
    r, bus, pins, clock = rig()
    r.handle("M 5")                                                    # J1 + J3
    assert r.on_mask() == 5 and pins.oe
    r.handle("A 0")
    assert r.on_mask() == 0 and not pins.oe and bus.channel(0) == [0, 0, 0, 0x10]


def test_bad_messages_are_ignored(rig):
    r, bus, pins, clock = rig()
    for m in ["D", "D x y", "Q 9 10", "Q 1 nan", "V -1 5", "S", "Zzz", ""]:
        assert r.handle(m) == []
    assert r.thr == 0 and r.trn == 0 and r.speed == 1
    assert r.vel == [0.0] * 6 and r.goal == [r.cur[i] for i in range(6)]


def test_ping_and_config(rig):
    r, bus, pins, clock = rig()
    assert r.handle("P 42") == ["P 42"]
    cfg = r.handle("G")[0]
    assert cfg.startswith("C{") and '"usable":63' in cfg and '"cal":1' in cfg and '"pca":1' in cfg
    t = r.telemetry().split()
    assert t[0] == "T" and t[1] == "0.00" and len(t) == 11            # no battery ADC -> page shows "-- V"


# ------------------------------------------------------------------ calibrator protocol
def test_calibrator_protocol(rig, tmp_path):
    r, bus, pins, clock = rig(calibrated=False)
    assert r.calibrator("hello")[0].startswith("OK HELLO tw-armcal 1") and "pca=1" in r.calibrator("HELLO")[0]
    assert r.calibrator("P 2 1600") == ["OK P 2 1600"] and pins.oe and r.raw[2]
    assert r.calibrator("STATE")[0] == "OK STATE oe=1 - - 1600 - - -"
    assert r.calibrator("P 9 1500") == ["ERR P channel"]
    assert r.calibrator("REL 2") == ["OK REL 2"] and not pins.oe
    assert r.calibrator("CAL 1 400 8 -1 500 1900 1") == ["ERR CAL 1 bad_values"]
    for i in range(6):
        k = armcal.KNOWN
        line = f"CAL {i} {k['zero_us'][i]} {k['us_per_deg'][i]} {k['sign'][i]} {k['min_us'][i]} {k['max_us'][i]} 1"
        assert r.calibrator(line) == [f"OK CAL {i} done=1"]
    assert r.calibrator("CALSAVE") == ["OK CALSAVE mask=3F"] and r.cal_ok
    got = r.calibrator("CALGET")
    assert got[-1] == "OK CALGET mask=3F complete=1" and got[0].startswith("CAL 0 2360.0 10.00000 -1 1250.0 2500.0 1")
    assert armcal.load(tmp_path).complete()                           # it reached the disk
    assert r.calibrator("FOO")[0].startswith("ERR unknown")


def test_calibrator_without_the_board(rig):
    r, bus, pins, clock = rig(pca_ok=False)
    assert r.calibrator("P 0 1500") == ["ERR P no_pca9685"]
    r.handle("A 1")
    assert r.on_mask() == 0                                            # no board, no joints


def test_raw_pulses_slew_at_the_rate(rig):
    r, bus, pins, clock = rig()
    r.calibrator("RATE 300")
    r.calibrator("P 4 1500")
    r.calibrator("P 4 1800")                                           # target; first pulse already jumped to 1500
    run(r, clock, 0.5)
    assert r.raw_cur[4] == pytest.approx(1650, abs=5)                  # 300 us/s
    run(r, clock, 1.0)
    assert r.raw_cur[4] == pytest.approx(1800)


# ------------------------------------------------------------------ the brain (Hasami) and the phone
def drive_both(r, clock, seconds, phone=None, brain=None):
    t_end, nxt = clock.t + seconds, clock.t
    while clock.t < t_end - 1e-9:
        clock.advance(config.TICK_S)
        if clock.t >= nxt:
            if phone is not None:
                r.handle(phone)
            if brain is not None:
                r.handle(brain, source="brain")
            nxt += 0.04
        r.tick()


def test_without_a_brain_nothing_changes(rig):
    r, bus, pins, clock = rig()
    r.connected()
    drive_both(r, clock, 0.6, phone="D 100 0")
    assert pins.motors[0] == pytest.approx(0.65) and not r.brain_up


def test_brain_drives_while_the_phone_idles(rig):
    r, bus, pins, clock = rig()
    r.connected()
    drive_both(r, clock, 0.6, phone="D 0 0", brain="D 100 0")
    assert pins.motors[0] == pytest.approx(0.65)


def test_phone_input_takes_control_for_two_seconds(rig):
    r, bus, pins, clock = rig()
    r.connected()
    drive_both(r, clock, 0.6, phone="D 0 0", brain="D 100 0")
    drive_both(r, clock, 0.3, phone="D -100 0", brain="D 100 0")      # the operator reverses
    assert pins.motors[0] < 0
    assert r.handle("D 100 0", source="brain") == ["E manual"]
    drive_both(r, clock, 2.6, phone="D 0 0", brain="D 100 0")         # operator lets go: after 2 s the brain is back (+ramp)
    assert pins.motors[0] == pytest.approx(0.65)


def test_phone_stop_pauses_the_brain_until_a_human_resumes(rig):
    r, bus, pins, clock = rig()
    r.connected()
    drive_both(r, clock, 0.6, phone="D 0 0", brain="D 100 0")
    r.handle("X")
    drive_both(r, clock, 3.0, phone="D 0 0", brain="D 100 0")
    assert r.brain_paused and pins.motors == [0.0, 0.0]
    assert r.handle("Q 0 30", source="brain") == ["E paused"]
    assert r.handle("R", source="brain") == ["E resumed"]
    drive_both(r, clock, 0.6, phone="D 0 0", brain="D 100 0")
    assert pins.motors[0] == pytest.approx(0.65)


def test_brain_can_always_stop_and_its_silence_stops_its_wheels(rig):
    r, bus, pins, clock = rig()
    r.connected()
    r.handle("A 1", source="brain")
    drive_both(r, clock, 0.6, phone="D 0 0", brain="D 100 0")
    r.handle("V 1 40", source="brain")
    drive_both(r, clock, 0.4, phone="D 0 0")                           # brain silent > 300 ms, phone still there
    assert pins.motors == [0.0, 0.0] and not r.brain_up and r.vel == [0.0] * 6 and pins.oe   # arm holds
    drive_both(r, clock, 0.2, phone="D 0 0", brain="D 100 0")
    r.handle("X", source="brain")
    drive_both(r, clock, 0.2, phone="D 0 0", brain="D 100 0")
    assert r.stop_latch and pins.motors == [0.0, 0.0]
