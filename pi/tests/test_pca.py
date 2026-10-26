from fakes import FakeBus

from thenar_pi.hw import PCA9685


def test_init_sequence_matches_the_firmware():
    bus = FakeBus()
    pca = PCA9685(bus)
    assert pca.init() and pca.ok
    assert pca.prescale == 121
    assert [w[2:] for w in bus.writes] == [
        (0x00, 0x10),   # MODE1 sleep
        (0xFE, 121),    # prescale for 50 Hz
        (0x01, 0x04),   # MODE2 totem-pole
        (0x00, 0x20),   # wake + auto-increment
        (0xFC, 0),      # ALL_LED_OFF_L
        (0xFD, 0x10),   # ALL_LED_OFF_H full-off: every servo limp
    ]


def test_pulse_and_limp_bytes():
    bus = FakeBus()
    pca = PCA9685(bus)
    pca.init()
    pca.chan_pulse(3, 1500)
    n = round(1500 * 25e6 / (1e6 * 122))
    assert n == 307 and bus.channel(3) == [0, 0, n & 0xFF, n >> 8]
    pca.chan_off(3)
    assert bus.channel(3) == [0, 0, 0, 0x10]


def test_missing_board_reports_not_ok():
    pca = PCA9685(FakeBus(fail=True))
    assert not pca.init() and not pca.ok
    pca.chan_pulse(0, 1500)   # must not raise
