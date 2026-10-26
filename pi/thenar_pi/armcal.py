"""Arm calibration record: the Pi version of ThenarLink/tw_armcal.h.

pulse_us = zero_us + q_deg * us_per_deg * sign. The record keeps the ESP32 struct layout and its CRC32, so the numbers
mean exactly the same thing on both builds. On the Pi it lives in <STATE_DIR>/armcal.json (written atomically);
the robot uses it only when magic, version, crc and all six joints are good. Changes are checked on a copy first, so
a bad number can never make the stored record unloadable.

    python -m thenar_pi.armcal show                 print the stored record
    python -m thenar_pi.armcal seed-known           store the Thenar Walker arm's calibrated numbers (CLAUDE.md table)
"""
from __future__ import annotations

import dataclasses
import json
import os
import struct
import sys
import tempfile
import zlib
from pathlib import Path

ARMCAL_MAGIC = 0x43415754   # "TWAC"
ARMCAL_VERSION = 1
_LAYOUT = struct.Struct("<IHBB6f6f6b2x6f6f")   # tw::ArmCal up to (not including) crc


def f32(v: float) -> float:
    """Round to float32, as the ESP32 struct stores it, so the CRC matches after a JSON round trip."""
    return struct.unpack("<f", struct.pack("<f", float(v)))[0]


@dataclasses.dataclass
class ArmCal:
    zero_us: list[float]
    us_per_deg: list[float]
    sign: list[int]
    min_us: list[float]
    max_us: list[float]
    done_mask: int = 0
    magic: int = ARMCAL_MAGIC
    version: int = ARMCAL_VERSION
    crc: int = 0

    @classmethod
    def defaults(cls) -> "ArmCal":
        c = cls([1500.0] * 6, [f32(5.555556)] * 6, [1] * 6, [1000.0] * 6, [2000.0] * 6)
        c.crc = c.compute_crc()
        return c

    def copy(self) -> "ArmCal":
        return dataclasses.replace(self, zero_us=list(self.zero_us), us_per_deg=list(self.us_per_deg),
                                   sign=list(self.sign), min_us=list(self.min_us), max_us=list(self.max_us))

    def packed(self) -> bytes:
        return _LAYOUT.pack(self.magic, self.version, self.done_mask & 0xFF, 0, *map(f32, self.zero_us),
                            *map(f32, self.us_per_deg), *self.sign, *map(f32, self.min_us), *map(f32, self.max_us))

    def compute_crc(self) -> int:
        return zlib.crc32(self.packed()) & 0xFFFFFFFF   # same polynomial and init/xor as tw::crc32

    def joint_ok(self, i: int) -> bool:
        """Physically sane: zero inside its limits, limits inside 500..2500, slope 3..20 us/deg, sign +-1."""
        return (self.min_us[i] >= 500 and self.max_us[i] <= 2500 and self.min_us[i] < self.zero_us[i] < self.max_us[i]
                and 3.0 <= self.us_per_deg[i] <= 20.0 and self.sign[i] in (1, -1))

    def valid(self) -> bool:
        if self.magic != ARMCAL_MAGIC or self.version != ARMCAL_VERSION or self.crc != self.compute_crc():
            return False
        return all(self.joint_ok(i) for i in range(6))

    def complete(self) -> bool:
        return self.valid() and (self.done_mask & 0x3F) == 0x3F

    def pulse(self, i: int, q_deg: float) -> float:
        return self.zero_us[i] + q_deg * self.us_per_deg[i] * self.sign[i]

    def pulse_ok(self, i: int, us: float) -> bool:
        """A commanded angle is allowed only if its pulse stays inside the recorded safe limits."""
        return self.min_us[i] <= us <= self.max_us[i]

    def limits_deg(self, i: int) -> tuple[float, float]:
        a = (self.min_us[i] - self.zero_us[i]) / (self.us_per_deg[i] * self.sign[i])
        b = (self.max_us[i] - self.zero_us[i]) / (self.us_per_deg[i] * self.sign[i])
        return min(a, b), max(a, b)

    def to_json(self) -> dict:
        d = dataclasses.asdict(self)
        for k in ("zero_us", "us_per_deg", "min_us", "max_us"):
            d[k] = [f32(v) for v in d[k]]
        return d

    @classmethod
    def from_json(cls, d: dict) -> "ArmCal":
        fields = {f.name for f in dataclasses.fields(cls)}
        return cls(**{k: v for k, v in d.items() if k in fields})


# The Thenar Walker arm as calibrated in the browser calibrator (thenar-walker CLAUDE.md, calibration.h).
KNOWN = {
    "zero_us": [2360.0, 755.7, 1669.2, 796.2, 1920.0, 1460.0],
    "us_per_deg": [10.000, 8.222, 11.778, 14.889, 10.556, 10.000],
    "sign": [-1, -1, 1, 1, -1, -1],
    "min_us": [1250.0, 500.0, 500.0, 500.0, 500.0, 500.0],
    "max_us": [2500.0, 1920.0, 1860.0, 1660.0, 2450.0, 1640.0],
    "done_mask": 0x3F,
}


def path(state_dir: str | os.PathLike) -> Path:
    return Path(state_dir) / "armcal.json"


def load(state_dir: str | os.PathLike) -> ArmCal | None:
    """The stored record, or None when it is missing or fails any check (then no joint may power up)."""
    p = path(state_dir)
    try:
        c = ArmCal.from_json(json.loads(p.read_text()))
    except (OSError, ValueError, TypeError):
        return None
    return c if c.valid() else None


def save(state_dir: str | os.PathLike, c: ArmCal) -> bool:
    c.magic, c.version = ARMCAL_MAGIC, ARMCAL_VERSION
    c.crc = c.compute_crc()
    p = path(state_dir)
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=p.parent, prefix=".armcal.")
        with os.fdopen(fd, "w") as f:
            json.dump(c.to_json(), f, indent=1)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, p)   # atomic: a power cut leaves the old record or the new one, never half of one
        return True
    except OSError:
        return False


def known() -> ArmCal:
    c = ArmCal(**{k: (list(v) if isinstance(v, list) else v) for k, v in KNOWN.items()})
    c.crc = c.compute_crc()
    return c


def main(argv: list[str]) -> int:
    from . import config
    cmd = argv[1] if len(argv) > 1 else "show"
    if cmd == "seed-known":
        c = known()
        if not all(c.joint_ok(i) for i in range(6)):
            print("ERR known numbers fail the joint checks", file=sys.stderr)
            return 1
        ok = save(config.STATE_DIR, c)
        print(f"{'OK' if ok else 'ERR'} wrote {path(config.STATE_DIR)} mask={c.done_mask:02X}")
        return 0 if ok else 1
    c = load(config.STATE_DIR)
    if c is None:
        print(f"no valid record at {path(config.STATE_DIR)}")
        return 1
    for i in range(6):
        print(f"CAL {i} {c.zero_us[i]:.1f} {c.us_per_deg[i]:.5f} {c.sign[i]} {c.min_us[i]:.1f} {c.max_us[i]:.1f} "
              f"{(c.done_mask >> i) & 1}")
    print(f"mask={c.done_mask:02X} complete={int(c.complete())}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
