import json
import shutil
import subprocess
from pathlib import Path

import pytest

from thenar_pi import armcal
from thenar_pi.armcal import ArmCal

ROOT = Path(__file__).resolve().parents[2]


def test_known_record_is_complete_and_matches_the_table():
    c = armcal.known()
    assert c.valid() and c.complete()
    # allowed q per joint, as in the CLAUDE.md calibration table (rounded by hand there, so within 1 degree)
    want = [(-14, 111), (-142, 31), (-99, 16), (-20, 58), (-50, 134), (-18, 96)]
    for i, (lo, hi) in enumerate(want):
        a, b = c.limits_deg(i)
        assert abs(a - lo) <= 1 and abs(b - hi) <= 1, (i, a, b)


def test_pulse_formula_and_safe_ends():
    c = armcal.known()
    assert c.pulse(0, 0) == pytest.approx(2360.0)
    assert c.pulse(0, 10) == pytest.approx(2260.0)   # sign -1
    assert c.pulse_ok(0, 1300) and not c.pulse_ok(0, 1200)


def test_save_load_round_trip_keeps_the_crc(tmp_path):
    c = armcal.known()
    assert armcal.save(tmp_path, c)
    back = armcal.load(tmp_path)
    assert back is not None and back.crc == c.crc and back.complete()


def test_corrupt_or_tampered_record_is_refused(tmp_path):
    assert armcal.load(tmp_path) is None                       # missing
    armcal.save(tmp_path, armcal.known())
    p = armcal.path(tmp_path)
    d = json.loads(p.read_text())
    d["zero_us"][1] += 50                                      # edited by hand: crc no longer matches
    p.write_text(json.dumps(d))
    assert armcal.load(tmp_path) is None
    p.write_text("{not json")
    assert armcal.load(tmp_path) is None


def test_joint_checks():
    c = armcal.known()
    bad = c.copy()
    bad.zero_us[2] = 400                                       # zero below 500 and outside the ends
    assert not bad.joint_ok(2) and c.joint_ok(2)               # the copy changed, the original did not
    bad = c.copy()
    bad.us_per_deg[3] = 25
    assert not bad.joint_ok(3)


@pytest.mark.skipif(shutil.which("c++") is None, reason="no C++ compiler")
def test_crc_matches_the_esp32_struct(tmp_path):
    """The Python record packs and CRCs exactly like tw::ArmCal in firmware/libraries/ThenarLink/src/tw_armcal.h."""
    k = armcal.KNOWN
    fill = "\n".join(
        f"c.zero_us[{i}]={k['zero_us'][i]}f; c.us_per_deg[{i}]={k['us_per_deg'][i]}f; c.sign[{i}]={k['sign'][i]};"
        f" c.min_us[{i}]={k['min_us'][i]}f; c.max_us[{i}]={k['max_us'][i]}f;" for i in range(6))
    src = tmp_path / "crc.cpp"
    src.write_text(f"""#include <cstdio>
#include "tw_armcal.h"
int main() {{ tw::ArmCal c; tw::armcal_defaults(c); c.done_mask = {k['done_mask']};
{fill}
printf("%u %d\\n", (unsigned)tw::armcal_crc(c), (int)sizeof(tw::ArmCal)); }}
""")
    exe = tmp_path / "crc"
    inc = ROOT / "firmware" / "libraries" / "ThenarLink" / "src"
    subprocess.run(["c++", "-std=c++17", f"-I{inc}", str(src), "-o", str(exe)], check=True)
    crc, size = subprocess.run([str(exe)], capture_output=True, text=True, check=True).stdout.split()
    assert int(crc) == armcal.known().crc
    assert int(size) == armcal._LAYOUT.size + 4
