"""Generate the Pi runtime's static pages from their single sources, so the ESP32 and Pi builds serve the same page.

    python pi/tools/extract_page.py          (run from the repo root after editing either source)

  firmware/thenar_phone/thenar_phone.ino  PAGE raw string  ->  pi/thenar_pi/static/phone.html
  docs/calibrator/index.html                               ->  pi/thenar_pi/static/calibrator.html
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
STATIC = ROOT / "pi" / "thenar_pi" / "static"


def phone_page() -> str:
    ino = (ROOT / "firmware" / "thenar_phone" / "thenar_phone.ino").read_text(encoding="utf-8")
    start = ino.index('R"HTML(') + len('R"HTML(')
    end = ino.index(')HTML"', start)
    return ino[start:end]


def main() -> None:
    STATIC.mkdir(parents=True, exist_ok=True)
    (STATIC / "phone.html").write_text(phone_page(), encoding="utf-8")
    (STATIC / "calibrator.html").write_text((ROOT / "docs" / "calibrator" / "index.html").read_text(encoding="utf-8"),
                                            encoding="utf-8")
    print("wrote", STATIC / "phone.html", "and", STATIC / "calibrator.html")


if __name__ == "__main__":
    main()
