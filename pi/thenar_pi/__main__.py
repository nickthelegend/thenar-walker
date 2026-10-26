"""Run the robot:  python3 -m thenar_pi            (on the Pi, as the thenar user or root; see pi/README.md)
                   python3 -m thenar_pi --dry-run  (any computer: the page and protocol with no robot attached)

Ports default to 80 / 81 / 82 like the ESP32 build (the phone page always connects to :81). --http-port, --ws-port and
--cal-port exist for tests; on the robot keep the defaults.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import socket

from . import config, hw
from .camera import Camera
from .robot import Robot
from .server import Server


def main() -> None:
    ap = argparse.ArgumentParser(prog="python -m thenar_pi")
    ap.add_argument("--dry-run", action="store_true", help="no I2C, GPIO or camera: serve the page and protocol only")
    ap.add_argument("--ssid", default=os.environ.get("THENAR_SSID", f"{socket.gethostname()}"),
                    help="name shown on the phone page (the robot's Wi-Fi SSID)")
    ap.add_argument("--http-port", type=int, default=config.HTTP_PORT)
    ap.add_argument("--ws-port", type=int, default=config.WS_PORT)
    ap.add_argument("--cal-port", type=int, default=config.CAL_PORT)
    ap.add_argument("--brain-port", type=int, default=config.BRAIN_PORT)
    ap.add_argument("--state-dir", default=config.STATE_DIR)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")

    if a.dry_run:
        bus, pins, camera, ssid = hw.DryBus(), hw.DryPins(), None, f"DRY RUN ({a.ssid}, no robot)"
    else:
        bus, pins, camera, ssid = hw.open_i2c(), hw.PiPins(), Camera(), a.ssid
    robot = Robot(hw.PCA9685(bus), pins, state_dir=a.state_dir, ssid=ssid, dry_run=a.dry_run)
    robot.boot()
    server = Server(robot, camera, a.http_port, a.ws_port, a.cal_port, brain_port=a.brain_port)

    async def run() -> None:
        await server.start()
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(sig, stop.set)
            except NotImplementedError:   # Windows: Ctrl+C raises KeyboardInterrupt instead
                pass
        try:
            await stop.wait()
        finally:
            await server.stop()   # wheels 0, every servo limp, OE high
            pins.close()
            if camera:
                camera.close()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
