"""End to end over real sockets: HTTP page, the phone WebSocket and the calibrator WebSocket, with the real Server and
Robot on top of the recording fakes (tests/fakes.py)."""
import asyncio
import json
import urllib.request

import pytest
import websockets
from fakes import FakeBus, FakePins

from thenar_pi import armcal
from thenar_pi.hw import PCA9685
from thenar_pi.robot import Robot
from thenar_pi.server import Server


@pytest.fixture
async def srv(tmp_path):
    armcal.save(tmp_path, armcal.known())
    pins = FakePins(battery=11.7)
    robot = Robot(PCA9685(FakeBus()), pins, state_dir=str(tmp_path), ssid="Test_TF")
    robot.boot()
    s = Server(robot, None, http_port=0, ws_port=0, cal_port=0, host="127.0.0.1")
    await s.start()
    yield s, robot, pins
    await s.stop()


def get(port, path):
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}", timeout=3) as r:
            return r.status, r.headers.get("Content-Type"), r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.headers.get("Content-Type"), e.read()


async def test_http_page_health_and_camera(srv):
    s, robot, pins = srv
    loop = asyncio.get_running_loop()
    status, ctype, body = await loop.run_in_executor(None, get, s.http_port, "/")
    assert status == 200 and "text/html" in ctype and b'new WebSocket("ws://"' in body and b":81/" in body
    status, _, body = await loop.run_in_executor(None, get, s.http_port, "/calibrator")
    assert status == 200 and b"Connect Pi (Wi-Fi)" in body
    status, _, body = await loop.run_in_executor(None, get, s.http_port, "/snap.jpg")
    assert status == 503 and b"Setup required: no camera" in body
    status, _, body = await loop.run_in_executor(None, get, s.http_port, "/health")
    h = json.loads(body)
    assert status == 200 and h["pca"] and h["cal"] and h["camera"] == "none"


async def test_phone_protocol_end_to_end(srv):
    s, robot, pins = srv
    async with websockets.connect(f"ws://127.0.0.1:{s.ws_port}/") as ws:
        cfg = json.loads((await asyncio.wait_for(ws.recv(), 2))[1:])
        assert cfg["ssid"] == "Test_TF" and cfg["usable"] == 63 and cfg["cal"] == 1
        await ws.send("P 7")
        got = [await asyncio.wait_for(ws.recv(), 2) for _ in range(3)]
        assert "P 7" in got
        await ws.send("A 1")
        for _ in range(10):                                   # drive for ~0.4 s with the 40 ms heartbeat
            await ws.send("D 100 0")
            await asyncio.sleep(0.04)
        assert pins.motors[0] > 0.5 and pins.oe
        tele = [m for m in [await asyncio.wait_for(ws.recv(), 2) for _ in range(6)] if m.startswith("T ")]
        assert tele and tele[-1].split()[1] in ("11.70", "0.00") and tele[-1].split()[2] == "63"
        await asyncio.sleep(0.45)                             # phone silent > 300 ms
        assert pins.motors == [0.0, 0.0] and not robot.link_up and pins.oe   # stopped, arm still holding


async def test_newest_phone_wins(srv):
    s, robot, pins = srv
    a = await websockets.connect(f"ws://127.0.0.1:{s.ws_port}/")
    await a.recv()
    b = await websockets.connect(f"ws://127.0.0.1:{s.ws_port}/")
    await b.recv()
    with pytest.raises(websockets.ConnectionClosed):
        for _ in range(20):
            await asyncio.wait_for(a.recv(), 2)
    await b.send("P 1")
    assert "P 1" in [await asyncio.wait_for(b.recv(), 2) for _ in range(3)]
    await b.close()


async def test_calibrator_websocket(srv):
    s, robot, pins = srv
    async with websockets.connect(f"ws://127.0.0.1:{s.cal_port}/") as ws:
        await ws.send("HELLO")
        assert (await asyncio.wait_for(ws.recv(), 2)).startswith("OK HELLO tw-armcal 1")
        await ws.send("CALGET")
        lines = [await asyncio.wait_for(ws.recv(), 2) for _ in range(7)]
        assert lines[-1] == "OK CALGET mask=3F complete=1"
        await ws.send("P 0 2000")
        assert await asyncio.wait_for(ws.recv(), 2) == "OK P 0 2000" and pins.oe
        await ws.send("OFF")
        assert await asyncio.wait_for(ws.recv(), 2) == "OK OFF" and not pins.oe
