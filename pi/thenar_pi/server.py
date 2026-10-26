"""Network side of the Pi runtime, same as the ESP32 firmware from the phone's point of view.

  :80   GET /            the phone page (static/phone.html, generated from thenar_phone.ino by tools/extract_page.py)
        GET /calibrator  the arm calibrator (docs/calibrator; on the Pi it talks over the :82 WebSocket)
        GET /snap.jpg    one camera frame for the brain (503 "Setup required: no camera" without one)
        GET /health      JSON status
  :81   WebSocket        the phone protocol (thenar_pi.robot); the newest connection wins, like the ESP32 build
  :82   WebSocket        the calibrator line protocol: one text message per line, replies as text messages
  :83   WebSocket        the brain (Hasami): the phone's messages at lower priority (see thenar_pi.robot); the newest
                         brain wins; replies "E manual" / "E paused" / "E resumed" when the phone has control

Control runs at 100 Hz; telemetry goes to the phone at 10 Hz; the battery is read once a second in a worker thread.
"""
from __future__ import annotations

import asyncio
import json
import logging
import socket
from pathlib import Path

import websockets

from . import config
from .camera import Camera
from .robot import Robot

log = logging.getLogger("thenar_pi.server")
STATIC = Path(__file__).parent / "static"


class Server:
    def __init__(self, robot: Robot, camera: Camera | None = None, http_port: int = config.HTTP_PORT,
                 ws_port: int = config.WS_PORT, cal_port: int = config.CAL_PORT, host: str = "0.0.0.0",
                 brain_port: int = config.BRAIN_PORT):
        self.robot, self.camera, self.host = robot, camera, host
        self.http_port, self.ws_port, self.cal_port, self.brain_port = http_port, ws_port, cal_port, brain_port
        self.phone = None   # the one phone WebSocket
        self.brain = None   # the one brain WebSocket
        self._servers = []
        self._tasks: list[asyncio.Task] = []

    # ------------------------------------------------------------- phone WebSocket
    async def _phone(self, ws) -> None:
        _nodelay(ws)
        old, self.phone = self.phone, ws
        if old is not None:
            await old.close()   # the newest connection wins
        await ws.send(self.robot.connected())
        try:
            async for msg in ws:
                if isinstance(msg, str):
                    for reply in self.robot.handle(msg):
                        await ws.send(reply)
                    if self.robot.brain_notes:   # the operator took over or pressed STOP: tell the brain now
                        notes, self.robot.brain_notes = self.robot.brain_notes, []
                        if self.brain is not None:
                            for n in notes:
                                try:
                                    await self.brain.send(n)
                                except websockets.ConnectionClosed:
                                    pass
        except websockets.ConnectionClosed:
            pass
        finally:
            if self.phone is ws:
                self.phone = None
                self.robot.disconnected()

    # ------------------------------------------------------------- brain WebSocket
    async def _brain(self, ws) -> None:
        _nodelay(ws)
        old, self.brain = self.brain, ws
        if old is not None:
            await old.close()   # the newest brain wins
        await ws.send(self.robot.config_msg())
        try:
            async for msg in ws:
                if isinstance(msg, str):
                    for reply in self.robot.handle(msg, source="brain"):
                        await ws.send(reply)
        except websockets.ConnectionClosed:
            pass
        finally:
            if self.brain is ws:
                self.brain = None
                self.robot.brain_disconnected()

    # ------------------------------------------------------------- calibrator WebSocket
    async def _calibrator(self, ws) -> None:
        _nodelay(ws)
        try:
            async for msg in ws:
                if isinstance(msg, str):
                    for line in msg.splitlines():
                        if line.strip():
                            for reply in self.robot.calibrator(line):
                                await ws.send(reply)
        except websockets.ConnectionClosed:
            pass

    # ------------------------------------------------------------- HTTP (GET only)
    async def _http(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            head = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), 5)
            path = head.split(b" ", 2)[1].decode(errors="replace").split("?")[0]
            if path in ("/", "/index.html"):
                await _reply(writer, 200, "text/html; charset=utf-8", (STATIC / "phone.html").read_bytes())
            elif path in ("/calibrator", "/calibrator/"):
                await _reply(writer, 200, "text/html; charset=utf-8", (STATIC / "calibrator.html").read_bytes())
            elif path == "/snap.jpg":
                jpg = await asyncio.get_running_loop().run_in_executor(None, self.camera.jpeg) if self.camera else None
                if jpg is None:
                    await _reply(writer, 503, "text/plain", b"Setup required: no camera\n")
                else:
                    await _reply(writer, 200, "image/jpeg", jpg, extra="Cache-Control: no-store\r\n")
            elif path == "/health":
                r = self.robot
                body = {"ok": True, "pca": r.pca_ok, "cal": r.cal_ok, "phone": self.phone is not None, "link": r.link_up,
                        "brain": self.brain is not None, "brain_paused": r.brain_paused,
                        "camera": self.camera.kind if self.camera else "none", "dry_run": r.dry_run,
                        "battery_v": r.battery, "on": r.on_mask(), "q": [round(c, 1) for c in r.cur]}
                await _reply(writer, 200, "application/json", json.dumps(body).encode())
            else:
                await _reply(writer, 302, "text/plain", b"", extra="Location: /\r\n")
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, asyncio.LimitOverrunError, IndexError, ConnectionError):
            pass
        finally:
            writer.close()

    # ------------------------------------------------------------- loops
    async def _control(self) -> None:
        loop = asyncio.get_running_loop()
        nxt = loop.time()
        while True:
            self.robot.tick()
            nxt += config.TICK_S
            await asyncio.sleep(max(0.0, nxt - loop.time()))
            if loop.time() - nxt > 0.5:   # fell far behind (laptop sleep, debugger): don't try to catch up
                nxt = loop.time()

    async def _telemetry(self) -> None:
        while True:
            await asyncio.sleep(config.TELE_S)
            t = self.robot.telemetry()
            for ws in (self.phone, self.brain):
                if ws is not None:
                    try:
                        await ws.send(t)
                    except websockets.ConnectionClosed:
                        pass

    async def _battery(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            self.robot.battery = await loop.run_in_executor(None, self.robot.pins.battery_v)
            await asyncio.sleep(1.0)

    async def start(self) -> None:
        self._servers = [
            await asyncio.start_server(self._http, self.host, self.http_port),
            await websockets.serve(self._phone, self.host, self.ws_port, max_size=4096, ping_interval=None),
            await websockets.serve(self._calibrator, self.host, self.cal_port, max_size=4096, ping_interval=None),
            await websockets.serve(self._brain, self.host, self.brain_port, max_size=4096, ping_interval=None),
        ]
        # port 0 (tests) binds a free port: keep the real ones
        self.http_port, self.ws_port, self.cal_port, self.brain_port = (s.sockets[0].getsockname()[1] for s in self._servers)
        self._tasks = [asyncio.create_task(c) for c in (self._control(), self._telemetry(), self._battery())]
        log.info("page http://<robot>:%d  phone ws :%d  calibrator ws :%d  brain ws :%d", self.http_port, self.ws_port,
                 self.cal_port, self.brain_port)

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        for s in self._servers:
            s.close()
        self.robot.shutdown()


def _nodelay(ws) -> None:
    """One frame = one TCP segment, sent at once (no Nagle), as on the ESP32."""
    sock = ws.transport.get_extra_info("socket") if getattr(ws, "transport", None) else None
    if sock is not None:
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass


async def _reply(writer: asyncio.StreamWriter, status: int, ctype: str, body: bytes, extra: str = "") -> None:
    reason = {200: "OK", 302: "Found", 503: "Service Unavailable"}[status]
    writer.write(f"HTTP/1.1 {status} {reason}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\n{extra}"
                 f"Connection: close\r\n\r\n".encode() + body)
    await writer.drain()
