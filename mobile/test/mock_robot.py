"""Stand-in for the robot firmware (firmware/thenar_phone) so the app can be tested without the ESP32.

    python mobile/test/mock_robot.py [port]          (default 8793; the app's robot address = 127.0.0.1:8793)

Same HTTP API and the same rules as the firmware: 50 Hz loop, arm slew 90 deg/s toward the targets, targets clamped
to the calibrated ends, joints not calibrated can't be switched on, phone silent 500 ms -> drive input zero and arm
holds, STOP latches the wheels until the sticks are centred, tw::Robot drive mixing and ramp.
Extra test-only endpoint: GET /_state (wheel outputs, latch, ...). J6 is left "not calibrated" on purpose.
"""
import json
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

SPEED_SCALE = [0.35, 0.65, 1.0]
TURN_SCALE = [0.5, 0.75, 1.0]
SPIN_MIN, DEADBAND, ACCEL, ARM_DPS, PHONE_TIMEOUT = 0.35, 0.05, 2.5, 90.0, 0.5

LO = [-85.0, -60.0, -80.0, -70.0, -85.0, -10.0]
HI = [85.0, 40.0, 30.0, 60.0, 85.0, 60.0]
USABLE = 0b011111

lock = threading.Lock()
S = dict(goal=[0.0] * 6, cur=[0.0] * 6, on=0, speed=1, wheelT=[0.0, 0.0], wheel=[0.0, 0.0], phone=(0.0, 0.0),
         last_cmd=0.0, phone_ok=False, latch=False, requests=0)


def clamp(v, a, b):
    return min(b, max(a, v))


def set_drive(thr, trn, sp):
    thr, trn = clamp(thr, -1, 1), clamp(trn, -1, 1)
    if abs(thr) < DEADBAND:
        thr = 0
    if abs(trn) < DEADBAND:
        trn = 0
    if thr == 0 and trn != 0:
        mag = SPIN_MIN + (TURN_SCALE[sp] - SPIN_MIN) * abs(trn)
        l = -mag if trn > 0 else mag
        return [l, -l]
    l = thr * SPEED_SCALE[sp] - trn * TURN_SCALE[sp]
    r = thr * SPEED_SCALE[sp] + trn * TURN_SCALE[sp]
    m = max(1.0, abs(l), abs(r))
    return [l / m, r / m]


def freeze():
    S['goal'] = S['cur'][:]


def loop():
    last = time.monotonic()
    while True:
        time.sleep(0.02)
        now = time.monotonic()
        dt = min(0.1, now - last)
        last = now
        with lock:
            if S['phone_ok'] and now - S['last_cmd'] > PHONE_TIMEOUT:
                S['phone_ok'] = False
                S['phone'] = (0.0, 0.0)
                freeze()
            thr, trn = S['phone']
            centred = abs(thr) < DEADBAND and abs(trn) < DEADBAND
            if S['latch'] and centred:
                S['latch'] = False
            if S['latch']:
                thr = trn = 0.0
            S['wheelT'] = set_drive(thr, trn, S['speed'])
            for k in range(2):
                t, w = S['wheelT'][k], S['wheel'][k]
                if t * w < 0:
                    w = 0.0
                elif abs(t) <= abs(w):
                    w = t
                else:
                    w += clamp(t - w, -ACCEL * dt, ACCEL * dt)
                S['wheel'][k] = w
            step = ARM_DPS * dt
            for i in range(6):
                if S['on'] >> i & 1:
                    S['cur'][i] += clamp(S['goal'][i] - S['cur'][i], -step, step)


def reply(full=False):
    r = dict(bat=12.4, on=S['on'], sp=S['speed'], q=[round(v, 1) for v in S['cur']], g=[round(v, 1) for v in S['goal']],
             pad='', padmem=0)
    if full:
        r.update(ssid='ThenarWalker_TF', cal=1, pca=1, bt=1, usable=USABLE, lo=LO, hi=HI)
    return r


class H(BaseHTTPRequestHandler):
    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        a = {k: v[0] for k, v in urllib.parse.parse_qs(u.query).items()}
        with lock:
            S['requests'] += 1
            if u.path == '/cfg':
                body = reply(True)
            elif u.path == '/c':
                S['last_cmd'] = time.monotonic()
                S['phone_ok'] = True
                if 'stop' in a:
                    S['wheelT'] = [0.0, 0.0]   # last stick input kept: the latch waits for it to be centred
                    S['wheel'] = [0.0, 0.0]
                    S['latch'] = True
                    freeze()
                else:
                    try:
                        thr, trn = (int(x) for x in a.get('d', '').split(','))
                        S['phone'] = (thr / 100, trn / 100)
                    except ValueError:
                        S['phone'] = (0.0, 0.0)
                    if 's' in a:
                        S['speed'] = clamp(int(a['s']), 0, 2)
                    if 'q' in a:
                        q = [float(x) for x in a['q'].split(',')]
                        if len(q) == 6:
                            S['goal'] = [clamp(q[i], LO[i], HI[i]) for i in range(6)]
                    if 'm' in a:
                        want = int(a['m']) & USABLE
                        for i in range(6):
                            if want >> i & 1 and not S['on'] >> i & 1:
                                S['cur'][i] = S['goal'][i]   # switching on: jumps to the target, like a servo
                        S['on'] = want
                body = reply()
            elif u.path == '/forgetpad':
                body = reply()
            elif u.path == '/_state':
                body = dict(S)
            else:
                self.send_response(302)
                self.send_header('Location', '/')
                self.end_headers()
                return
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *a):
        pass


if __name__ == '__main__':
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8793
    threading.Thread(target=loop, daemon=True).start()
    print(f'mock robot on http://127.0.0.1:{port}', flush=True)
    ThreadingHTTPServer(('127.0.0.1', port), H).serve_forever()
