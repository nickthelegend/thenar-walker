"""Thenar Walker end-to-end mission simulation (MuJoCo + firmware-in-the-loop).

  python sim/run_mission.py A      main course: pick 50 mm cube -> 15 deg ramp -> bridge -> ramp down
                                   -> 4 speed breakers -> place in the 80 x 80 touchdown square
  python sim/run_mission.py B      same, plus a 1.5 s radio drop on the bridge while forged and
                                   replayed packets are injected (rule: disconnect failsafe + auth)
  python sim/run_mission.py C      Downy: take the 60 mm cube off the 400 mm pole, place it in its square

Loop (10 ms): simulated operator (leader-arm joint angles + joystick, like the human would give)
-> real firmware code in sim/fw_bridge.exe (HMAC gate + tw::Robot state machine, unmodified headers)
-> wheel duty -> JGA25-370 torque-speed model -> MuJoCo wheels; servo targets -> MG996R position
servos (stall-limited). Results: sim/out/<scenario>.json, .mp4 and key-frame PNGs.
"""
import json
import math
import os
import subprocess
import sys

import imageio
import mujoco
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'out')
sys.path.insert(0, HERE)
import build_model as BM   # noqa: E402

A = json.load(open(os.path.join(HERE, 'assumptions.json')))
# what-if motor runs without touching the baseline results:  TW_MOTOR_RPM=100 TW_MOTOR_STALL_KGCM=4.9 python run_mission.py A
if os.environ.get('TW_MOTOR_RPM'):
    A['motor_free_rad_s'] = float(os.environ['TW_MOTOR_RPM']) * 2 * math.pi / 60
if os.environ.get('TW_MOTOR_STALL_KGCM'):
    A['motor_stall_Nm'] = float(os.environ['TW_MOTOR_STALL_KGCM']) * 0.0980665
JN = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
Q_LO = np.array([-85, -80, -80, -80, -85, 0.0])
Q_HI = np.array([85, 80, 80, 80, 85, 70.0])
STOW = [0, -80, 80, 50, 85, 0]
HOME = [0, -25, 35, 0, 0, 20]
CHASSIS_COL = ('col_tub', 'col_deck', 'col_motor')
OBSTACLES = ('ramp', 'breaker', 'wall', 'pole')


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Sim:
    def __init__(self, scenario, video=True):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(HERE, 'thenar_walker.xml'))
        self.d = mujoco.MjData(self.m)
        self.scr = mujoco.MjData(self.m)          # scratch data for IK
        self.sc = scenario
        self.t_ms = 0
        self.bridge = subprocess.Popen([os.path.join(HERE, 'fw_bridge.exe')], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                       text=True, bufsize=1)
        self.qadr = {n: self.m.jnt_qposadr[self.m.joint(n).id] for n in JN}
        self.vadr = {n: self.m.jnt_dofadr[self.m.joint(n).id] for n in JN}
        self.wheels = [(f'wheel_{x}{y}', 0 if y == 'L' else 1) for x in 'FR' for y in 'LR']
        self.wheel_ids = [(self.m.jnt_dofadr[self.m.joint(wn).id], self.m.actuator('m_' + wn[6:]).id, side) for wn, side in self.wheels]
        self.servo_act = [self.m.actuator('s_' + n).id for n in JN]
        self.chassis_dof = self.m.jnt_dofadr[self.m.joint('chassis').id]
        self.robot_geoms = set(i for i in range(self.m.ngeom) if self.m.geom(i).name.startswith(CHASSIS_COL))
        self.obst_geoms = set(i for i in range(self.m.ngeom) if self.m.geom(i).name.startswith(OBSTACLES))
        self.fw = {'state': 0, 'reason': 1, 'wl': 0.0, 'wr': 0.0, 'pwm': 0, 'q': list(STOW), 'acc': 0}
        self.log = []
        self.events = []
        self.frames = []
        self.video = video
        self.renderer = mujoco.Renderer(self.m, 540, 960) if video else None
        self.cam = mujoco.MjvCamera()
        self.geom_name = [self.m.geom(i).name for i in range(self.m.ngeom)]
        self.max_scrape = 0
        self.scrapes = []

    # ------------------------------------------------------------------ robot state as the operator sees it
    def place_robot(self, x, y, yaw_deg, q=STOW):
        a = self.m.jnt_qposadr[self.m.joint('chassis').id]
        self.d.qpos[a:a + 3] = [x, y, 0.0005]
        h = math.radians(yaw_deg) / 2
        self.d.qpos[a + 3:a + 7] = [math.cos(h), 0, 0, math.sin(h)]
        for n, v in zip(JN, q):
            self.d.qpos[self.qadr[n]] = math.radians(v)
            self.d.ctrl[self.m.actuator('s_' + n).id] = math.radians(v)
        mujoco.mj_forward(self.m, self.d)

    def pose(self):
        b = self.d.body('chassis')
        return b.xpos.copy(), b.xmat.reshape(3, 3).copy()

    def to_robot(self, p_world):
        p, R = self.pose()
        return R.T @ (np.asarray(p_world) - p)

    def tool_world(self):
        return self.d.site('tool').xpos.copy()

    def joint_deg(self):
        return np.array([math.degrees(self.d.qpos[self.qadr[n]]) for n in JN])

    def speed(self):
        a = self.chassis_dof
        return float(np.linalg.norm(self.d.qvel[a:a + 2]))

    # ------------------------------------------------------------------ inverse kinematics in the robot frame
    def tool_fk(self, q5_deg):
        s = self.scr
        s.qpos[:] = 0
        a = self.m.jnt_qposadr[self.m.joint('chassis').id]
        s.qpos[a + 3] = 1.0
        for n, v in zip(JN[:5], q5_deg):
            s.qpos[self.qadr[n]] = math.radians(v)
        mujoco.mj_kinematics(self.m, s)
        site = s.site('tool')
        return site.xpos.copy(), site.xmat.reshape(3, 3).copy()

    def ik(self, p_robot, approach, q0, horizontal_close=False, roll=None, off=(0, 0, 0)):
        """Joint angles putting the point tool + R @ off (held object centre, tool frame) at p_robot."""
        off = np.asarray(off, float)
        approach = np.asarray(approach, float) / np.linalg.norm(approach)
        lo, hi = Q_LO[:5].copy(), Q_HI[:5].copy()
        if roll is not None:
            lo[4], hi[4] = roll - 1e-6, roll + 1e-6
        x0 = np.clip(np.array(q0[:5], float), lo + 1e-3, hi - 1e-3)

        def res(q):
            p, R = self.tool_fk(q)
            r = list((p + R @ off - p_robot) * 1000) + list(60 * (R[:, 2] - approach))
            if horizontal_close:
                r.append(60 * R[2, 0])
            r += list(0.02 * (q - x0))
            return r
        best = None
        for start in (x0, np.clip(np.array(HOME[:5], float), lo + 1e-3, hi - 1e-3)):
            s = least_squares(res, start, bounds=(lo, hi), xtol=1e-9, ftol=1e-9)
            if best is None or s.cost < best.cost:
                best = s
        p, R = self.tool_fk(best.x)
        err = float(np.linalg.norm(p + R @ off - p_robot) * 1000)
        return list(best.x), err

    # ------------------------------------------------------------------ one 10 ms control tick
    def tick(self, op, mode=1):
        q = op['q']
        line = f"T {self.t_ms} {mode} {op['flags']} {int(op['thr'])} {int(op['turn'])} {op['speed']} " + ' '.join(f'{v:.3f}' for v in q) + '\n'
        self.bridge.stdin.write(line)
        self.bridge.stdin.flush()
        o = self.bridge.stdout.readline().split()
        self.fw = {'state': int(o[0]), 'reason': int(o[1]), 'wl': float(o[2]), 'wr': float(o[3]), 'pwm': int(o[4]),
                   'q': [float(x) for x in o[5:11]], 'acc': int(o[11])}
        for n, aid, qd in zip(JN, self.servo_act, self.fw['q']):
            self.d.ctrl[aid] = math.radians(qd) if self.fw['pwm'] else self.d.qpos[self.qadr[n]]
        Ts, kv, wf = A['motor_stall_Nm'], A['battery_V_over_12'], A['motor_free_rad_s']
        duty = (self.fw['wl'], self.fw['wr'])
        i_mot = i_bat = stall = 0.0
        for _ in range(10):                       # 10 x 1 ms physics steps
            for dof, aid, side in self.wheel_ids:
                tau = Ts * (duty[side] * kv - self.d.qvel[dof] / wf)
                tau = Ts if tau > Ts else (-Ts if tau < -Ts else tau)
                self.d.ctrl[aid] = tau
                im = 0.06 + 1.3 * abs(tau) / Ts            # JGA25-370 78:1: 0.06 A free, 1.3 A stall
                i_mot += im / 10; i_bat += im * abs(duty[side]) / 10; stall = max(stall, abs(tau) / Ts)
            mujoco.mj_step(self.m, self.d)
            if self.d.ncon:
                self.check_contacts()
        self.t_ms += 10
        self.elec = (i_mot, i_bat, stall)
        self.record(op, mode)

    def check_contacts(self):
        g1, g2 = self.d.contact.geom1[:self.d.ncon], self.d.contact.geom2[:self.d.ncon]
        for k in range(self.d.ncon):
            a, b = int(g1[k]), int(g2[k])
            if (a in self.robot_geoms and b in self.obst_geoms) or (b in self.robot_geoms and a in self.obst_geoms):
                self.scrapes.append((self.t_ms, self.geom_name[a], self.geom_name[b], float(-self.d.contact.dist[k])))

    def record(self, op, mode):
        if op.get('phase') != getattr(self, '_last_phase', None):
            self._last_phase = op.get('phase')
            p0, _ = self.pose()
            print(f"  t={self.t_ms / 1000:6.2f}s  phase {self._last_phase:12s} robot ({p0[0]:+.3f}, {p0[1]:+.3f})  fw state {self.fw['state']}", flush=True)
        p, R = self.pose()
        rec = {'t': self.t_ms / 1000, 'x': p[0], 'y': p[1], 'z': p[2], 'yaw': math.degrees(yaw_of(R)),
               'pitch': math.degrees(math.asin(-R[2, 0])), 'roll': math.degrees(math.atan2(R[2, 1], R[2, 2])),
               'v': self.speed(), 'state': self.fw['state'], 'wl': self.fw['wl'], 'wr': self.fw['wr'], 'mode': mode,
               'acc': self.fw['acc'], 'i_motors': self.elec[0], 'i_batt_motors': self.elec[1], 'stall_frac': self.elec[2], 'q_meas': self.joint_deg().tolist(), 'q_cmd': list(op['q']), 'phase': op.get('phase', '')}
        for c in ('cube50', 'cube60'):
            rec[c] = self.d.body(c).xpos.tolist()
        rec['tool'] = self.tool_world().tolist()
        xt = self.d.site('tool').xmat.reshape(3, 3)[:, 0]
        rec['grip50'] = (self.tool_world() - 0.0265 * xt).tolist()
        rec['grip60'] = (self.tool_world() - 0.0315 * xt).tolist()
        self.log.append(rec)
        if self.video and self.t_ms % 33 < 10:
            self.frame()

    def frame(self):
        p, R = self.pose()
        yaw = math.degrees(yaw_of(R))
        cam = self.cam
        cam.lookat[:] = p + R @ np.array([0.12, 0, 0.12])
        cam.distance = 1.05
        cam.azimuth = yaw + 205
        cam.elevation = -24
        self.renderer.update_scene(self.d, cam)
        img = self.renderer.render().copy()
        self.frames.append(img)

    def close(self):
        self.bridge.stdin.close()
        self.bridge.wait(timeout=5)


# ====================================================================== operator
class Operator:
    """What the human does with the leader arm + joystick. It looks at the scene (sim state), it does
    not drive the robot directly: everything goes through the firmware."""

    def __init__(self, sim):
        self.s = sim
        self.q = list(STOW)
        self.thr = self.turn = 0
        self.speed = 1
        self.flags = 7
        self.phase = 'engage'
        self.off = np.array([-0.0265, 0, 0])   # expected object centre in the tool frame (50 mm cube)

    def out(self):
        return {'q': self.q, 'flags': self.flags, 'thr': self.thr, 'turn': self.turn, 'speed': self.speed, 'phase': self.phase}

    def run(self, ticks, mode_fn=lambda: 1):
        for _ in range(int(ticks)):
            self.s.tick(self.out(), mode_fn())

    def wait_arm(self, timeout=6.0, tol=3.0):
        """Hold still until the firmware has slewed to the leader pose and the servos have settled."""
        prev = self.s.joint_deg()
        for _ in range(int(timeout * 100)):
            self.s.tick(self.out())
            fwq, meas = np.array(self.s.fw['q']), self.s.joint_deg()
            moving = np.max(np.abs(meas[:5] - prev[:5])) > 0.02     # deg per 10 ms = 2 deg/s
            prev = meas
            if np.max(np.abs(fwq - np.array(self.q))) < 0.2 and np.max(np.abs(meas[:5] - fwq[:5])) < tol and not moving:
                return True
        return False

    def arm_to(self, q, timeout=6.0, rate=90.0):
        """Move the leader smoothly (a person does not teleport the leader arm): `rate` deg/s per joint."""
        goal = np.array(q, float)
        cur = np.array(self.q, float)
        step = rate / 100.0
        while np.max(np.abs(goal - cur)) > 1e-6:
            cur = cur + np.clip(goal - cur, -step, step)
            self.q = [float(v) for v in cur]
            self.s.tick(self.out())
        return self.wait_arm(timeout)

    def arm_reach(self, p_robot, approach, grip, horizontal_close=False, roll=None, timeout=6.0, rate=90.0):
        q5, err = self.s.ik(np.asarray(p_robot), approach, self.q, horizontal_close, roll, self.off)
        if err > 4:
            self.s.events.append({'t': self.s.t_ms / 1000, 'event': 'ik_unreachable', 'target': list(map(float, p_robot)), 'err_mm': err})
        return self.arm_to(q5 + [grip], timeout, rate), err

    def drive_to_reach(self, target_world, reach_x, heading=None, lane_x=None, tol=0.004, timeout=20.0):
        """Joystick: creep until target_world sits reach_x ahead of the robot centre, steering onto it."""
        self.speed = 0
        for _ in range(int(timeout * 100)):
            pr = self.s.to_robot(target_world)
            ex = pr[0] - reach_x
            self.thr = float(np.clip(900 * ex, -100, 100))
            if abs(self.thr) < 12 and abs(ex) > tol:
                self.thr = 12 * np.sign(ex)
            self.turn = float(np.clip(400 * math.atan2(pr[1], max(pr[0], 0.05)), -60, 60)) if abs(ex) > 0.02 else 0
            if abs(ex) <= tol:
                self.thr = self.turn = 0
                break
            self.s.tick(self.out())
        self.thr = self.turn = 0
        for _ in range(60):                     # let it stop
            self.s.tick(self.out())
        return abs(self.s.to_robot(target_world)[0] - reach_x)

    def drive_lane(self, heading_deg, lane_x, until_y, speed=1, mode_fn=lambda t: 1, timeout=60.0):
        """Drive along +Y holding the lane centre line (what the operator steers by eye)."""
        self.speed = speed
        h_ref = math.radians(heading_deg)
        for _ in range(int(timeout * 100)):
            p, R = self.s.pose()
            if p[1] >= until_y:
                break
            e_lat = p[0] - lane_x
            e_h = wrap(h_ref - yaw_of(R))
            self.thr = 100
            self.turn = float(np.clip(150 * e_h + 700 * e_lat, -45, 45))
            self.s.tick(self.out(), mode_fn(self.s.t_ms))
        self.thr = self.turn = 0

    def turn_to(self, heading_deg, timeout=8.0):
        self.speed = 0
        h_ref = math.radians(heading_deg)
        for _ in range(int(timeout * 100)):
            _, R = self.s.pose()
            e = wrap(h_ref - yaw_of(R))
            if abs(e) < math.radians(1.0):
                break
            self.thr = 0
            self.turn = float(np.clip(250 * e, -100, 100))
            if abs(self.turn) < 35:
                self.turn = 35 * np.sign(e)
            self.s.tick(self.out())
        self.thr = self.turn = 0
        self.run(50)
        _, R = self.s.pose()
        self.s.events.append({'t': self.s.t_ms / 1000, 'event': 'turn_in_place_final_error_deg',
                              'value': round(math.degrees(wrap(h_ref - yaw_of(R))), 2)})


    def arc_turn_to(self, heading_deg, throttle, turn_max=45, speed=1, tol=3.0, timeout=12.0):
        """How a driver turns a 4WD skid-steer rover: keep the wheels rolling (forward or reverse) and
        steer, so the tyres roll instead of scrubbing sideways. Turn sign sets yaw direction either way."""
        self.speed = speed
        h_ref = math.radians(heading_deg)
        for _ in range(int(timeout * 100)):
            _, R = self.s.pose()
            e = wrap(h_ref - yaw_of(R))
            if abs(e) < math.radians(tol):
                break
            self.thr = throttle
            self.turn = float(np.clip(200 * e, -turn_max, turn_max))
            self.s.tick(self.out())
        self.thr = self.turn = 0
        self.run(40)
        _, R = self.s.pose()
        self.s.events.append({'t': self.s.t_ms / 1000, 'event': f'arc_turn ({"reverse" if throttle < 0 else "forward"}) heading error deg',
                              'value': round(math.degrees(wrap(h_ref - yaw_of(R))), 2)})


# ====================================================================== scenarios
DOWN = [0, 0, -1]
SPIN = '--spin' in sys.argv   # scenario C turning style (default: arcs, like a driver)
GRIP50 = 15      # leader gripper angle when squeezing the 50 mm cube (contact at ~29 deg)


def pick_from_floor(op, obj, reach=0.255):
    s = op.s
    obj_w = s.d.body(obj).xpos.copy()
    op.phase = 'unfold'
    op.arm_to(HOME)
    op.arm_reach([reach, 0, 0.12], DOWN, 40)
    op.phase = 'approach'
    s.events.append({'t': s.t_ms / 1000, 'event': 'approach_error_mm', 'value': 1000 * op.drive_to_reach(obj_w, reach)})
    op.phase = 'descend'
    pr = s.to_robot(s.d.body(obj).xpos)
    op.arm_reach([pr[0], pr[1], 0.07], DOWN, 45, rate=40)
    op.arm_reach([pr[0], pr[1], 0.026], DOWN, 45, rate=20)
    op.phase = 'grip'
    op.arm_to(op.q[:5] + [GRIP50], rate=60)
    op.run(50)
    op.phase = 'lift'
    op.arm_reach([pr[0], pr[1], 0.12], DOWN, GRIP50, rate=40)
    observe_in_hand(op, obj)
    z = float(s.d.body(obj).xpos[2])
    s.events.append({'t': s.t_ms / 1000, 'event': f'{obj}_lifted_to_mm', 'value': round(z * 1000, 1)})
    return z


def carry_pose(op):
    op.phase = 'carry'
    return op.arm_reach([0.19, 0, 0.20], DOWN, op.q[5])


def observe_in_hand(op, obj):
    """Operator looks at where the object actually sits between the jaws (tool frame)."""
    s = op.s
    R = s.d.site('tool').xmat.reshape(3, 3)
    op.off = R.T @ (s.d.body(obj).xpos - s.tool_world())
    s.events.append({'t': s.t_ms / 1000, 'event': f'{obj}_in_hand_offset_mm', 'value': [round(float(v) * 1000, 1) for v in op.off]})


def grasp_robot(op):
    """Current grasp point in the robot frame (what the operator sees between the jaws)."""
    s = op.s
    xt = s.d.site('tool').xmat.reshape(3, 3)[:, 0]
    return s.to_robot(s.tool_world() - op.off * xt), s.pose()[1].T @ xt


def place_on_floor(op, obj, target_w, reach=0.255, release=60):
    s = op.s
    op.phase = 'align'
    tw = np.array([target_w[0], target_w[1], 0.0])
    s.events.append({'t': s.t_ms / 1000, 'event': 'place_align_error_mm', 'value': 1000 * op.drive_to_reach(tw, reach)})
    half = 0.025 if obj == 'cube50' else 0.030
    op.phase = 'lower'
    observe_in_hand(op, obj)
    aim = s.to_robot(tw)[:2].copy()
    for z, rate in ((half + 0.04, 40), (half + 0.006, 20)):
        op.arm_reach([aim[0], aim[1], z], DOWN, op.q[5], rate=rate)
        for _ in range(4):                   # operator nudges until the object is over the square
            err = s.to_robot(tw)[:2] - s.to_robot(s.d.body(obj).xpos)[:2]
            if np.linalg.norm(err) < 0.0015:
                break
            aim += err
            op.arm_reach([aim[0], aim[1], z], DOWN, op.q[5], rate=15)
    op.arm_reach([aim[0], aim[1], half + 0.0015], DOWN, op.q[5], rate=10)
    op.phase = 'release'
    op.arm_to(op.q[:5] + [release], rate=60)
    op.run(30)
    # centre the open jaws around the object (fixed-finger gripper), then lift straight up
    R = s.d.site('tool').xmat.reshape(3, 3)
    rel = R.T @ (s.d.body(obj).xpos - s.tool_world())
    gap_centre = -0.0316 if release >= 60 else -0.028
    shift_w = R[:, 0] * (rel[0] - gap_centre)
    tool_r = s.to_robot(s.tool_world() + shift_w)
    op.off = np.zeros(3)
    op.arm_reach(tool_r, DOWN, release, rate=10)
    tool_r = s.to_robot(s.tool_world())
    op.arm_reach([tool_r[0], tool_r[1], tool_r[2] + 0.07], DOWN, release, rate=20)
    op.phase = 'retract'
    op.arm_to(HOME)


def scenario_AB(sim, faults):
    op = Operator(sim)
    sim.place_robot(-1.2, -1.25, 90)
    op.run(40)                                     # engage at STOW
    ok_engage = sim.fw['state'] == 1
    pick_from_floor(op, 'cube50')
    carry_pose(op)
    op.phase = 'course'
    fault = {'start': None}

    def mode_fn(t_ms):
        if not faults:
            return 1 if (t_ms // 10) % 2 == 0 else 0
        p, _ = sim.pose()
        if fault['start'] is None and -0.40 <= p[1] <= -0.30:
            fault['start'] = t_ms
            sim.events.append({'t': t_ms / 1000, 'event': 'RADIO DROP begins (robot on the bridge)'})
        if fault['start'] is not None and t_ms - fault['start'] < 1500:
            k = (t_ms - fault['start']) // 10
            if k % 2:
                return 0
            return 2 if k < 75 else 3                # attacker: forged packets, then replays
        if fault['start'] is not None and t_ms - fault['start'] == 1500:
            sim.events.append({'t': t_ms / 1000, 'event': 'radio restored'})
        return 1 if (t_ms // 10) % 2 == 0 else 0
    lane_x = -1.2
    target_y = BM.TOUCHDOWN[1] - 0.255 - 0.06
    op.drive_lane(90, lane_x, target_y, speed=1, mode_fn=mode_fn, timeout=60)
    place_on_floor(op, 'cube50', BM.TOUCHDOWN)
    op.phase = 'done'
    op.run(100)
    return op, ok_engage, fault


def scenario_C(sim):
    op = Operator(sim)
    op.off = np.array([-0.0315, 0, 0])
    px, py = BM.POLE
    sim.place_robot(px - 0.62, py, 0)
    op.run(40)
    op.phase = 'unfold'
    op.arm_to(HOME)
    cube = sim.d.body('cube60').xpos.copy()
    op.arm_reach([0.17, 0, cube[2]], [1, 0, 0], 50, horizontal_close=True)
    op.phase = 'approach'
    sim.events.append({'t': sim.t_ms / 1000, 'event': 'approach_error_mm', 'value': 1000 * op.drive_to_reach(cube, 0.25)})
    pr = sim.to_robot(sim.d.body('cube60').xpos)
    op.phase = 'reach_450'
    op.arm_reach([pr[0] - 0.035, pr[1], pr[2]], [1, 0, 0], 50, horizontal_close=True)
    op.arm_reach([pr[0], pr[1], pr[2]], [1, 0, 0], 50, horizontal_close=True)
    op.phase = 'grip'
    op.q[5] = 30
    op.run(80)
    op.phase = 'lift_off_pole'
    op.arm_reach([pr[0], pr[1], pr[2] + 0.02], [1, 0, 0], 30, horizontal_close=True)
    sim.events.append({'t': sim.t_ms / 1000, 'event': 'cube60_lifted_off_pole_mm', 'value': round(1000 * (sim.d.body('cube60').xpos[2] - 0.43), 1)})
    op.phase = 'back_off'
    op.thr = -60; op.speed = 0
    op.run(120)
    op.thr = 0
    op.run(40)
    op.phase = 'carry'
    op.arm_reach([0.20, 0, 0.25], [1, 0, -0.6], 30)
    tx, ty = BM.DOWNY_TARGET
    p, _ = sim.pose()
    op.phase = 'turn'
    h_goal = math.degrees(math.atan2(ty - p[1], tx - p[0]))
    if SPIN:                                   # spin on the spot (scrubs all four tyres)
        op.turn_to(h_goal)
    else:                                      # three-point style: reverse arc, then forward arc
        arc_speed = int(os.environ.get('TW_ARC_SPEED', '1'))   # 0 = precision mode (what a driver uses with faster motors)
        op.arc_turn_to(h_goal / 2, throttle=-70, turn_max=45, speed=arc_speed)
        op.arc_turn_to(h_goal, throttle=55, turn_max=45, speed=arc_speed)
        p, _ = sim.pose()                      # the arcs moved the robot: re-aim at the square
        op.turn_to(math.degrees(math.atan2(ty - p[1], tx - p[0])))
    place_on_floor(op, 'cube60', BM.DOWNY_TARGET, release=55)
    op.phase = 'done'
    op.run(100)
    return op


# ====================================================================== evaluation
def evaluate(sim, scenario, extra):
    L = sim.log
    res = {'scenario': scenario, 'sim_time_s': sim.t_ms / 1000, 'events': sim.events, 'checks': []}

    def chk(name, ok, detail):
        res['checks'].append({'check': name, 'pass': bool(ok), 'detail': detail})
    pitch = [abs(r['pitch']) for r in L]
    roll = [abs(r['roll']) for r in L]
    chk('Robot never tips (|roll| < 25 deg, |pitch| < 25 deg)', max(roll) < 25 and max(pitch) < 25,
        f'max |pitch| {max(pitch):.1f} deg, max |roll| {max(roll):.1f} deg')
    scr = sim.scrapes
    chk('Chassis / motors never touch an obstacle (only tyres do)', not scr,
        'no contacts' if not scr else f'{len(scr)} contact steps, first {scr[0]}')
    if scenario in ('A', 'B'):
        obj, target, half = 'cube50', BM.TOUCHDOWN, 0.025
        pick_t = next((r['t'] for r in L if r['phase'] == 'carry'), None)
        place_t = next((r['t'] for r in L if r['phase'] == 'release'), None)
        held = [r for r in L if pick_t and place_t and pick_t <= r['t'] < place_t]
        drops = [r for r in held if np.linalg.norm(np.array(r['cube50']) - np.array(r['grip50'])) > 0.02]
        chk('Pickup: cube lifted clear of the ground', any(r['cube50'][2] > 0.08 for r in L), f"max cube height {max(r['cube50'][2] for r in L) * 1000:.0f} mm")
        chk('Cube held by the gripper the whole way (no drop)', held and not drops,
            f'{len(held)} samples from pickup to touchdown, {len(drops)} with the cube > 20 mm from the grasp point')
        ymax = max(r['y'] for r in L)
        zmax = max(r['z'] for r in L)
        chk('Climbed the 15 deg ramp and crossed the 400 mm bridge', zmax > 0.075, f'max chassis lift {zmax * 1000:.1f} mm (bridge = 80)')
        last_breaker = -0.8 + max(BM.T.BREAKERS) / 1000 + 0.04
        rear_past = any(r['y'] - 0.076 > last_breaker for r in L)
        chk('Drove down and over all four 40 mm speed breakers', rear_past, f'rear axle passed y = {last_breaker:.3f} m')
    else:
        obj, target, half = 'cube60', BM.DOWNY_TARGET, 0.030
        chk('Downy: 60 mm cube taken off the 400 mm pole', any(r['cube60'][2] > 0.45 for r in L),
            f"max cube centre {max(r['cube60'][2] for r in L) * 1000:.0f} mm")
        tw_ = [r for r in L if r['phase'] == 'turn']
        moving = [r for r in tw_ if abs(r['wl']) + abs(r['wr']) > 0.05]
        yw = np.unwrap(np.radians([r['yaw'] for r in tw_]))
        dh = float(np.degrees(np.sum(np.abs(np.diff(yw)))))          # total rotation travelled
        fin = [e['value'] for e in sim.events if 'heading error' in e['event'] or e['event'] == 'turn_in_place_final_error_deg']
        res['turn'] = {'style': 'spin in place' if SPIN else 'reverse arc + forward arc', 'duration_s': round(len(tw_) / 100, 1),
                       'rotation_travelled_deg': round(dh, 1), 'final_aim_error_deg': fin[-1] if fin else None,
                       'peak_motor_current_A_total': round(max(r['i_motors'] for r in tw_), 2),
                       'mean_motor_current_A_total': round(float(np.mean([r['i_motors'] for r in moving])), 2) if moving else 0,
                       'mean_battery_current_A_motors': round(float(np.mean([r['i_batt_motors'] for r in moving])), 2) if moving else 0,
                       'time_above_80pct_stall_s': round(sum(1 for r in tw_ if r['stall_frac'] > 0.8) / 100, 2)}
        chk('Turned to face the target square (final aim error < 3 deg)', bool(fin) and abs(fin[-1]) < 3, json.dumps(res['turn']))
    c = np.array(L[-1][obj])
    tx, ty = target
    sq = 0.04 if scenario in ('A', 'B') else 0.05
    inside = abs(c[0] - tx) + half <= sq + 1e-4 and abs(c[1] - ty) + half <= sq + 1e-4 and c[2] < half + 0.005
    chk(f'Placed completely inside the {int(sq * 2000)} x {int(sq * 2000)} mm square', inside,
        f'cube centre offset ({(c[0] - tx) * 1000:+.1f}, {(c[1] - ty) * 1000:+.1f}) mm, z {c[2] * 1000:.1f} mm')
    if scenario == 'B':
        f0 = extra['fault']['start']
        if f0 is None:
            chk('Radio drop injected', False, 'robot never reached the bridge window')
        else:
            win = [r for r in L if f0 < round(r['t'] * 1000) <= f0 + 1500]   # record t is stamped after its tick
            after = [r for r in win if r['t'] >= f0 / 1000 + 0.26]
            chk('Forged + replayed packets during the drop are all rejected', all(r['acc'] == 0 for r in win),
                f"{sum(1 for r in win if r['mode'] in (2, 3))} attack packets, {sum(r['acc'] for r in win)} accepted")
            chk('Link lost -> firmware HOLD within 260 ms, wheel duty 0', after and all(r['state'] == 2 and r['wl'] == 0 and r['wr'] == 0 for r in after),
                f"state during drop: {sorted(set(r['state'] for r in after))}")
            v_end = after[-1]['v'] if after else None
            moved = math.hypot(after[-1]['x'] - after[0]['x'], after[-1]['y'] - after[0]['y']) if after else None
            chk('Robot stops (speed < 2 cm/s by the end of the drop)', v_end is not None and v_end < 0.02,
                f'speed {v_end * 1000:.0f} mm/s at end, rolled {moved * 1000:.0f} mm after HOLD')
            qd = np.array([r['q_meas'] for r in after])
            drift = float(np.max(np.ptp(qd, axis=0))) if len(qd) else 99
            chk('Arm motion ceases (joint drift < 2 deg during HOLD)', drift < 2.0, f'max joint drift {drift:.2f} deg')
            held = all(np.linalg.norm(np.array(r['cube50']) - np.array(r['grip50'])) < 0.02 for r in win)
            chk('Gripper keeps holding the cube through the drop', held, 'cube stayed in the jaws' if held else 'cube left the jaws')
            res['fault_window_s'] = [f0 / 1000, f0 / 1000 + 1.5]
    res['all_pass'] = all(c['pass'] for c in res['checks'])
    return res


def main():
    sc = (sys.argv[1] if len(sys.argv) > 1 else 'A').upper()
    video = '--novideo' not in sys.argv
    os.makedirs(OUT, exist_ok=True)
    sim = Sim(sc, video)
    extra = {}
    if sc in ('A', 'B'):
        op, ok, fault = scenario_AB(sim, faults=(sc == 'B'))
        extra['fault'] = fault
    else:
        scenario_C(sim)
    sim.close()
    res = evaluate(sim, sc, extra)
    tag = sc + ('_spin' if (sc == 'C' and SPIN) else '')
    tag += os.environ.get('TW_TAG', '')
    json.dump(res, open(os.path.join(OUT, f'mission_{tag}.json'), 'w'), indent=1)
    json.dump(sim.log[::5], open(os.path.join(OUT, f'mission_{tag}_log.json'), 'w'))
    for c in res['checks']:
        print(('PASS ' if c['pass'] else 'FAIL ') + c['check'] + ' — ' + c['detail'])
    for e in sim.events:
        print('  event', e)
    print(f"scenario {sc}: {'ALL PASS' if res['all_pass'] else 'FAILURES'} in {res['sim_time_s']:.1f} s simulated")
    if video and sim.frames:
        path = os.path.join(OUT, f'mission_{tag}.mp4')
        imageio.mimwrite(path, sim.frames, fps=30, quality=7, macro_block_size=4)
        n = len(sim.frames)
        for k, frac in enumerate((0.05, 0.2, 0.4, 0.6, 0.8, 0.98)):
            imageio.imwrite(os.path.join(OUT, f'mission_{tag}_{k}.png'), sim.frames[int(frac * (n - 1))])
        print('video', path, n, 'frames')


if __name__ == '__main__':
    main()
