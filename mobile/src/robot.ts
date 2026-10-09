// Link to the robot firmware (firmware/thenar_phone): HTTP on the robot's own Wi-Fi AP, polled at 10 Hz.
//   GET /cfg  -> joint limits, calibration state, current targets
//   GET /c?d=thr,turn[&s=speed][&m=mask][&q=q0..q5]   thr/turn -100..100, turn > 0 = left
//   GET /c?stop=1      wheels stop (latched until every stick is centred), arm holds
//   GET /forgetpad     the robot forgets its Bluetooth gamepad
// q and m are only sent when this app changed them, so a gamepad paired to the robot can move the arm at the same time;
// otherwise the robot's reply is the truth. The robot stops the wheels by itself if the app goes quiet for 0.5 s.

export const JOINTS = [
  { id: 'J1', name: 'Base pan' },
  { id: 'J2', name: 'Shoulder lift' },
  { id: 'J3', name: 'Elbow' },
  { id: 'J4', name: 'Wrist flex' },
  { id: 'J5', name: 'Wrist roll' },
  { id: 'J6', name: 'Gripper' },
] as const;
export const GRIPPER = 5;
export const SPEEDS = ['Slow', 'Mid', 'Fast'] as const;

export type Reply = {
  bat: number;
  on: number;
  sp: number;
  q: number[]; // where each joint is now
  g: number[]; // where each joint is going
  pad?: string; // gamepad paired to the robot ("" = none); missing on builds without Bluepad32
  padmem?: number;
};
export type Cfg = Reply & { ssid: string; cal: number; pca: number; bt: number; usable: number; lo: number[]; hi: number[] };

export type LinkState = 'connecting' | 'ok' | 'lost';
export type Snapshot = {
  link: LinkState;
  cfg: Cfg | null;
  bat: number;
  robotPad: string;
  q: number[]; // targets shown on the sliders
  cur: number[];
  mask: number;
  sp: number;
  error: string;
};

type Fetch = (url: string, init?: { signal?: AbortSignal; cache?: string }) => Promise<{ ok: boolean; json(): Promise<unknown> }>;

const clamp = (v: number, a: number, b: number) => Math.min(b, Math.max(a, v));
const ZERO6 = () => [0, 0, 0, 0, 0, 0];

export class RobotLink {
  host: string;
  q = ZERO6();
  cur = ZERO6();
  mask = 0;
  sp = 1;
  touching = -1; // slider being dragged: replies do not move it
  touchDrive = { thr: 0, trn: 0 }; // on-screen joystick
  padDrive = { thr: 0, trn: 0 }; // gamepad left stick (wins while pushed)
  onTick: ((dt: number) => void) | null = null;

  private cfg: Cfg | null = null;
  private qEdit = 0;
  private mEdit = 0;
  private sEdit = 0;
  private qSent = 0;
  private mSent = 0;
  private sSent = 0;
  private stopReq = false;
  private busy = false;
  private fails = 0;
  private bat = 0;
  private robotPad = '';
  private link: LinkState = 'connecting';
  private error = '';
  private timer: ReturnType<typeof setInterval> | null = null;
  private lastTick = 0;
  private listeners = new Set<() => void>();
  private snap: Snapshot;

  // fetch is wrapped: calling the browser's fetch as a method of another object throws "Illegal invocation"
  constructor(host: string, private fetchImpl: Fetch = (u, i) => fetch(u, i as RequestInit) as unknown as ReturnType<Fetch>, private timeoutMs = 450) {
    this.host = host;
    this.snap = this.makeSnap();
  }

  // ---------------------------------------------------------------- React glue (useSyncExternalStore)
  subscribe = (fn: () => void) => {
    this.listeners.add(fn);
    return () => this.listeners.delete(fn);
  };
  getSnapshot = () => this.snap;
  private changed() {
    this.snap = this.makeSnap();
    this.listeners.forEach((fn) => fn());
  }
  private makeSnap(): Snapshot {
    return {
      link: this.link, cfg: this.cfg, bat: this.bat, robotPad: this.robotPad, q: this.q.slice(), cur: this.cur.slice(),
      mask: this.mask, sp: this.sp, error: this.error,
    };
  }

  // ---------------------------------------------------------------- lifecycle
  start(periodMs = 100) {
    if (this.timer) return;
    this.lastTick = Date.now();
    this.timer = setInterval(() => void this.tick(), periodMs);
  }
  stopLoop() {
    if (this.timer) clearInterval(this.timer);
    this.timer = null;
  }
  setHost(host: string) {
    this.host = host.trim();
    this.cfg = null;
    this.link = 'connecting';
    this.changed();
  }
  get base() {
    return 'http://' + this.host;
  }

  // ---------------------------------------------------------------- actions (all cheap; sent on the next tick)
  usable(i: number) {
    return !!this.cfg && ((this.cfg.usable >> i) & 1) === 1;
  }
  setJoint(i: number, v: number) {
    if (!this.cfg) return;
    this.q[i] = clamp(v, this.cfg.lo[i], this.cfg.hi[i]);
    this.qEdit++;
    this.changed();
  }
  nudge(i: number, d: number) {
    this.setJoint(i, this.q[i] + d);
  }
  moveJoints(v: number[]) {   // gamepad: add v[i] degrees to every joint that is on
    if (!this.cfg) return;
    let moved = false;
    for (let i = 0; i < 6; i++)
      if (v[i] && (this.mask >> i) & 1) {
        this.q[i] = clamp(this.q[i] + v[i], this.cfg.lo[i], this.cfg.hi[i]);
        moved = true;
      }
    if (moved) { this.qEdit++; this.changed(); }
  }
  setMask(m: number) {
    this.mask = this.cfg ? m & this.cfg.usable : 0;
    this.mEdit++;
    this.changed();
  }
  toggle(i: number) {
    this.setMask(this.mask ^ (1 << i));
  }
  allOn() {
    if (this.cfg) this.setMask(this.cfg.usable);
  }
  limp() {
    this.setMask(0);
  }
  zero() {
    if (!this.cfg) return;
    for (let i = 0; i < 6; i++) this.q[i] = clamp(0, this.cfg.lo[i], this.cfg.hi[i]);
    this.qEdit++;
    this.changed();
  }
  gripper(open: boolean) {
    if (this.cfg) this.setJoint(GRIPPER, open ? this.cfg.hi[GRIPPER] : this.cfg.lo[GRIPPER]);
  }
  setSpeed(s: number) {
    this.sp = clamp(Math.round(s), 0, 2);
    this.sEdit++;
    this.changed();
  }
  stop() {
    this.touchDrive = { thr: 0, trn: 0 };
    this.stopReq = true;
  }
  async forgetPad() {
    await this.get('/forgetpad');
  }

  // ---------------------------------------------------------------- the 10 Hz exchange
  private async get(path: string): Promise<unknown> {
    const ac = new AbortController();
    const to = setTimeout(() => ac.abort(), this.timeoutMs);
    try {
      const r = await this.fetchImpl(this.base + path, { signal: ac.signal, cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP error');
      return await r.json();
    } finally {
      clearTimeout(to);
    }
  }

  drive() {
    const p = this.padDrive;
    return p.thr || p.trn ? p : this.touchDrive;
  }

  async tick() {
    const now = Date.now();
    const dt = Math.min(0.2, (now - this.lastTick) / 1000);
    this.lastTick = now;
    this.onTick?.(dt);
    if (this.busy) return;
    this.busy = true;
    try {
      if (!this.cfg) {
        const c = (await this.get('/cfg')) as Cfg;
        this.cfg = c;
        this.q = c.g.slice();
        this.cur = c.q.slice();
        this.mask = c.on;
        this.sp = c.sp;
        this.qSent = this.qEdit;
        this.mSent = this.mEdit;
        this.sSent = this.sEdit;
        this.accept(c);
        return;
      }
      const sq = this.qEdit, sm = this.mEdit, ss = this.sEdit;
      let path: string;
      if (this.stopReq) {
        this.stopReq = false;
        path = '/c?stop=1';
      } else {
        const d = this.drive();
        path = `/c?d=${Math.round(clamp(d.thr, -1, 1) * 100)},${Math.round(clamp(d.trn, -1, 1) * 100)}`;
        if (ss !== this.sSent) path += `&s=${this.sp}`;
        if (sm !== this.mSent) path += `&m=${this.mask}`;
        if (sq !== this.qSent) path += `&q=${this.q.map((v) => v.toFixed(1)).join(',')}`;
      }
      const r = (await this.get(path)) as Reply;
      this.qSent = sq;
      this.mSent = sm;
      this.sSent = ss;
      if (this.qEdit === sq && this.touching < 0) this.q = r.g.slice();
      if (this.mEdit === sm) this.mask = r.on;
      if (this.sEdit === ss) this.sp = r.sp;
      this.cur = r.q.slice();
      this.accept(r);
    } catch (e) {
      if (++this.fails > 2 && this.link !== 'lost') {
        this.link = this.cfg ? 'lost' : 'connecting';
        this.error = e instanceof Error ? e.message : String(e);
        this.changed();
      }
    } finally {
      this.busy = false;
    }
  }

  private accept(r: Reply) {
    this.fails = 0;
    this.link = 'ok';
    this.error = '';
    this.bat = r.bat;
    this.robotPad = r.pad ?? '';
    this.changed();
  }
}
