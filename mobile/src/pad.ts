// Game controller layout — identical to the robot's own Bluepad32 mapping and the robot web page:
//   left stick drive | right stick J1 pan (x), J2 shoulder (y) | d-pad up/down J3 elbow, left/right J5 roll
//   Y / A J4 wrist up / down | L2 / R2 gripper open / close | L1 / R1 speed | B STOP | X precision arm
//   hold START 1 s = arm on | hold SELECT 1 s = arm limp
import type { GamepadState } from '../modules/thenar-native';
import { RobotLink } from './robot';

export const PAD_DPS = 60;
export const PAD_FINE_DPS = 20;
export const HOLD_MS = 1000;
const DEAD = 0.12;

export const BTN = { A: 0, B: 1, X: 2, Y: 3, L1: 4, R1: 5, L2: 6, R2: 7, SELECT: 8, START: 9, UP: 12, DOWN: 13, LEFT: 14, RIGHT: 15 };

export function deadzone(v: number) {
  return Math.abs(v) < DEAD ? 0 : (v - Math.sign(v) * DEAD) / (1 - DEAD);
}

export const NO_PAD: GamepadState = { connected: false, name: '', axes: [0, 0, 0, 0, 0, 0], buttons: 0 };

export class PadMapper {
  state: GamepadState = NO_PAD;
  fine = false;
  private prev = 0;
  private holdStart: Record<number, number> = {};

  constructor(private link: RobotLink, private now = () => Date.now()) {}

  update(s: GamepadState) {
    this.state = s;
    if (!s.connected) this.link.padDrive = { thr: 0, trn: 0 };
  }

  // called by the link every tick, before it talks to the robot
  step(dt: number) {
    const s = this.state, link = this.link;
    if (!s.connected) {
      link.padDrive = { thr: 0, trn: 0 };
      this.prev = 0;
      return;
    }
    const b = s.buttons;
    const down = (i: number) => ((b >> i) & 1) === 1;
    const edge = (i: number) => down(i) && !((this.prev >> i) & 1);
    const ax = (i: number) => deadzone(s.axes[i] ?? 0);
    link.padDrive = { thr: -ax(1), trn: -ax(0) }; // stick up = forward, stick left = turn left
    if (edge(BTN.B)) link.stop();
    if (edge(BTN.R1)) link.setSpeed(link.sp + 1);
    if (edge(BTN.L1)) link.setSpeed(link.sp - 1);
    if (edge(BTN.X)) this.fine = !this.fine;
    const t = this.now();
    for (const [btn, act] of [[BTN.START, () => link.allOn()], [BTN.SELECT, () => link.limp()]] as const) {
      if (!down(btn)) delete this.holdStart[btn];
      else if (this.holdStart[btn] === undefined) this.holdStart[btn] = t;
      else if (t - this.holdStart[btn] >= HOLD_MS) { act(); this.holdStart[btn] = Infinity; }
    }
    const l2 = Math.max(down(BTN.L2) ? 1 : 0, s.axes[4] ?? 0), r2 = Math.max(down(BTN.R2) ? 1 : 0, s.axes[5] ?? 0);
    const v = [
      ax(2), // pan: right = + (clockwise from above)
      -ax(3), // shoulder: up = + (lean forward)
      +down(BTN.DOWN) - +down(BTN.UP), // elbow: down = + (bend down)
      +down(BTN.A) - +down(BTN.Y), // wrist flex: A = + (down)
      +down(BTN.RIGHT) - +down(BTN.LEFT), // roll: right = + (clockwise)
      l2 - r2, // gripper: L2 opens, R2 closes
    ];
    const k = (this.fine ? PAD_FINE_DPS : PAD_DPS) * dt;
    link.moveJoints(v.map((x) => x * k));
    this.prev = b;
  }
}
