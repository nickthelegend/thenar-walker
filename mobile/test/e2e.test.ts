// End-to-end test of the app's robot link + gamepad mapping against the firmware stand-in (test/mock_robot.py).
//   npm test          (starts the mock itself; needs python on PATH)
import { spawn } from 'node:child_process';
import assert from 'node:assert/strict';
import path from 'node:path';

import { PadMapper, BTN } from '../src/pad';
import { RobotLink } from '../src/robot';

const PORT = 8793 + Math.floor(Math.random() * 500);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const pad = (axes: number[] = [0, 0, 0, 0, 0, 0], buttons: number[] = []) => ({
  connected: true, name: 'Test Pad', axes, buttons: buttons.reduce((m, b) => m | (1 << b), 0),
});
let failures = 0;
async function test(name: string, fn: () => Promise<void>) {
  try {
    await fn();
    console.log('  ok   ' + name);
  } catch (e) {
    failures++;
    console.log('  FAIL ' + name + '\n       ' + (e instanceof Error ? e.message : e));
  }
}

async function main() {
  const mock = spawn('python', [path.join(__dirname, 'mock_robot.py'), String(PORT)], { stdio: ['ignore', 'pipe', 'inherit'] });
  await new Promise<void>((res) => mock.stdout!.once('data', () => res()));
  const robot = async () => (await (await fetch(`http://127.0.0.1:${PORT}/_state`)).json()) as any;

  // browsers throw "Illegal invocation" when fetch is called with another `this`; make Node just as strict
  const nodeFetch = globalThis.fetch;
  globalThis.fetch = function (this: unknown, ...args: Parameters<typeof fetch>) {
    if (this !== undefined && this !== globalThis) throw new TypeError('Illegal invocation');
    return nodeFetch(...args);
  } as typeof fetch;
  const link = new RobotLink(`127.0.0.1:${PORT}`);
  const mapper = new PadMapper(link);
  link.onTick = (dt) => mapper.step(dt);
  link.start(100);

  console.log(`Thenar Remote end-to-end (mock robot :${PORT})`);
  await test('connects and reads the calibration', async () => {
    await sleep(400);
    const s = link.getSnapshot();
    assert.equal(s.link, 'ok');
    assert.equal(s.cfg?.usable, 0b011111);
    assert.deepEqual(s.cfg?.hi, [85, 40, 30, 60, 85, 60]);
  });

  await test('All joints ON switches every calibrated joint on, J6 (not calibrated) stays off', async () => {
    link.allOn();
    await sleep(300);
    assert.equal((await robot()).on, 0b011111);
    assert.equal(link.getSnapshot().mask, 0b011111);
  });

  await test('slider targets are clamped to the safe ends and the arm slews at 90 deg/s', async () => {
    link.setJoint(0, 200);
    assert.equal(link.q[0], 85);
    await sleep(500);
    const r = await robot();
    assert.equal(r.goal[0], 85);
    assert.ok(r.cur[0] > 20 && r.cur[0] < 75, `cur ${r.cur[0]}`);
    await sleep(800);
    assert.equal((await robot()).cur[0], 85);
  });

  await test('on-screen joystick drives both sides forward at Mid speed (65 %)', async () => {
    link.touchDrive = { thr: 1, trn: 0 };
    await sleep(700);
    const r = await robot();
    assert.ok(Math.abs(r.wheel[0] - 0.65) < 1e-6 && Math.abs(r.wheel[1] - 0.65) < 1e-6, `wheels ${r.wheel}`);
    link.touchDrive = { thr: 0, trn: 0 };
    await sleep(300);
    assert.deepEqual((await robot()).wheel, [0, 0]);
  });

  await test('gamepad left stick wins over the screen; stick left spins left', async () => {
    link.touchDrive = { thr: 1, trn: 0 };
    mapper.update(pad([-1, 0, 0, 0, 0, 0]));
    await sleep(500);
    const r = await robot();
    assert.ok(r.wheel[0] < 0 && r.wheel[1] > 0, `wheels ${r.wheel}`);
    mapper.update(pad());
    link.touchDrive = { thr: 0, trn: 0 };
    await sleep(300);
  });

  await test('right stick moves J1 at ~60 deg/s, X halves... precision 20 deg/s', async () => {
    link.setJoint(0, 0);
    await sleep(300);
    mapper.update(pad([0, 0, -1, 0, 0, 0]));
    await sleep(1000);
    mapper.update(pad());
    await sleep(300);
    const fast = (await robot()).goal[0];
    assert.ok(fast < -45 && fast > -75, `after 1 s full stick: ${fast}`);
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.X]));
    await sleep(150);
    mapper.update(pad([0, 0, 1, 0, 0, 0]));
    await sleep(1000);
    mapper.update(pad());
    await sleep(300);
    const fine = (await robot()).goal[0] - fast;
    assert.ok(fine > 12 && fine < 28, `precision 1 s: ${fine}`);
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.X]));
    await sleep(150);
    mapper.update(pad());
  });

  await test('d-pad, Y/A and triggers move elbow, wrist and gripper the right way', async () => {
    link.zero();
    await sleep(300);
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.DOWN, BTN.A, BTN.RIGHT]));
    await sleep(500);
    mapper.update(pad());
    await sleep(300);
    const g = (await robot()).goal;
    assert.ok(g[2] > 15 && g[3] > 15 && g[4] > 15, `elbow/wrist/roll ${g}`);
    // gripper is not calibrated in the mock -> it must not move even with L2 held
    mapper.update(pad([0, 0, 0, 0, 1, 0]));
    await sleep(400);
    mapper.update(pad());
    assert.equal(link.q[5], 0);
  });

  await test('B = STOP: wheels stop and stay stopped while the stick is still pushed, free again once centred', async () => {
    mapper.update(pad([0, -1, 0, 0, 0, 0]));
    await sleep(500);
    assert.ok((await robot()).wheel[0] > 0.3);
    mapper.update(pad([0, -1, 0, 0, 0, 0], [BTN.B]));
    await sleep(300);
    let r = await robot();
    assert.deepEqual(r.wheel, [0, 0]);
    assert.equal(r.latch, true);
    await sleep(400);
    assert.deepEqual((await robot()).wheel, [0, 0], 'still latched with the stick held');
    mapper.update(pad());
    await sleep(300);
    r = await robot();
    assert.equal(r.latch, false);
  });

  await test('L1/R1 change speed; hold SELECT 1 s = limp, hold START 1 s = arm on', async () => {
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.R1]));
    await sleep(200);
    mapper.update(pad());
    await sleep(250);
    assert.equal((await robot()).speed, 2);
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.SELECT]));
    await sleep(600);
    assert.equal((await robot()).on, 0b011111, 'not yet after 0.6 s');
    await sleep(700);
    assert.equal((await robot()).on, 0);
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.START]));
    await sleep(1400);
    mapper.update(pad());
    await sleep(200);
    assert.equal((await robot()).on, 0b011111);
    mapper.update(pad([0, 0, 0, 0, 0, 0], [BTN.L1]));
    await sleep(200);
    mapper.update(pad());
  });

  await test('another controller moving the arm (robot-side pad) shows up in the app', async () => {
    await fetch(`http://127.0.0.1:${PORT}/c?d=0,0&q=10,-20,5,0,0,0`);
    await sleep(400);
    assert.deepEqual(link.getSnapshot().q.slice(0, 3), [10, -20, 5]);
  });

  await test('gamepad switched off -> its drive input drops to zero', async () => {
    mapper.update(pad([0, -1, 0, 0, 0, 0]));
    await sleep(400);
    assert.ok((await robot()).wheel[0] > 0);
    mapper.update({ connected: false, name: '', axes: [0, 0, 0, 0, 0, 0], buttons: 0 });
    await sleep(300);
    assert.deepEqual((await robot()).wheel, [0, 0]);
  });

  await test('app goes quiet -> robot stops the wheels within 0.5 s and holds the arm', async () => {
    link.touchDrive = { thr: 1, trn: 0 };
    await sleep(500);
    assert.ok((await robot()).wheel[0] > 0);
    link.stopLoop();
    await sleep(700);
    const r = await robot();
    assert.deepEqual(r.wheel, [0, 0]);
    assert.deepEqual(r.goal, r.cur);
    link.touchDrive = { thr: 0, trn: 0 };
  });

  await test('robot unreachable -> app shows link lost', async () => {
    link.start(100);
    await sleep(300);
    mock.kill();
    await sleep(900);
    assert.equal(link.getSnapshot().link, 'lost');
  });

  link.stopLoop();
  mock.kill();
  console.log(failures ? `${failures} FAILED` : 'all passed');
  process.exit(failures ? 1 : 0);
}

main();
