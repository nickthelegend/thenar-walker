// Thenar Walker robot core: failsafe state machine, skid-steer mixing, arm slew limiting.
// Pure logic (no Arduino calls) so it is unit tested on the host (firmware/tests).
//
// RoboReach disconnect rule: if the link drops, drive motors stop, arm motion ceases, the
// gripper stays where it was, and the bot stays disabled until a NEW valid command arrives.
// => HOLD: wheel outputs zero (brake), arm goals frozen at the current position, servo PWM
//    left ON (an MG996R without PWM goes limp and would drop the cube).
#pragma once
#include <math.h>
#include <stdint.h>
#include "tw_link.h"

namespace tw {

constexpr uint32_t LINK_TIMEOUT_MS = 250;
constexpr float ENGAGE_TOL_DEG = 6.0f;          // leader must match the robot's STOW pose to engage
constexpr float ARM_SLEW_DPS = 180.0f;          // normal arm speed limit per joint
constexpr float RESUME_SLEW_DPS = 45.0f;        // gentle catch-up for 1.5 s after a HOLD
constexpr uint32_t RESUME_MS = 1500;
constexpr float DRIVE_ACCEL_PER_S = 2.5f;       // full scale in 0.4 s (keeps the arm-loaded bot from pitching)
constexpr float DRIVE_DEADBAND = 0.05f;
constexpr float SPEED_SCALE[3] = {0.35f, 0.65f, 1.0f};
// Skid steer (wheelbase 152 ~ track 168): turning scrubs the tyres sideways. With hard PETG front
// tyres + TPU rear (sim/turn_study.py) it spins from ~30 % duty; rotation gets its own scale and a
// breakaway floor so a gentle stick input still turns, without making precision mode twitchy.
constexpr float TURN_SCALE[3] = {0.5f, 0.75f, 1.0f};
constexpr float SPIN_MIN = 0.35f;

// Joint software limits (thenar-arms model.h) and the start pose (cad/tools/design.py POSES['STOW']).
constexpr float Q_LO[6] = {-85, -80, -80, -80, -85, 0};
constexpr float Q_HI[6] = {85, 80, 80, 80, 85, 70};
constexpr float Q_STOW[6] = {0, -80, 80, 50, 85, 0};

enum State : uint8_t { DISARMED = 0, ACTIVE = 1, HOLD = 2 };
enum Reason : uint8_t { R_NONE = 0, R_BOOT, R_LINK_LOST, R_OPERATOR_DISABLE, R_BAD_TARGET, R_NOT_AT_STOW };

struct Outputs {
  float wheel[2];      // left, right: -1..1
  bool servo_pwm_on;   // PCA9685 outputs enabled
  float q[6];          // servo targets actually sent this tick (deg)
};

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline void mix(float throttle, float turn, float &l, float &r) {
  if (fabsf(throttle) < DRIVE_DEADBAND) throttle = 0;
  if (fabsf(turn) < DRIVE_DEADBAND) turn = 0;
  l = throttle - turn;
  r = throttle + turn;
  float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(r)));
  l /= m;
  r /= m;
}

inline bool valid_q(const float *q) {
  for (int i = 0; i < 6; i++)
    if (!(q[i] >= Q_LO[i] && q[i] <= Q_HI[i])) return false;  // also rejects NaN
  return true;
}

struct Robot {
  State state = DISARMED;
  Reason reason = R_BOOT;
  uint32_t last_cmd_ms = 0;
  uint32_t resume_until = 0;
  bool pwm_on = false;
  float goal[6], cur[6];
  float wheel_target[2] = {0, 0}, wheel[2] = {0, 0};
  uint32_t last_tick = 0;
  bool ticked = false;

  Robot() {
    for (int i = 0; i < 6; i++) goal[i] = cur[i] = Q_STOW[i];
  }

  void enter_hold(Reason r) {
    state = HOLD;
    reason = r;
    wheel_target[0] = wheel_target[1] = 0;
    wheel[0] = wheel[1] = 0;                   // stop immediately, no ramp-down
    for (int i = 0; i < 6; i++) goal[i] = cur[i];  // arm motion ceases where it is
  }

  // A command that already passed CmdGate (authentic, right session, newer seq).
  void on_command(uint32_t now, const Cmd &c) {
    float q[6];
    for (int i = 0; i < 6; i++) q[i] = c.q_cdeg[i] / 100.0f;
    last_cmd_ms = now;
    if (!(c.flags & F_ENABLE)) {
      if (state == ACTIVE) enter_hold(R_OPERATOR_DISABLE);
      return;
    }
    if (!valid_q(q)) {
      if (state == ACTIVE) enter_hold(R_BAD_TARGET);
      return;
    }
    if (state == DISARMED) {
      for (int i = 0; i < 6; i++)
        if (fabsf(q[i] - Q_STOW[i]) > ENGAGE_TOL_DEG) { reason = R_NOT_AT_STOW; return; }
      for (int i = 0; i < 6; i++) cur[i] = goal[i] = q[i];
      pwm_on = true;
    } else if (state == HOLD) {
      resume_until = now + RESUME_MS;
    }
    state = ACTIVE;
    reason = R_NONE;
    if (c.flags & F_ARM)
      for (int i = 0; i < 6; i++) goal[i] = q[i];
    if (c.flags & F_DRIVE) {
      int sp = c.speed > 2 ? 2 : c.speed;
      float thr = c.throttle / 100.0f, trn = c.turn / 100.0f;
      if (fabsf(thr) < DRIVE_DEADBAND) thr = 0;
      if (fabsf(trn) < DRIVE_DEADBAND) trn = 0;
      if (thr == 0 && trn != 0) {          // spin in place: breakaway floor + turn scale
        float mag = SPIN_MIN + (TURN_SCALE[sp] - SPIN_MIN) * fabsf(trn);
        wheel_target[0] = trn > 0 ? -mag : mag;
        wheel_target[1] = -wheel_target[0];
      } else {
        float l = thr * SPEED_SCALE[sp] - trn * TURN_SCALE[sp], r = thr * SPEED_SCALE[sp] + trn * TURN_SCALE[sp];
        float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(r)));
        wheel_target[0] = l / m;
        wheel_target[1] = r / m;
      }
    } else {
      wheel_target[0] = wheel_target[1] = 0;
    }
  }

  Outputs tick(uint32_t now) {
    float dt = ticked ? (now - last_tick) / 1000.0f : 0.0f;
    last_tick = now;
    ticked = true;
    if (state == ACTIVE && uint32_t(now - last_cmd_ms) > LINK_TIMEOUT_MS) enter_hold(R_LINK_LOST);
    if (state == ACTIVE) {
      float rate = int32_t(resume_until - now) > 0 ? RESUME_SLEW_DPS : ARM_SLEW_DPS;
      float step = rate * dt;
      for (int i = 0; i < 6; i++) cur[i] += clampf(goal[i] - cur[i], -step, step);
      float a = DRIVE_ACCEL_PER_S * dt;
      for (int k = 0; k < 2; k++) {
        float t = wheel_target[k], w = wheel[k];
        if (t * w < 0) w = 0;                    // reversing: stop first
        else if (fabsf(t) <= fabsf(w)) w = t;    // slowing / stopping: immediate
        else w += clampf(t - w, -a, a);          // speeding up: ramped
        wheel[k] = w;
      }
    }
    Outputs o;
    o.wheel[0] = state == ACTIVE ? wheel[0] : 0;
    o.wheel[1] = state == ACTIVE ? wheel[1] : 0;
    o.servo_pwm_on = pwm_on;       // once engaged, PWM stays on: HOLD keeps the gripper closed
    for (int i = 0; i < 6; i++) o.q[i] = cur[i];
    return o;
  }
};

}  // namespace tw
