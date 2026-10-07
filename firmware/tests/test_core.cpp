// Host unit tests for the ThenarLink headers (the same code the ESP32s run).
// Build + run:  powershell -File firmware/tests/run_tests.ps1
#include <cmath>
#include <cstdio>
#include <cstring>
#include "../libraries/ThenarLink/src/tw_robot.h"

static int fails = 0, checks = 0;
#define CHECK(cond)                                                              \
  do {                                                                           \
    checks++;                                                                    \
    if (!(cond)) { fails++; std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); } \
  } while (0)

static void hex(const uint8_t *d, size_t n, char *out) {
  for (size_t i = 0; i < n; i++) std::sprintf(out + 2 * i, "%02x", d[i]);
}

static const uint8_t KEY[32] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16,
                                17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32};

static tw::Cmd cmd(uint32_t session, uint32_t seq, uint8_t flags, int thr, int turn, const float *q, uint8_t speed = 2) {
  tw::Cmd c;
  std::memset(&c, 0, sizeof c);
  c.magic = tw::MAGIC_CMD; c.session = session; c.seq = seq; c.flags = flags;
  c.throttle = int8_t(thr); c.turn = int8_t(turn); c.speed = speed;
  for (int i = 0; i < 6; i++) c.q_cdeg[i] = int16_t(std::lround(q[i] * 100));
  tw::sign(c, KEY);
  return c;
}

static void test_sha() {
  char h[65];
  uint8_t out[32];
  tw::Sha256 s; s.init(); s.update("abc", 3); s.final(out); hex(out, 32, h);
  CHECK(!std::strcmp(h, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"));
  s.init(); s.final(out); hex(out, 32, h);
  CHECK(!std::strcmp(h, "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"));
  const char *m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";   // 2-block message
  s.init(); s.update(m, std::strlen(m)); s.final(out); hex(out, 32, h);
  CHECK(!std::strcmp(h, "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"));
  // RFC 4231 test case 2
  tw::hmac_sha256((const uint8_t *)"Jefe", 4, "what do ya want for nothing?", 28, out); hex(out, 32, h);
  CHECK(!std::strcmp(h, "5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"));
}

static void test_name() {
  char n[40];
  tw::device_name("Thenar Walker", n, sizeof n);
  CHECK(!std::strcmp(n, "ThenarWalker_TF"));
  tw::device_name("The Extremely Long Robotics Team", n, sizeof n);   // 28 non-space chars
  CHECK(!std::strcmp(n, "TheExtremelyLongRobo_TF"));
  CHECK(std::strlen(n) == 20 + 3);
}

static void test_gate() {
  float q[6] = {0, -80, 80, 50, 85, 0};
  tw::CmdGate g; g.new_session(0xABCD1234);
  tw::Cmd c = cmd(0xABCD1234, 1, tw::F_ENABLE, 0, 0, q), out;
  CHECK(g.accept((uint8_t *)&c, sizeof c, KEY, out));
  CHECK(!g.accept((uint8_t *)&c, sizeof c, KEY, out));                    // exact replay
  tw::Cmd c2 = cmd(0xABCD1234, 2, tw::F_ENABLE, 50, 0, q);
  tw::Cmd bad = c2; bad.throttle = 100;                                   // tampered after signing
  CHECK(!g.accept((uint8_t *)&bad, sizeof bad, KEY, out));
  uint8_t other[32]; std::memcpy(other, KEY, 32); other[0] ^= 1;
  tw::Cmd forged = c2; tw::sign(forged, other);                           // attacker without the key
  CHECK(!g.accept((uint8_t *)&forged, sizeof forged, KEY, out));
  CHECK(g.accept((uint8_t *)&c2, sizeof c2, KEY, out));
  tw::Cmd old = cmd(0xABCD1234, 2, tw::F_ENABLE, 0, 0, q);
  CHECK(!g.accept((uint8_t *)&old, sizeof old, KEY, out));               // not newer
  tw::Cmd wrongSession = cmd(0x11111111, 9, tw::F_ENABLE, 0, 0, q);
  CHECK(!g.accept((uint8_t *)&wrongSession, sizeof wrongSession, KEY, out));
  CHECK(!g.accept((uint8_t *)&c2, sizeof c2 - 1, KEY, out));             // wrong length
  g.new_session(0x5555);                                                  // re-pair: old session dead
  tw::Cmd c3 = cmd(0xABCD1234, 99, tw::F_ENABLE, 0, 0, q);
  CHECK(!g.accept((uint8_t *)&c3, sizeof c3, KEY, out));
}

static void test_robot() {
  using namespace tw;
  Robot r;
  uint32_t t = 1000;
  float stow[6] = {0, -80, 80, 50, 85, 0};
  float far[6] = {0, 0, 0, 0, 0, 20};
  Outputs o = r.tick(t);
  CHECK(r.state == DISARMED && !o.servo_pwm_on && o.wheel[0] == 0);
  // leader not at STOW: refuse to engage (prevents the arm jumping on power-up)
  r.on_command(t, cmd(1, 1, F_ENABLE | F_DRIVE | F_ARM, 100, 0, far));
  o = r.tick(t += 10);
  CHECK(r.state == DISARMED && r.reason == R_NOT_AT_STOW && !o.servo_pwm_on && o.wheel[0] == 0);
  // engage at STOW
  r.on_command(t, cmd(1, 2, F_ENABLE | F_DRIVE | F_ARM, 100, 0, stow));
  o = r.tick(t += 10);
  CHECK(r.state == ACTIVE && o.servo_pwm_on);
  CHECK(o.wheel[0] > 0 && o.wheel[0] < 0.1f);                // ramped, not a step
  for (int k = 0; k < 60; k++) { r.on_command(t, cmd(1, 3 + k, F_ENABLE | F_DRIVE | F_ARM, 100, 0, stow)); o = r.tick(t += 10); }
  CHECK(std::fabs(o.wheel[0] - 1.0f) < 1e-4f && std::fabs(o.wheel[1] - 1.0f) < 1e-4f);
  // pure turn left: left wheel back, right wheel forward; reversing stops first
  r.on_command(t, cmd(1, 100, F_ENABLE | F_DRIVE | F_ARM, 0, 100, stow));
  o = r.tick(t += 10);
  CHECK(o.wheel[0] == 0.0f && o.wheel[1] == 1.0f);
  // move the arm towards a target, slew limited to 180 deg/s
  float tgt[6] = {30, -80, 80, 50, 85, 40};
  r.on_command(t, cmd(1, 101, F_ENABLE | F_ARM, 0, 0, tgt));
  o = r.tick(t += 100);
  CHECK(std::fabs(o.q[0] - 18.0f) < 0.01f);                   // 0.1 s * 180 deg/s
  CHECK(o.wheel[0] == 0 && o.wheel[1] == 0);                  // F_DRIVE cleared -> wheels stop
  // ---- link loss: no packets for > 250 ms
  float qBefore[6]; std::memcpy(qBefore, o.q, sizeof qBefore);
  r.on_command(t, cmd(1, 102, F_ENABLE | F_DRIVE | F_ARM, 100, 0, tgt));
  o = r.tick(t += 10);
  float q1 = o.q[0];
  o = r.tick(t += 300);
  CHECK(r.state == HOLD && r.reason == R_LINK_LOST);
  CHECK(o.wheel[0] == 0 && o.wheel[1] == 0);                  // motors stop
  CHECK(o.servo_pwm_on);                                      // gripper keeps holding the cube
  float frozen = o.q[0];
  CHECK(frozen >= q1 && frozen < 30.0f);
  for (int k = 0; k < 50; k++) o = r.tick(t += 20);
  CHECK(o.q[0] == frozen && o.q[5] == r.cur[5]);             // arm motion ceased
  CHECK(r.state == HOLD && o.wheel[0] == 0);                  // stays disabled with no new command
  // the old, already-seen command is rejected by the gate before it reaches Robot (tested above);
  // a NEW valid command resumes, gently
  r.on_command(t, cmd(1, 103, F_ENABLE | F_DRIVE | F_ARM, 0, 0, tgt));
  o = r.tick(t += 100);
  CHECK(r.state == ACTIVE);
  CHECK(std::fabs(o.q[0] - (frozen + 4.5f)) < 0.01f || o.q[0] == 30.0f);   // 45 deg/s resume rate
  // operator disable (ENABLE toggle off) -> HOLD immediately
  r.on_command(t, cmd(1, 104, 0, 100, 0, tgt));
  o = r.tick(t += 10);
  CHECK(r.state == HOLD && r.reason == R_OPERATOR_DISABLE && o.wheel[0] == 0 && o.servo_pwm_on);
  // out-of-range / NaN target -> HOLD, never commanded
  r.on_command(t, cmd(1, 105, F_ENABLE | F_ARM, 0, 0, tgt));
  o = r.tick(t += 10);
  CHECK(r.state == ACTIVE);
  float badq[6] = {0, -80, 80, 50, 85, 95};
  r.on_command(t, cmd(1, 106, F_ENABLE | F_ARM, 0, 0, badq));
  o = r.tick(t += 10);
  CHECK(r.state == HOLD && r.reason == R_BAD_TARGET && o.q[5] <= 70);
  // spin in place in precision mode: breakaway floor, opposite wheels
  {
    Robot r2;
    uint32_t t2 = 5000;
    r2.on_command(t2, cmd(1, 1, F_ENABLE | F_DRIVE | F_ARM, 0, 20, stow, 0));
    for (int k = 0; k < 100; k++) { r2.on_command(t2, cmd(1, 2 + k, F_ENABLE | F_DRIVE | F_ARM, 0, 20, stow, 0)); o = r2.tick(t2 += 10); }
    CHECK(o.wheel[0] < -0.35f && o.wheel[1] > 0.35f && std::fabs(o.wheel[0] + o.wheel[1]) < 1e-5f);
    // driving straight in precision mode stays at 35 %
    for (int k = 0; k < 100; k++) { r2.on_command(t2, cmd(1, 200 + k, F_ENABLE | F_DRIVE | F_ARM, 100, 0, stow, 0)); o = r2.tick(t2 += 10); }
    CHECK(std::fabs(o.wheel[0] - 0.35f) < 1e-4f && std::fabs(o.wheel[1] - 0.35f) < 1e-4f);
  }
  float l, rr;
  mix(0.03f, 0.02f, l, rr);
  CHECK(l == 0 && rr == 0);                                   // deadband
  mix(1, 1, l, rr);
  CHECK(std::fabs(l - 0) < 1e-6 && std::fabs(rr - 1) < 1e-6);
}

int main() {
  test_sha();
  test_name();
  test_gate();
  test_robot();
  std::printf("%d/%d checks passed\n", checks - fails, checks);
  return fails ? 1 : 0;
}
