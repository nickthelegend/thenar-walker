// Firmware-in-the-loop bridge: runs the robot's real ThenarLink code (packet signing, CmdGate,
// tw::Robot state machine) for the MuJoCo simulator, one control tick per stdin line.
//
// stdin  (one line per 10 ms tick):
//   T <t_ms> <mode> <flags> <throttle> <turn> <speed> <q0..q5 deg>
//      mode 0 = no packet this tick (radio silent / dropped)
//      mode 1 = controller sends a correctly signed packet
//      mode 2 = attacker: same content signed with a WRONG key (must be ignored)
//      mode 3 = attacker: replays the last packet the controller sent (must be ignored)
// stdout (one line per tick):
//   <state> <reason> <wheelL> <wheelR> <pwm_on> <q0..q5 deg> <accepted 0/1>
#include <cstdio>
#include <cstring>
#include <cmath>
#include "../firmware/libraries/ThenarLink/src/tw_robot.h"

static const uint8_t KEY[32] = {0x3a, 0x91, 0x07, 0xc4, 0x5e, 0x22, 0xb8, 0x6f, 0x10, 0xd3, 0x77, 0x4a, 0x99, 0x01, 0xee, 0x5b,
                                0x68, 0x2c, 0xf0, 0x13, 0xa7, 0x45, 0x8e, 0xb2, 0x3d, 0xc9, 0x56, 0x0f, 0x84, 0x6a, 0x1e, 0xd7};

int main() {
  tw::Robot robot;
  tw::CmdGate gate;
  const uint32_t session = 0x5A17C0DE;   // what the robot would hand out in WELCOME
  gate.new_session(session);
  uint32_t seq = 0;
  tw::Cmd last;
  bool haveLast = false;
  char line[512];
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  while (std::fgets(line, sizeof line, stdin)) {
    unsigned t; int mode, flags, thr, turn, speed; float q[6];
    if (std::sscanf(line, "T %u %d %d %d %d %d %f %f %f %f %f %f", &t, &mode, &flags, &thr, &turn, &speed,
                    q, q + 1, q + 2, q + 3, q + 4, q + 5) != 12) {
      std::printf("ERR parse\n");
      continue;
    }
    int accepted = 0;
    if (mode != 0) {
      tw::Cmd c;
      std::memset(&c, 0, sizeof c);
      c.magic = tw::MAGIC_CMD; c.session = session; c.flags = uint8_t(flags);
      c.throttle = int8_t(thr); c.turn = int8_t(turn); c.speed = uint8_t(speed);
      for (int i = 0; i < 6; i++) c.q_cdeg[i] = int16_t(std::lround(q[i] * 100));
      if (mode == 1) {
        c.seq = ++seq;
        tw::sign(c, KEY);
        last = c; haveLast = true;
      } else if (mode == 2) {
        uint8_t wrong[32]; std::memcpy(wrong, KEY, 32); wrong[7] ^= 0x40;
        c.seq = seq + 1000;
        tw::sign(c, wrong);
      } else if (mode == 3 && haveLast) {
        c = last;   // byte-identical replay
      }
      tw::Cmd out;
      if (gate.accept(reinterpret_cast<const uint8_t *>(&c), sizeof c, KEY, out)) {
        robot.on_command(t, out);
        accepted = 1;
      }
    }
    tw::Outputs o = robot.tick(t);
    std::printf("%d %d %.5f %.5f %d %.4f %.4f %.4f %.4f %.4f %.4f %d\n", robot.state, robot.reason, o.wheel[0], o.wheel[1],
                o.servo_pwm_on ? 1 : 0, o.q[0], o.q[1], o.q[2], o.q[3], o.q[4], o.q[5], accepted);
    std::fflush(stdout);   // MSVCRT treats _IOLBF as full buffering on pipes
  }
  return 0;
}
