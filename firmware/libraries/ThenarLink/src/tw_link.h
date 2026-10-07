// Thenar Walker radio link: authenticated UDP packets between the operator's controller
// (leader arm + joystick) and the robot. Header-only and platform independent.
//
// RoboReach rules covered here:
//  * SSID / device name "TeamName_TF" (spaces removed, team name cut to 20 characters)
//  * only the team's controller can drive the robot: every packet carries an HMAC-SHA256
//    (truncated to 128 bit) over a pre-shared 32-byte key; the robot ignores anything else
//  * replay protection: per-boot random session id from the robot + strictly increasing seq
#pragma once
#include <stdint.h>
#include <string.h>
#include "tw_sha256.h"

namespace tw {

constexpr uint16_t UDP_PORT = 4210;
constexpr size_t KEY_LEN = 32;
constexpr size_t MAC_LEN = 16;
constexpr uint32_t MAGIC_HELLO = 0x31485754;    // "TWH1"
constexpr uint32_t MAGIC_WELCOME = 0x31575754;  // "TWW1"
constexpr uint32_t MAGIC_CMD = 0x31435754;      // "TWC1"
constexpr uint32_t MAGIC_STATUS = 0x31535754;   // "TWS1"

enum Flags : uint8_t { F_ENABLE = 1, F_DRIVE = 2, F_ARM = 4 };

#pragma pack(push, 1)
struct Hello {    // controller -> robot (broadcast or unicast)
  uint32_t magic;
  uint32_t ctrl_nonce;
  uint8_t mac[MAC_LEN];
};
struct Welcome {  // robot -> controller
  uint32_t magic;
  uint32_t ctrl_nonce;
  uint32_t session;
  uint8_t mac[MAC_LEN];
};
struct Cmd {      // controller -> robot, 50 Hz
  uint32_t magic;
  uint32_t session;
  uint32_t seq;
  uint8_t flags;
  int8_t throttle;   // -100..100 (+ forward)
  int8_t turn;       // -100..100 (+ turn left / counter-clockwise)
  uint8_t speed;     // 0 precision, 1 normal, 2 fast
  int16_t q_cdeg[6]; // joint targets, centi-degrees (thenar-arms logical angles)
  uint8_t mac[MAC_LEN];
};
struct Status {   // robot -> controller, 10 Hz
  uint32_t magic;
  uint32_t session;
  uint32_t seq_ack;
  uint8_t state;
  uint8_t reason;
  uint16_t batt_mv;
  uint8_t mac[MAC_LEN];
};
#pragma pack(pop)

template <class P>
inline void sign(P &p, const uint8_t key[KEY_LEN]) {
  uint8_t full[32];
  hmac_sha256(key, KEY_LEN, &p, sizeof(P) - MAC_LEN, full);
  memcpy(p.mac, full, MAC_LEN);
}

template <class P>
inline bool verify(const P &p, const uint8_t key[KEY_LEN]) {
  uint8_t full[32];
  hmac_sha256(key, KEY_LEN, &p, sizeof(P) - MAC_LEN, full);
  return ct_equal(full, p.mac, MAC_LEN);
}

// Parse an incoming datagram; true only for a correctly sized, correctly signed packet.
template <class P>
inline bool parse(const uint8_t *buf, size_t n, uint32_t magic, const uint8_t key[KEY_LEN], P &out) {
  if (n != sizeof(P)) return false;
  memcpy(&out, buf, sizeof(P));
  if (out.magic != magic) return false;
  return verify(out, key);
}

// Robot side acceptance of commands: right session, newer than anything accepted so far.
struct CmdGate {
  uint32_t session = 0;
  uint32_t last_seq = 0;
  bool have = false;
  void new_session(uint32_t s) { session = s; last_seq = 0; have = false; }
  // returns true if the command may be acted on
  bool accept(const uint8_t *buf, size_t n, const uint8_t key[KEY_LEN], Cmd &out) {
    if (!parse(buf, n, MAGIC_CMD, key, out)) return false;
    if (out.session != session) return false;
    if (have && int32_t(out.seq - last_seq) <= 0) return false;  // replay / reordered
    last_seq = out.seq;
    have = true;
    return true;
  }
};

// "TeamName_TF": spaces removed, team name limited to its first 20 characters.
inline void device_name(const char *team, char *out, size_t cap) {
  size_t o = 0, kept = 0;
  for (const char *p = team; *p && kept < 20 && o + 1 < cap; ++p) {
    if (*p == ' ') continue;
    out[o++] = *p;
    kept++;
  }
  const char *suf = "_TF";
  for (const char *p = suf; *p && o + 1 < cap; ++p) out[o++] = *p;
  out[o] = 0;
}

}  // namespace tw
