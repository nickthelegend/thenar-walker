#pragma once
// Arm calibration record shared by the calibration sketch (firmware/arm_calibration) and the robot firmware.
// pulse_us = zero_us + q_deg * us_per_deg * sign  (zero_us = pulse at LOGICAL q = 0, same as calibration.h).
// Stored in NVS (Preferences "tw-armcal", key "cal"); the robot uses it only when magic, version, crc and all 6 joints are good.
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace tw {

constexpr uint32_t ARMCAL_MAGIC = 0x43415754;   // "TWAC"
constexpr uint16_t ARMCAL_VERSION = 1;

struct ArmCal {
  uint32_t magic;
  uint16_t version;
  uint8_t done_mask;      // bit i = joint i calibrated
  uint8_t pad;
  float zero_us[6];
  float us_per_deg[6];
  int8_t sign[6];
  uint8_t pad2[2];
  float min_us[6];
  float max_us[6];
  uint32_t crc;
};

inline uint32_t crc32(const uint8_t *p, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    c ^= p[i];
    for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
  }
  return ~c;
}

inline uint32_t armcal_crc(const ArmCal &c) { return crc32((const uint8_t *)&c, offsetof(ArmCal, crc)); }

inline void armcal_defaults(ArmCal &c) {
  memset(&c, 0, sizeof c);
  c.magic = ARMCAL_MAGIC; c.version = ARMCAL_VERSION;
  for (int i = 0; i < 6; i++) { c.zero_us[i] = 1500; c.us_per_deg[i] = 5.555556f; c.sign[i] = 1; c.min_us[i] = 1000; c.max_us[i] = 2000; }
  c.crc = armcal_crc(c);
}

// one joint is usable when its numbers are physically sane: zero inside its limits, limits inside 500..2500, slope 3..20 us/deg
inline bool armcal_joint_ok(const ArmCal &c, int i) {
  return c.min_us[i] >= 500 && c.max_us[i] <= 2500 && c.min_us[i] < c.zero_us[i] && c.zero_us[i] < c.max_us[i] &&
         c.us_per_deg[i] >= 3.0f && c.us_per_deg[i] <= 20.0f && (c.sign[i] == 1 || c.sign[i] == -1);
}

inline bool armcal_valid(const ArmCal &c) {
  if (c.magic != ARMCAL_MAGIC || c.version != ARMCAL_VERSION || c.crc != armcal_crc(c)) return false;
  for (int i = 0; i < 6; i++) if (!armcal_joint_ok(c, i)) return false;
  return true;
}

inline bool armcal_complete(const ArmCal &c) { return armcal_valid(c) && (c.done_mask & 0x3F) == 0x3F; }

inline float armcal_pulse(const ArmCal &c, int i, float q_deg) { return c.zero_us[i] + q_deg * c.us_per_deg[i] * c.sign[i]; }

// a commanded angle is allowed only if its pulse stays inside the recorded safe limits
inline bool armcal_pulse_ok(const ArmCal &c, int i, float us) { return us >= c.min_us[i] && us <= c.max_us[i]; }

}  // namespace tw

#ifdef ARDUINO
#include <Preferences.h>
namespace tw {
inline bool armcal_load(ArmCal &c) {
  Preferences p;
  if (!p.begin("tw-armcal", true)) return false;
  bool ok = p.getBytesLength("cal") == sizeof c && p.getBytes("cal", &c, sizeof c) == sizeof c;
  p.end();
  return ok && armcal_valid(c);
}
inline bool armcal_save(ArmCal &c) {
  c.magic = ARMCAL_MAGIC; c.version = ARMCAL_VERSION; c.crc = armcal_crc(c);
  Preferences p;
  if (!p.begin("tw-armcal", false)) return false;
  bool ok = p.putBytes("cal", &c, sizeof c) == sizeof c;
  p.end();
  return ok;
}
}  // namespace tw
#endif
