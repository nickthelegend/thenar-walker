// Thenar Walker — arm calibration bridge (ESP32-S3 or classic ESP32 + PCA9685).
// Used by the browser calibrator (docs/calibrator/index.html, Web Serial, 115200). Line protocol, one reply line per command:
//   HELLO | SCAN | PINS sda scl oe | P ch us | REL ch | OFF | RATE us_per_s | STATE
//   CAL ch zero us_per_deg sign min max done | CALSAVE | CALGET | CALCLEAR
// Safety: outputs start limp (OE high, every channel full-off); a channel only gets a pulse after "P"; moves between
// commanded pulses are slew-limited (default 300 us/s); pulses are clamped to 500..2500 us. The very first pulse a servo
// receives makes it go there at full speed (a hobby servo has no position feedback) — the calibrator starts each joint
// from its saved zero or 1500 and warns first.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <tw_armcal.h>

#if CONFIG_IDF_TARGET_ESP32S3
constexpr int DEF_SDA = 8, DEF_SCL = 9, DEF_OE = 10;
#define CHIP_NAME "esp32s3"
#else
constexpr int DEF_SDA = 21, DEF_SCL = 22, DEF_OE = 13;
#define CHIP_NAME "esp32"
#endif
constexpr uint8_t PCA = 0x40;
constexpr float PCA_CLOCK_HZ = 25000000;    // same constant as the robot firmware, so calibrated pulses match
constexpr uint32_t TICK_MS = 20;            // 50 Hz slew update

int sdaPin = DEF_SDA, sclPin = DEF_SCL, oePin = DEF_OE;
uint8_t prescale = 121;
bool pcaOk = false, outputsOn = false;
float cur[6], tgt[6];
bool active[6];
float rate = 300;                          // us per second
uint32_t lastTick = 0;
tw::ArmCal cal;
char line[96];
size_t used = 0;
bool overflow = false;

bool reg(uint8_t r, uint8_t v) { Wire.beginTransmission(PCA); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0; }

bool chanOff(int ch) {   // full-off bit: the servo gets no pulse and goes limp
  Wire.beginTransmission(PCA); Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
  return Wire.endTransmission() == 0;
}

bool chanPulse(int ch, float us) {
  uint16_t count = lroundf(us * PCA_CLOCK_HZ / (1e6f * (prescale + 1)));
  Wire.beginTransmission(PCA); Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0);
  Wire.write(count & 0xFF); Wire.write((count >> 8) & 0x0F);
  return Wire.endTransmission() == 0;
}

void allOff() {
  digitalWrite(oePin, HIGH);
  outputsOn = false;
  for (int i = 0; i < 6; i++) { active[i] = false; if (pcaOk) chanOff(i); }
}

bool pcaInit() {
  prescale = uint8_t(lroundf(PCA_CLOCK_HZ / (4096.0f * 50)) - 1);
  bool ok = reg(0, 0x10) && reg(0xFE, prescale) && reg(1, 4) && reg(0, 0x20);
  delay(2);
  ok = ok && reg(0xFC, 0) && reg(0xFD, 0x10);    // ALL_LED_OFF: every channel full-off until commanded
  return ok;
}

void busInit() {
  pinMode(oePin, OUTPUT);
  digitalWrite(oePin, HIGH);
  Wire.end();
  Wire.begin(sdaPin, sclPin);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  pcaOk = pcaInit();
  allOff();
}

void reply(const char *s) { Serial.println(s); }

void cmdState() {
  Serial.print("OK STATE oe="); Serial.print(outputsOn ? 1 : 0);
  for (int i = 0; i < 6; i++) { Serial.print(' '); if (active[i]) Serial.print(lroundf(cur[i])); else Serial.print('-'); }
  Serial.println();
}

void command() {
  for (char *p = line; *p; p++) *p = toupper(*p);
  int a, b, c, d;
  float z, s, mn, mx;
  if (!strcmp(line, "HELLO")) {
    Serial.printf("OK HELLO tw-armcal 1 sda=%d scl=%d oe=%d pca=%d chip=%s\n", sdaPin, sclPin, oePin, pcaOk ? 1 : 0, CHIP_NAME);
  } else if (!strcmp(line, "SCAN")) {
    Serial.print("OK SCAN");
    for (uint8_t addr = 1; addr < 127; addr++) { Wire.beginTransmission(addr); if (Wire.endTransmission() == 0) Serial.printf(" %02X", addr); }
    Serial.println();
  } else if (sscanf(line, "PINS %d %d %d", &a, &b, &c) == 3) {
    if (a < 0 || b < 0 || c < 0 || a > 48 || b > 48 || c > 48 || a == b || a == c || b == c) { reply("ERR PINS bad"); return; }
    sdaPin = a; sclPin = b; oePin = c;
    Preferences p; p.begin("tw-armpins", false); p.putInt("sda", a); p.putInt("scl", b); p.putInt("oe", c); p.end();
    busInit();
    Serial.printf("OK PINS sda=%d scl=%d oe=%d pca=%d\n", sdaPin, sclPin, oePin, pcaOk ? 1 : 0);
  } else if (sscanf(line, "P %d %d", &a, &b) == 2) {
    if (!pcaOk) { reply("ERR P no_pca9685"); return; }
    if (a < 0 || a > 5) { reply("ERR P channel"); return; }
    float us = constrain(b, 500, 2500);
    tgt[a] = us;
    if (!active[a]) { cur[a] = us; active[a] = true; chanPulse(a, us); }   // first pulse: the servo jumps here
    if (!outputsOn) { digitalWrite(oePin, LOW); outputsOn = true; }
    Serial.printf("OK P %d %d\n", a, int(us));
  } else if (sscanf(line, "REL %d", &a) == 1) {
    if (a < 0 || a > 5) { reply("ERR REL channel"); return; }
    active[a] = false; if (pcaOk) chanOff(a);
    Serial.printf("OK REL %d\n", a);
  } else if (!strcmp(line, "OFF")) {
    allOff(); reply("OK OFF");
  } else if (sscanf(line, "RATE %d", &a) == 1) {
    rate = constrain(a, 20, 2000); Serial.printf("OK RATE %d\n", int(rate));
  } else if (!strcmp(line, "STATE")) {
    cmdState();
  } else if (sscanf(line, "CAL %d %f %f %d %f %f %d", &a, &z, &s, &b, &mn, &mx, &d) == 7) {
    if (a < 0 || a > 5) { reply("ERR CAL channel"); return; }
    tw::ArmCal t = cal;     // check on a copy: invalid numbers are refused, so the stored record always stays loadable
    t.zero_us[a] = z; t.us_per_deg[a] = s; t.sign[a] = b >= 0 ? 1 : -1; t.min_us[a] = mn; t.max_us[a] = mx;
    if (!tw::armcal_joint_ok(t, a)) { Serial.printf("ERR CAL %d bad_values\n", a); return; }
    cal = t;
    if (d) cal.done_mask |= 1 << a; else cal.done_mask &= ~(1 << a);
    Serial.printf("OK CAL %d done=%d\n", a, d ? 1 : 0);
  } else if (!strcmp(line, "CALSAVE")) {
    Serial.printf(tw::armcal_save(cal) ? "OK CALSAVE mask=%02X\n" : "ERR CALSAVE nvs mask=%02X\n", cal.done_mask);
  } else if (!strcmp(line, "CALGET")) {
    for (int i = 0; i < 6; i++)
      Serial.printf("CAL %d %.1f %.5f %d %.1f %.1f %d\n", i, cal.zero_us[i], cal.us_per_deg[i], cal.sign[i], cal.min_us[i], cal.max_us[i],
                    (cal.done_mask >> i) & 1);
    Serial.printf("OK CALGET mask=%02X complete=%d\n", cal.done_mask, tw::armcal_complete(cal) ? 1 : 0);
  } else if (!strcmp(line, "CALCLEAR")) {
    tw::armcal_defaults(cal); tw::armcal_save(cal); reply("OK CALCLEAR");
  } else {
    reply("ERR unknown (HELLO SCAN PINS P REL OFF RATE STATE CAL CALSAVE CALGET CALCLEAR)");
  }
}

void setup() {
  Serial.begin(115200);
  Preferences p;
  if (p.begin("tw-armpins", true)) { sdaPin = p.getInt("sda", DEF_SDA); sclPin = p.getInt("scl", DEF_SCL); oePin = p.getInt("oe", DEF_OE); p.end(); }
  for (int i = 0; i < 6; i++) { cur[i] = tgt[i] = 1500; active[i] = false; }
  if (!tw::armcal_load(cal)) tw::armcal_defaults(cal);
  busInit();
  Serial.printf("READY tw-armcal sda=%d scl=%d oe=%d pca=%d\n", sdaPin, sclPin, oePin, pcaOk ? 1 : 0);
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      line[used] = 0;
      if (overflow) { allOff(); reply("ERR line_overflow -> OFF"); }
      else if (used) command();
      used = 0; overflow = false;
    } else if (used < sizeof line - 1) line[used++] = ch;
    else overflow = true;
  }
  uint32_t now = millis();
  if (uint32_t(now - lastTick) < TICK_MS) return;
  float step = rate * (now - lastTick) / 1000.0f;
  lastTick = now;
  for (int i = 0; i < 6; i++) {
    if (!active[i] || cur[i] == tgt[i]) continue;
    float d = tgt[i] - cur[i];
    cur[i] = fabsf(d) <= step ? tgt[i] : cur[i] + (d > 0 ? step : -step);
    chanPulse(i, cur[i]);
  }
}
