// Thenar Walker — phone control, low latency (ESP32-S3 + PCA9685 arm + Cytron MDD10A wheels).
// The robot is a Wi-Fi access point "<Team>_TF" (WPA2 password from team_config.h, one phone at a time).
// The phone opens http://192.168.4.1 ; the page then talks over ONE WebSocket (port 81) that stays open:
// every touch is sent the moment it happens (no polling), Nagle is off, Wi-Fi power save is off, the loop runs at 100 Hz.
//
// Phone -> robot (one text frame each):  D thr turn  (-100..100, turn > 0 = left)   S speed(0..2)   X (STOP)
//   A 1|0 (arm on/limp)   M mask   Q joint deg (target)   V joint deg_per_s (hold buttons, 0 = stop)   P n (ping)   G (config)
// Robot -> phone:  C{json config}   T bat on speed latch q0..q5   P n (pong)
//
// Safety: the phone sends at least every 50 ms. Nothing for 300 ms -> wheels stop, hold buttons stop, the arm stays
// where it is with PWM on (the gripper keeps its grip). STOP latches the wheels until the stick is centred.
// Every joint starts limp; switching a joint on makes it go to its target at once. Angles are clamped to the
// calibrated safe ends; joints not marked done in the calibrator can't be switched on. The KCD4 kill switch cuts power.
// Calibration: the browser calibrator (docs/calibrator) works over USB with this same firmware (serial commands below).
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <SHA1Builder.h>
#include <base64.h>
#include <esp_wifi.h>
#include <ThenarLink.h>
#include <tw_armcal.h>

#ifdef TW_EXAMPLE_CREDENTIALS
#warning "Example credentials in use: run  python firmware/tools/new_key.py \"Thenar Walker\"  first"
#endif

// ----------------------------------------------------------------- pins (ESP32-S3 board)
int sdaPin = 8, sclPin = 9, oePin = 10;          // overridden by the calibrator's PINS setting (NVS "tw-armpins")
constexpr int PWM_PIN[2] = {4, 6};               // MDD10A PWM1 (left side), PWM2 (right side)
constexpr int DIR_PIN[2] = {5, 7};               // MDD10A DIR1, DIR2
constexpr int SIDE_SIGN[2] = {1, -1};            // right-side motors face the other way
constexpr int BATT_PIN = 1, LED_PIN = 2;
constexpr float BATT_DIVIDER = (100.0f + 27.0f) / 27.0f;   // 100k / 27k divider
constexpr int PWM_HZ = 20000, PWM_BITS = 10;
constexpr uint8_t PCA = 0x40;
constexpr float PCA_CLOCK_HZ = 25000000;         // same constant as the calibrator, so calibrated pulses match

constexpr uint32_t LINK_TIMEOUT_MS = 300;
constexpr uint32_t TICK_MS = 10;                 // 100 Hz control loop
constexpr uint32_t TELE_MS = 100;                // status to the phone
constexpr float ARM_DPS = 120;                   // slew limit toward a slider target
constexpr float VEL_MAX = 90;                    // hold-button speed cap

WebServer http(80);
WiFiServer wsServer(81);
WiFiClient ws;
bool wsOpen = false;
uint8_t rx[192];
size_t rxLen = 0;

tw::ArmCal cal;
bool calOk = false, pcaOk = false, linkUp = false, stopLatch = false;
uint8_t prescale = 121;
int speed = 1;                                   // 0 slow, 1 mid, 2 fast (tw::SPEED_SCALE)
float lo[6], hi[6], goal[6], cur[6], vel[6];
bool on[6], raw[6];                              // raw: driven by the calibrator in plain microseconds
float rawCur[6], rawTgt[6], rawRate = 300;
float thr = 0, trn = 0, wheelT[2] = {0, 0}, wheel[2] = {0, 0};
uint32_t lastMsg = 0, lastTick = 0, lastTele = 0;
char ssid[32];

// ----------------------------------------------------------------- hardware
bool reg(uint8_t r, uint8_t v) { Wire.beginTransmission(PCA); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0; }

void chanOff(int ch) {   // full-off bit: no pulse, the servo goes limp
  Wire.beginTransmission(PCA); Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
  Wire.endTransmission();
}

void chanPulse(int ch, float us) {
  uint16_t count = lroundf(us * PCA_CLOCK_HZ / (1e6f * (prescale + 1)));
  Wire.beginTransmission(PCA); Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0);
  Wire.write(count & 0xFF); Wire.write((count >> 8) & 0x0F);
  Wire.endTransmission();
}

void servoWrite(int ch, float q) {
  float us = tw::armcal_pulse(cal, ch, q);
  if (!tw::armcal_pulse_ok(cal, ch, us)) { chanOff(ch); on[ch] = false; return; }   // never outside the safe ends
  chanPulse(ch, us);
}

void updateOe() {
  bool any = false;
  for (int i = 0; i < 6; i++) any |= on[i] || raw[i];
  digitalWrite(oePin, any ? LOW : HIGH);
}

void motorWrite(int side, float v) {   // side 0 = left, 1 = right; v in -1..1
  v = tw::clampf(v * SIDE_SIGN[side], -1, 1);
  digitalWrite(DIR_PIN[side], v < 0 ? HIGH : LOW);
  ledcWrite(PWM_PIN[side], lroundf(fabsf(v) * ((1 << PWM_BITS) - 1)));
}

float batteryV() { return analogReadMilliVolts(BATT_PIN) * BATT_DIVIDER / 1000.0f; }

// ----------------------------------------------------------------- control
void setDrive(float t, float r) {   // same mixing as tw::Robot (turn > 0 = left)
  t = tw::clampf(t, -1, 1); r = tw::clampf(r, -1, 1);
  if (fabsf(t) < tw::DRIVE_DEADBAND) t = 0;
  if (fabsf(r) < tw::DRIVE_DEADBAND) r = 0;
  if (t == 0 && r != 0) {          // spin in place: breakaway floor + turn scale
    float mag = tw::SPIN_MIN + (tw::TURN_SCALE[speed] - tw::SPIN_MIN) * fabsf(r);
    wheelT[0] = r > 0 ? -mag : mag;
    wheelT[1] = -wheelT[0];
  } else {
    float l = t * tw::SPEED_SCALE[speed] - r * tw::TURN_SCALE[speed], rr = t * tw::SPEED_SCALE[speed] + r * tw::TURN_SCALE[speed];
    float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(rr)));
    wheelT[0] = l / m; wheelT[1] = rr / m;
  }
}

void freezeArm() { for (int i = 0; i < 6; i++) { goal[i] = cur[i]; vel[i] = 0; } }

void stopAll() {   // STOP: wheels stop now and stay stopped until the stick is centred; arm holds where it is
  wheelT[0] = wheelT[1] = wheel[0] = wheel[1] = 0;
  stopLatch = true;
  freezeArm();
}

void applyCal() {   // angle range of every joint = its calibrated safe ends
  for (int i = 0; i < 6; i++) {
    float a = (cal.min_us[i] - cal.zero_us[i]) / (cal.us_per_deg[i] * cal.sign[i]);
    float b = (cal.max_us[i] - cal.zero_us[i]) / (cal.us_per_deg[i] * cal.sign[i]);
    lo[i] = fminf(a, b); hi[i] = fmaxf(a, b);
    goal[i] = cur[i] = tw::clampf(0, lo[i], hi[i]);
    vel[i] = 0;
  }
}

bool jointUsable(int i) { return calOk && ((cal.done_mask >> i) & 1) && lo[i] < hi[i]; }

int usableMask() {
  int m = 0;
  for (int i = 0; i < 6; i++) m |= jointUsable(i) << i;
  return m;
}

int onMask() {
  int m = 0;
  for (int i = 0; i < 6; i++) m |= on[i] << i;
  return m;
}

void setJoint(int i, bool want) {
  want = want && pcaOk && jointUsable(i);
  if (want && !on[i]) { raw[i] = false; vel[i] = 0; cur[i] = goal[i]; on[i] = true; servoWrite(i, cur[i]); }   // jumps to the target
  else if (!want && on[i]) { on[i] = false; vel[i] = 0; chanOff(i); }
}

void setMask(int m) {
  for (int i = 0; i < 6; i++) setJoint(i, (m >> i) & 1);
  updateOe();
}

void allLimp() {
  for (int i = 0; i < 6; i++) { on[i] = raw[i] = false; vel[i] = 0; if (pcaOk) chanOff(i); }
  updateOe();
}

// ----------------------------------------------------------------- WebSocket (RFC 6455, one phone, small text frames)
void wsClose() {
  if (ws) ws.stop();
  wsOpen = false;
  rxLen = 0;
}

void wsSend(const char *s, size_t n) {
  if (!wsOpen) return;
  uint8_t buf[700];
  size_t h = 2;
  if (n + 4 > sizeof buf) return;
  buf[0] = 0x81;                                   // FIN + text
  if (n < 126) buf[1] = n;
  else { buf[1] = 126; buf[2] = n >> 8; buf[3] = n & 0xFF; h = 4; }
  memcpy(buf + h, s, n);
  if (ws.write(buf, h + n) != h + n) wsClose();   // one write per frame -> one TCP segment, sent at once (no Nagle)
}

void sendCfg() {
  char b[640];
  int n = snprintf(b, sizeof b, "C{\"ssid\":\"%s\",\"cal\":%d,\"pca\":%d,\"usable\":%d,\"on\":%d,\"sp\":%d,\"lo\":[", ssid,
                   calOk ? 1 : 0, pcaOk ? 1 : 0, usableMask(), onMask(), speed);
  for (int i = 0; i < 6; i++) n += snprintf(b + n, sizeof b - n, "%s%.1f", i ? "," : "", lo[i]);
  n += snprintf(b + n, sizeof b - n, "],\"hi\":[");
  for (int i = 0; i < 6; i++) n += snprintf(b + n, sizeof b - n, "%s%.1f", i ? "," : "", hi[i]);
  n += snprintf(b + n, sizeof b - n, "],\"g\":[");
  for (int i = 0; i < 6; i++) n += snprintf(b + n, sizeof b - n, "%s%.1f", i ? "," : "", goal[i]);
  n += snprintf(b + n, sizeof b - n, "]}");
  wsSend(b, n);
}

void sendTele() {
  char b[160];
  int n = snprintf(b, sizeof b, "T %.2f %d %d %d", batteryV(), onMask(), speed, stopLatch ? 1 : 0);
  for (int i = 0; i < 6; i++) n += snprintf(b + n, sizeof b - n, " %.1f", cur[i]);
  wsSend(b, n);
}

// finds "name:" (case-insensitive) in an HTTP header block and copies its value
bool headerValue(const char *req, const char *name, char *out, size_t cap) {
  size_t nl = strlen(name);
  for (const char *p = req; *p; p++) {
    if ((p == req || p[-1] == '\n') && !strncasecmp(p, name, nl) && p[nl] == ':') {
      p += nl + 1;
      while (*p == ' ') p++;
      size_t k = 0;
      while (*p && *p != '\r' && *p != '\n' && k + 1 < cap) out[k++] = *p++;
      out[k] = 0;
      return k > 0;
    }
  }
  return false;
}

void wsAccept() {
  WiFiClient c = wsServer.accept();
  if (!c) return;
  char req[1024];
  size_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 400 && n < sizeof req - 1) {   // the upgrade request arrives in one go
    int a = c.available();
    if (a > 0) {
      int r = c.read((uint8_t *)req + n, min((size_t)a, sizeof req - 1 - n));
      if (r > 0) n += r;
      req[n] = 0;
      if (strstr(req, "\r\n\r\n")) break;
    } else {
      delay(1);
    }
  }
  req[n] = 0;
  char key[64];
  if (!headerValue(req, "Sec-WebSocket-Key", key, sizeof key)) { c.stop(); return; }
  String k = String(key) + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  SHA1Builder sha;
  sha.begin();
  sha.add((const uint8_t *)k.c_str(), k.length());
  sha.calculate();
  uint8_t digest[20];
  sha.getBytes(digest);
  String resp = "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: " +
                base64::encode(digest, 20) + "\r\n\r\n";
  if (wsOpen) wsClose();                           // the newest connection wins (the AP allows one phone anyway)
  ws = c;
  ws.setNoDelay(true);
  ws.write((const uint8_t *)resp.c_str(), resp.length());
  wsOpen = true;
  rxLen = 0;
  lastMsg = millis();
  linkUp = true;
  Serial.println("phone connected");
  sendCfg();
}

void handleCmd(char *c) {
  int i, v;
  float f;
  switch (c[0]) {
    case 'D': {
      int a = 0, b = 0;
      if (sscanf(c + 1, "%d %d", &a, &b) == 2) { thr = a / 100.0f; trn = b / 100.0f; }
      break;
    }
    case 'S': if (sscanf(c + 1, "%d", &v) == 1) speed = constrain(v, 0, 2); break;
    case 'X': stopAll(); break;
    case 'A': if (sscanf(c + 1, "%d", &v) == 1) setMask(v ? usableMask() : 0); break;
    case 'M': if (sscanf(c + 1, "%d", &v) == 1) setMask(v); break;
    case 'Q':
      if (sscanf(c + 1, "%d %f", &i, &f) == 2 && i >= 0 && i < 6 && isfinite(f)) { goal[i] = tw::clampf(f, lo[i], hi[i]); vel[i] = 0; }
      break;
    case 'V':
      if (sscanf(c + 1, "%d %f", &i, &f) == 2 && i >= 0 && i < 6 && isfinite(f)) vel[i] = tw::clampf(f, -VEL_MAX, VEL_MAX);
      break;
    case 'P': wsSend(c, strlen(c)); break;         // ping: echo straight back so the phone can show the round trip
    case 'G': sendCfg(); break;
  }
}

void wsPoll() {
  if (!wsOpen) return;
  if (!ws.connected()) { Serial.println("phone disconnected"); wsClose(); return; }
  int a;
  while ((a = ws.available()) > 0 && rxLen < sizeof rx) {
    int r = ws.read(rx + rxLen, min((size_t)a, sizeof rx - rxLen));
    if (r <= 0) break;
    rxLen += r;
  }
  while (rxLen >= 2) {                             // every complete frame in the buffer
    uint8_t op = rx[0] & 0x0F;
    size_t len = rx[1] & 0x7F, h = 2;
    if (!(rx[1] & 0x80) || len > 125) { wsClose(); return; }   // browsers always mask; our messages are short
    if (rxLen < h + 4 + len) return;
    uint8_t *mask = rx + h, *p = rx + h + 4;
    for (size_t j = 0; j < len; j++) p[j] ^= mask[j & 3];
    if (op == 1) {
      char cmd[128];
      memcpy(cmd, p, len);
      cmd[len] = 0;
      lastMsg = millis();
      if (!linkUp) { linkUp = true; Serial.println("phone link back"); }
      handleCmd(cmd);
    } else if (op == 8) {
      wsClose();
      return;
    } else if (op == 9) {                          // ping -> pong with the same payload
      uint8_t pong[130] = {0x8A, (uint8_t)len};
      memcpy(pong + 2, p, len);
      ws.write(pong, len + 2);
    }
    size_t used = h + 4 + len;
    memmove(rx, rx + used, rxLen - used);
    rxLen -= used;
  }
}

// ----------------------------------------------------------------- serial: the browser calibrator (docs/calibrator)
char sline[96];
size_t sused = 0;

void serialCommand() {
  for (char *c = sline; *c; c++) *c = toupper(*c);
  int a, b, d;
  float z, sl, mn, mx;
  if (!strcmp(sline, "HELLO")) {
    Serial.printf("OK HELLO tw-armcal 1 sda=%d scl=%d oe=%d pca=%d chip=esp32s3 app=thenar_phone\n", sdaPin, sclPin, oePin, pcaOk ? 1 : 0);
  } else if (sscanf(sline, "RATE %d", &a) == 1) {
    rawRate = constrain(a, 20, 2000);
    Serial.printf("OK RATE %d\n", int(rawRate));
  } else if (!strcmp(sline, "OFF")) {
    allLimp();
    Serial.println("OK OFF");
  } else if (sscanf(sline, "P %d %d", &a, &b) == 2) {
    if (!pcaOk) { Serial.println("ERR P no_pca9685"); return; }
    if (a < 0 || a > 5) { Serial.println("ERR P channel"); return; }
    float us = constrain(b, 500, 2500);
    on[a] = false;                                 // the phone lets go of this joint
    rawTgt[a] = us;
    if (!raw[a]) { raw[a] = true; rawCur[a] = us; chanPulse(a, us); }   // first pulse: the servo jumps there
    updateOe();
    Serial.printf("OK P %d %d\n", a, int(us));
  } else if (sscanf(sline, "REL %d", &a) == 1) {
    if (a < 0 || a > 5) { Serial.println("ERR REL channel"); return; }
    on[a] = raw[a] = false;
    if (pcaOk) chanOff(a);
    updateOe();
    Serial.printf("OK REL %d\n", a);
  } else if (!strcmp(sline, "STATE")) {
    Serial.printf("OK STATE oe=%d", digitalRead(oePin) == LOW ? 1 : 0);
    for (int i = 0; i < 6; i++) {
      if (raw[i]) Serial.printf(" %ld", lroundf(rawCur[i]));
      else Serial.print(" -");
    }
    Serial.println();
  } else if (sscanf(sline, "CAL %d %f %f %d %f %f %d", &a, &z, &sl, &b, &mn, &mx, &d) == 7) {
    if (a < 0 || a > 5) { Serial.println("ERR CAL channel"); return; }
    tw::ArmCal t = cal;   // checked on a copy: invalid numbers are refused, the stored record always stays loadable
    t.zero_us[a] = z; t.us_per_deg[a] = sl; t.sign[a] = b >= 0 ? 1 : -1; t.min_us[a] = mn; t.max_us[a] = mx;
    if (!tw::armcal_joint_ok(t, a)) { Serial.printf("ERR CAL %d bad_values\n", a); return; }
    cal = t;
    if (d) cal.done_mask |= 1 << a;
    else cal.done_mask &= ~(1 << a);
    Serial.printf("OK CAL %d done=%d\n", a, d ? 1 : 0);
  } else if (!strcmp(sline, "CALSAVE")) {
    bool ok = tw::armcal_save(cal);
    calOk = ok && tw::armcal_valid(cal);
    allLimp();
    applyCal();                                    // new numbers: every joint starts limp again
    Serial.printf(ok ? "OK CALSAVE mask=%02X\n" : "ERR CALSAVE nvs mask=%02X\n", cal.done_mask);
  } else if (!strcmp(sline, "CALGET")) {
    for (int i = 0; i < 6; i++)
      Serial.printf("CAL %d %.1f %.5f %d %.1f %.1f %d\n", i, cal.zero_us[i], cal.us_per_deg[i], cal.sign[i], cal.min_us[i], cal.max_us[i],
                    (cal.done_mask >> i) & 1);
    Serial.printf("OK CALGET mask=%02X complete=%d\n", cal.done_mask, tw::armcal_complete(cal) ? 1 : 0);
  } else {
    Serial.println("ERR unknown (HELLO RATE OFF P REL STATE CAL CALSAVE CALGET)");
  }
}

void serialPoll() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 13) continue;                        // CR
    if (ch == 10) {                                // LF ends a command
      sline[sused] = 0;
      if (sused) serialCommand();
      sused = 0;
    } else if (sused < sizeof sline - 1) {
      sline[sused++] = ch;
    }
  }
}

// ----------------------------------------------------------------- setup / loop
extern const char PAGE[];

void setup() {
  Serial.begin(115200);
  Preferences p;
  if (p.begin("tw-armpins", true)) { sdaPin = p.getInt("sda", sdaPin); sclPin = p.getInt("scl", sclPin); oePin = p.getInt("oe", oePin); p.end(); }
  pinMode(oePin, OUTPUT); digitalWrite(oePin, HIGH);   // servo outputs off until a joint is switched on
  pinMode(LED_PIN, OUTPUT);
  for (int k = 0; k < 2; k++) {
    pinMode(DIR_PIN[k], OUTPUT); digitalWrite(DIR_PIN[k], LOW);
    ledcAttach(PWM_PIN[k], PWM_HZ, PWM_BITS); ledcWrite(PWM_PIN[k], 0);
  }
  analogSetPinAttenuation(BATT_PIN, ADC_11db);

  calOk = tw::armcal_load(cal);
  if (!calOk) tw::armcal_defaults(cal);
  for (int i = 0; i < 6; i++) { on[i] = raw[i] = false; rawCur[i] = rawTgt[i] = 1500; }
  applyCal();

  Wire.begin(sdaPin, sclPin); Wire.setClock(400000); Wire.setTimeOut(20);
  prescale = uint8_t(lroundf(PCA_CLOCK_HZ / (4096.0f * 50)) - 1);
  pcaOk = reg(0, 0x10) && reg(0xFE, prescale) && reg(1, 4) && reg(0, 0x20);
  delay(2);
  pcaOk = pcaOk && reg(0xFC, 0) && reg(0xFD, 0x10);    // ALL_LED_OFF: every channel limp

  tw::device_name(team::TEAM_NAME, ssid, sizeof ssid);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, team::WIFI_PASSWORD, 6, 0, 1);    // channel 6, visible, one phone
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);                      // radio always on: no 100 ms power-save wake-ups
  http.on("/", [] { http.send_P(200, "text/html", PAGE); });
  http.onNotFound([] { http.sendHeader("Location", "/"); http.send(302, "text/plain", ""); });
  http.begin();
  wsServer.begin();
  wsServer.setNoDelay(true);
  Serial.printf("THENAR PHONE  AP %s  http://%s  PCA9685 %s  arm calibration %s (done mask %02X)\n", ssid,
                WiFi.softAPIP().toString().c_str(), pcaOk ? "ok" : "MISSING", calOk ? "ok" : "MISSING", calOk ? cal.done_mask : 0);
}

void loop() {
  http.handleClient();
  wsAccept();
  wsPoll();
  serialPoll();
  uint32_t now = millis();
  if (uint32_t(now - lastTick) < TICK_MS) return;
  float dt = (now - lastTick) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;
  lastTick = now;

  if (linkUp && uint32_t(now - lastMsg) > LINK_TIMEOUT_MS) {   // phone silent: stop wheels, hold the arm
    linkUp = false;
    thr = trn = 0;
    freezeArm();
    Serial.println("phone silent -> wheels stopped, arm holding");
  }

  bool centred = fabsf(thr) < tw::DRIVE_DEADBAND && fabsf(trn) < tw::DRIVE_DEADBAND;
  if (stopLatch && centred) stopLatch = false;
  setDrive(stopLatch ? 0 : thr, stopLatch ? 0 : trn);
  float a = tw::DRIVE_ACCEL_PER_S * dt;
  for (int k = 0; k < 2; k++) {
    float t = wheelT[k], w = wheel[k];
    if (t * w < 0) w = 0;                          // reversing: stop first
    else if (fabsf(t) <= fabsf(w)) w = t;          // slowing / stopping: immediate
    else w += tw::clampf(t - w, -a, a);            // speeding up: ramped
    wheel[k] = w;
    motorWrite(k, w);
  }

  float step = ARM_DPS * dt;
  for (int i = 0; i < 6; i++) {
    if (!on[i]) continue;
    if (vel[i] != 0) goal[i] = tw::clampf(goal[i] + vel[i] * dt, lo[i], hi[i]);
    if (cur[i] == goal[i]) continue;
    cur[i] += tw::clampf(goal[i] - cur[i], -step, step);
    servoWrite(i, cur[i]);
  }
  float rstep = rawRate * dt;
  for (int i = 0; i < 6; i++) {
    if (!raw[i] || rawCur[i] == rawTgt[i]) continue;
    rawCur[i] += tw::clampf(rawTgt[i] - rawCur[i], -rstep, rstep);
    chanPulse(i, rawCur[i]);
  }
  updateOe();
  digitalWrite(LED_PIN, linkUp ? HIGH : ((now / 250) & 1));
  if (wsOpen && uint32_t(now - lastTele) >= TELE_MS) { lastTele = now; sendTele(); }
}

// ----------------------------------------------------------------- the phone page
const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="theme-color" content="#0e1318">
<title>Thenar Walker</title>
<style>
:root{--bg:#0e1318;--card:#161d25;--card2:#1d2630;--line:#2b3744;--fg:#e9eef3;--dim:#8f9eae;--acc:#f2913d;--ok:#45d184;--bad:#ff5b52;--warn:#ffc15e}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;-webkit-touch-callout:none;-webkit-user-select:none;user-select:none}
html,body{height:100%;margin:0;background:var(--bg);color:var(--fg);font:14px/1.35 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif;overscroll-behavior:none}
body{display:flex;flex-direction:column;padding:env(safe-area-inset-top) max(10px,env(safe-area-inset-right)) env(safe-area-inset-bottom) max(10px,env(safe-area-inset-left))}
.bar{display:flex;align-items:center;gap:6px;flex-wrap:wrap;padding:8px 0}
.bar b{font-size:15px;letter-spacing:.06em;margin-right:auto}
.pill{font:600 12px ui-monospace,Consolas,monospace;padding:4px 9px;border-radius:99px;background:var(--card2);color:var(--dim);white-space:nowrap}
.pill.ok{background:var(--ok);color:#06140c}.pill.bad{background:var(--bad);color:#fff}.pill.warn{background:var(--warn);color:#1d1300}
button{font:600 14px system-ui,sans-serif;color:var(--fg);background:var(--card2);border:1px solid var(--line);border-radius:10px;padding:9px 12px;touch-action:manipulation}
button:active,button.held{background:var(--acc);color:#1b0d01;border-color:var(--acc)}
button.sel{background:var(--acc);color:#1b0d01;border-color:var(--acc)}
button:disabled{opacity:.35}
#stop{background:var(--bad);border-color:var(--bad);color:#fff;font-size:16px;padding:10px 18px;letter-spacing:.06em}
.main{flex:1;display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.25fr);gap:10px;min-height:0;padding-bottom:8px}
@media (orientation:portrait){.main{grid-template-columns:minmax(0,1fr)}}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:10px;min-width:0;display:flex;flex-direction:column;gap:8px}
.card h2{margin:0;font-size:11px;letter-spacing:.14em;color:var(--dim);text-transform:uppercase;display:flex;align-items:center;gap:6px}
.card h2 span{margin-left:auto}
#padWrap{flex:1;display:grid;place-items:center;min-height:220px}
#pad{position:relative;width:min(100%,62vh,330px);aspect-ratio:1;border-radius:50%;background:radial-gradient(circle,#1d2732 0 30%,#141b22 70%);border:2px solid var(--line);touch-action:none}
#pad::before,#pad::after{content:"";position:absolute;background:var(--line);left:50%;top:8%;bottom:8%;width:1px}
#pad::after{top:50%;left:8%;right:8%;height:1px;width:auto;bottom:auto}
#knob{position:absolute;left:50%;top:50%;width:34%;aspect-ratio:1;margin:-17% 0 0 -17%;border-radius:50%;background:var(--acc);box-shadow:0 6px 18px #0009;pointer-events:none;will-change:transform}
.seg{display:flex;gap:4px}.seg button{flex:1;padding:8px 4px}
.joints{display:flex;flex-direction:column;gap:6px;overflow-y:auto;min-height:0}
.j{display:grid;grid-template-columns:76px 52px minmax(0,1fr) 52px 56px;gap:6px;align-items:center}
.j .n{font-weight:700;font-size:13px;line-height:1.15}.j .n small{display:block;font-weight:400;color:var(--dim);font-size:11px}
.j .h{padding:10px 0;font-size:18px;touch-action:none}
.j .v{grid-column:3;text-align:center;font:600 15px ui-monospace,Consolas,monospace}
.j input{grid-column:3;width:100%;margin:0;accent-color:var(--acc);height:28px;touch-action:none}
.j .slot{display:grid;grid-template-rows:auto auto}
.j .t{padding:9px 0}.j .t.on{background:var(--ok);color:#06140c;border-color:var(--ok)}
.row{display:flex;gap:6px;flex-wrap:wrap;align-items:center}
.note{color:var(--dim);font-size:12px}
.warn{color:var(--warn)}
</style></head><body>
<div class="bar">
  <b>THENAR</b>
  <span id="link" class="pill bad">connecting</span>
  <span id="rtt" class="pill">-- ms</span>
  <span id="bat" class="pill">-- V</span>
  <button id="fs">Full screen</button>
  <button id="stop">STOP</button>
</div>
<div class="main">
  <div class="card">
    <h2>Drive <span id="latch" class="pill warn" hidden>STOPPED — centre the stick</span></h2>
    <div id="padWrap"><div id="pad"><div id="knob"></div></div></div>
    <div class="seg" id="speed"><button data-s="0">Slow</button><button data-s="1" class="sel">Mid</button><button data-s="2">Fast</button></div>
  </div>
  <div class="card">
    <h2>Arm <span class="row"><button id="armOn">Arm ON</button><button id="limp">Limp</button><button id="fine">Fine</button></span></h2>
    <div class="joints" id="joints"></div>
    <div class="note" id="calnote"></div>
  </div>
</div>
<script>
const NAMES=[["J1","Base pan"],["J2","Shoulder"],["J3","Elbow"],["J4","Wrist flex"],["J5","Wrist roll"],["J6","Gripper"]];
const $=id=>document.getElementById(id);
let ws=null,open=false,cfg=null,mask=0,sp=1,fine=false,thr=0,trn=0,lastD="",dirty=false,drag=-1,pingT=0;
const q=[0,0,0,0,0,0],qSend=[null,null,null,null,null,null];
function send(s){if(open&&ws.readyState===1)ws.send(s)}
function connect(){
  ws=new WebSocket("ws://"+(location.hostname||"192.168.4.1")+":81/");
  ws.onopen=()=>{open=true;$("link").textContent="linked";$("link").className="pill ok";send("G");sendDrive(true)};
  ws.onclose=()=>{open=false;$("link").textContent="NO LINK";$("link").className="pill bad";$("rtt").textContent="-- ms";setTimeout(connect,300)};
  ws.onerror=()=>{try{ws.close()}catch(e){}};
  ws.onmessage=e=>onMsg(e.data);
}
function onMsg(m){
  const k=m[0];
  if(k==="T"){const a=m.split(" ");const bat=+a[1];mask=+a[2];sp=+a[3];$("latch").hidden=a[4]!=="1";
    $("bat").textContent=bat>1?bat.toFixed(1)+" V":"-- V";for(let i=0;i<6;i++){q[i]=+a[5+i];}showJoints();showSpeed();}
  else if(k==="P"){const t=+m.slice(2);const r=performance.now()-t;$("rtt").textContent=r.toFixed(0)+" ms";$("rtt").className="pill"+(r<30?" ok":r<80?" warn":" bad")}
  else if(k==="C"){cfg=JSON.parse(m.slice(1));mask=cfg.on;sp=cfg.sp;for(let i=0;i<6;i++)q[i]=cfg.g[i];build();showSpeed()}
}
// ---- drive: sent the moment the stick moves (once per frame at most), plus a 40 ms heartbeat
function sendDrive(force){const d="D "+Math.round(thr*100)+" "+Math.round(trn*100);if(force||d!==lastD){send(d);lastD=d}}
function frame(){if(dirty){dirty=false;sendDrive(false)}
  for(let i=0;i<6;i++)if(qSend[i]!==null){send("Q "+i+" "+qSend[i].toFixed(1));qSend[i]=null}
  requestAnimationFrame(frame)}
requestAnimationFrame(frame);
setInterval(()=>{if(!document.hidden)sendDrive(true)},40);
setInterval(()=>{if(!document.hidden)send("P "+performance.now().toFixed(1))},500);
const pad=$("pad"),knob=$("knob");let pid=null;
function setStick(x,y){thr=y;trn=-x;knob.style.transform=`translate(${x*118}%,${-y*118}%)`;dirty=true}
function padMove(e){const b=pad.getBoundingClientRect(),r=b.width/2;let x=(e.clientX-b.left-r)/(r*.82),y=-(e.clientY-b.top-r)/(r*.82);
  const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}setStick(x,y)}
pad.addEventListener("pointerdown",e=>{pid=e.pointerId;try{pad.setPointerCapture(pid)}catch(_){}padMove(e)});
pad.addEventListener("pointermove",e=>{if(e.pointerId===pid)padMove(e)});
["pointerup","pointercancel","lostpointercapture"].forEach(t=>pad.addEventListener(t,e=>{if(e.pointerId===pid){pid=null;setStick(0,0);sendDrive(true)}}));
// ---- buttons
$("stop").onclick=()=>{setStick(0,0);send("X");sendDrive(true)};
document.querySelectorAll("#speed button").forEach(b=>b.onclick=()=>{sp=+b.dataset.s;send("S "+sp);showSpeed()});
$("armOn").onclick=()=>send("A 1");
$("limp").onclick=()=>send("A 0");
$("fine").onclick=()=>{fine=!fine;$("fine").classList.toggle("sel",fine)};
$("fs").onclick=async()=>{try{await document.documentElement.requestFullscreen();await screen.orientation.lock("landscape")}catch(e){}};
function showSpeed(){document.querySelectorAll("#speed button").forEach(b=>b.classList.toggle("sel",+b.dataset.s===sp))}
// ---- arm: hold buttons send a speed (robot moves the joint until release), sliders send a target
function build(){
  const box=$("joints");box.innerHTML="";
  NAMES.forEach(([id,n],i)=>{
    const ok=(cfg.usable>>i)&1,d=document.createElement("div");d.className="j";
    const minus=i===5?"close":"−",plus=i===5?"open":"+";
    d.innerHTML=`<div class="n">${id}<small>${n}</small></div><button class="h" data-i="${i}" data-d="-1" ${ok?"":"disabled"}>${minus}</button>
<div class="slot"><div class="v" id="v${i}">--</div><input type="range" id="r${i}" min="${cfg.lo[i]}" max="${cfg.hi[i]}" step="0.5" value="${q[i]}" ${ok?"":"disabled"}></div>
<button class="h" data-i="${i}" data-d="1" ${ok?"":"disabled"}>${plus}</button><button class="t" id="t${i}" ${ok?"":"disabled"}>OFF</button>`;
    box.appendChild(d);
    const r=$("r"+i);
    r.addEventListener("pointerdown",()=>drag=i);
    ["pointerup","pointercancel"].forEach(t=>r.addEventListener(t,()=>{drag=-1}));
    r.oninput=()=>{qSend[i]=+r.value;$("v"+i).textContent=(+r.value).toFixed(1)+"°"};
    $("t"+i).onclick=()=>{mask^=1<<i;send("M "+mask);showJoints()};
  });
  box.querySelectorAll(".h").forEach(b=>{
    const i=+b.dataset.i,dir=+b.dataset.d;let p=null;
    const stop=()=>{if(p===null)return;p=null;b.classList.remove("held");send("V "+i+" 0")};
    b.addEventListener("pointerdown",e=>{p=e.pointerId;try{b.setPointerCapture(p)}catch(_){}b.classList.add("held");send("V "+i+" "+dir*(fine?15:50))});
    ["pointerup","pointercancel","lostpointercapture"].forEach(t=>b.addEventListener(t,stop));
  });
  $("calnote").innerHTML=!cfg.pca?'<span class="warn">Servo board (PCA9685) not found: check SDA/SCL and its power.</span>':
    !cfg.cal?'<span class="warn">No arm calibration on the robot: run the calibrator over USB and Save to robot.</span>':
    "Arm ON: every joint jumps to its target, so hold the arm near the shown angles first. Hold − / + to move.";
  showJoints();
}
function showJoints(){if(!cfg)return;
  for(let i=0;i<6;i++){const r=$("r"+i);if(!r)continue;if(drag!==i){r.value=q[i];$("v"+i).textContent=q[i].toFixed(1)+"°"}
    const t=$("t"+i),o=(mask>>i)&1;t.textContent=o?"ON":"OFF";t.classList.toggle("on",!!o)}}
document.addEventListener("visibilitychange",()=>{if(document.hidden){setStick(0,0);sendDrive(true)}});
connect();
</script></body></html>)HTML";
