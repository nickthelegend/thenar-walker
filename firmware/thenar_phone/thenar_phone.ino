// Thenar Walker — phone + gamepad control (ESP32-S3 + PCA9685 arm + Cytron MDD10A wheels).
// The robot is a Wi-Fi access point "<Team>_TF" (WPA2 password from team_config.h, one phone at a time) and serves
// the control page at http://192.168.4.1 — joystick for the wheels, a slider per arm joint. Two gamepad routes:
//   A) gamepad paired to the PHONE: the page reads it (browser Gamepad API) and sends the same commands over Wi-Fi.
//      Works with any controller the phone accepts. Builds with the normal esp32:esp32 core.
//   B) gamepad paired to the ROBOT: Bluepad32. Build with the esp32-bluepad32 core (FQBN esp32-bluepad32:esp32:esp32s3).
//      The S3 radio is BLE only: Xbox Series / BLE-mode pads work, Bluetooth-Classic-only pads need route A.
//      The first pad that connects is remembered (NVS "tw-pad"); any other pad is refused until "Forget gamepad".
// Both routes use the same layout: left stick drive, right stick pan/shoulder, d-pad elbow/roll, Y/A wrist,
// L2/R2 gripper open/close, L1/R1 speed, B stop, X precision arm, hold START 1 s = arm on, hold SELECT 1 s = arm limp.
// Arm angles use the calibration saved by docs/calibrator ("Save to robot", NVS "tw-armcal"); a joint that was not
// marked done there cannot be switched on, and every angle is clamped to that joint's calibrated safe ends.
// Safety: every joint starts limp; switching a joint on makes it go to its target at once (support the arm).
//         Phone silent 500 ms / gamepad disconnected -> its drive input drops to zero and the arm freezes where it is,
//         PWM kept on so the gripper holds. STOP latches the wheels until every stick is back in the centre.
//         The KCD4 kill switch still cuts all power.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <ThenarLink.h>
#include <tw_armcal.h>
#if __has_include(<Bluepad32.h>)
#include <Bluepad32.h>
#define TW_PAD 1
#else
#define TW_PAD 0
#endif

#ifdef TW_EXAMPLE_CREDENTIALS
#warning "Example credentials in use: run  python firmware/tools/new_key.py \"Thenar Walker\"  first"
#endif

// ----------------------------------------------------------------- pins (ESP32-S3 board)
int sdaPin = 8, sclPin = 9, oePin = 10;          // overridden by the calibrator's PINS command (NVS "tw-armpins")
constexpr int PWM_PIN[2] = {4, 6};               // MDD10A PWM1 (left side), PWM2 (right side)
constexpr int DIR_PIN[2] = {5, 7};               // MDD10A DIR1, DIR2
constexpr int SIDE_SIGN[2] = {1, -1};            // right-side motors face the other way
constexpr int BATT_PIN = 1, LED_PIN = 2;
constexpr float BATT_DIVIDER = (100.0f + 27.0f) / 27.0f;   // 100k / 27k divider
constexpr int PWM_HZ = 20000, PWM_BITS = 10;
constexpr uint8_t PCA = 0x40;
constexpr float PCA_CLOCK_HZ = 25000000;         // same constant as the calibrator, so calibrated pulses match
constexpr uint32_t PHONE_TIMEOUT_MS = 500;
constexpr float ARM_DPS = 90;                    // arm slew limit per joint, deg/s
constexpr float PAD_DPS = 60, PAD_FINE_DPS = 20; // gamepad arm speed at full stick (normal / precision)
constexpr uint32_t HOLD_MS = 1000;               // START / SELECT hold time

WebServer server(80);
tw::ArmCal cal;
bool calOk = false, pcaOk = false, phone = false, stopLatch = false;
uint8_t prescale = 121;
int speed = 1;                                   // 0 slow, 1 mid, 2 fast (tw::SPEED_SCALE)
float lo[6], hi[6], goal[6], cur[6];
bool on[6];
float wheelT[2] = {0, 0}, wheel[2] = {0, 0};
float phoneThr = 0, phoneTrn = 0, padThr = 0, padTrn = 0;
uint32_t lastCmd = 0, lastTick = 0;
char ssid[32];

// ----------------------------------------------------------------- hardware
bool reg(uint8_t r, uint8_t v) { Wire.beginTransmission(PCA); Wire.write(r); Wire.write(v); return Wire.endTransmission() == 0; }

void chanOff(int ch) {   // full-off bit: no pulse, the servo goes limp
  Wire.beginTransmission(PCA); Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
  Wire.endTransmission();
}

void servoWrite(int ch, float q) {
  float us = tw::armcal_pulse(cal, ch, q);
  if (!tw::armcal_pulse_ok(cal, ch, us)) { chanOff(ch); on[ch] = false; return; }   // never outside the safe ends
  uint16_t count = lroundf(us * PCA_CLOCK_HZ / (1e6f * (prescale + 1)));
  Wire.beginTransmission(PCA); Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0);
  Wire.write(count & 0xFF); Wire.write((count >> 8) & 0x0F);
  Wire.endTransmission();
}

void updateOe() {
  bool any = false;
  for (int i = 0; i < 6; i++) any |= on[i];
  digitalWrite(oePin, any ? LOW : HIGH);
}

void pwmSetup(int k) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWM_PIN[k], PWM_HZ, PWM_BITS);
#else
  ledcSetup(k, PWM_HZ, PWM_BITS); ledcAttachPin(PWM_PIN[k], k);   // Bluepad32 core is arduino-esp32 2.x
#endif
}

void pwmWrite(int k, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWM_PIN[k], duty);
#else
  ledcWrite(k, duty);
#endif
}

void motorWrite(int side, float v) {   // side 0 = left, 1 = right; v in -1..1
  v = tw::clampf(v * SIDE_SIGN[side], -1, 1);
  digitalWrite(DIR_PIN[side], v < 0 ? HIGH : LOW);
  pwmWrite(side, lroundf(fabsf(v) * ((1 << PWM_BITS) - 1)));
}

float batteryV() { return analogReadMilliVolts(BATT_PIN) * BATT_DIVIDER / 1000.0f; }

// ----------------------------------------------------------------- control
void setDrive(float thr, float trn) {   // same mixing as tw::Robot (turn > 0 = left)
  thr = tw::clampf(thr, -1, 1); trn = tw::clampf(trn, -1, 1);
  if (fabsf(thr) < tw::DRIVE_DEADBAND) thr = 0;
  if (fabsf(trn) < tw::DRIVE_DEADBAND) trn = 0;
  if (thr == 0 && trn != 0) {          // spin in place: breakaway floor + turn scale
    float mag = tw::SPIN_MIN + (tw::TURN_SCALE[speed] - tw::SPIN_MIN) * fabsf(trn);
    wheelT[0] = trn > 0 ? -mag : mag;
    wheelT[1] = -wheelT[0];
  } else {
    float l = thr * tw::SPEED_SCALE[speed] - trn * tw::TURN_SCALE[speed], r = thr * tw::SPEED_SCALE[speed] + trn * tw::TURN_SCALE[speed];
    float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(r)));
    wheelT[0] = l / m; wheelT[1] = r / m;
  }
}

void freezeArm() { for (int i = 0; i < 6; i++) goal[i] = cur[i]; }

void stopAll() {   // STOP: wheels stop now and stay stopped until every stick is centred; arm holds where it is
  wheelT[0] = wheelT[1] = wheel[0] = wheel[1] = 0;
  stopLatch = true;
  freezeArm();
}

bool jointUsable(int i) { return calOk && ((cal.done_mask >> i) & 1) && lo[i] < hi[i]; }

void setJoint(int i, bool want) {
  want = want && pcaOk && jointUsable(i);
  if (want && !on[i]) { cur[i] = goal[i]; on[i] = true; servoWrite(i, cur[i]); }   // first pulse: jumps to the target
  else if (!want && on[i]) { on[i] = false; chanOff(i); }
}

void setMask(int m) {
  for (int i = 0; i < 6; i++) setJoint(i, (m >> i) & 1);
  updateOe();
}

int onMask() {
  int m = 0;
  for (int i = 0; i < 6; i++) m |= on[i] << i;
  return m;
}

int usableMask() {
  int m = 0;
  for (int i = 0; i < 6; i++) m |= jointUsable(i) << i;
  return m;
}

// ----------------------------------------------------------------- gamepad on the robot (route B, Bluepad32)
#if TW_PAD
ControllerPtr pad = nullptr;
uint8_t padAddr[6];
bool padKnown = false, padFine = false;
uint16_t padPrevBtn = 0;
uint32_t startDown = 0, selectDown = 0;

void padSaveAddr() {
  Preferences p; p.begin("tw-pad", false);
  if (padKnown) p.putBytes("addr", padAddr, 6); else p.remove("addr");
  p.end();
}

void rumble(ControllerPtr c, uint16_t ms) { c->playDualRumble(0, ms, 0x60, 0x60); }

void onPadConnected(ControllerPtr c) {
  ControllerProperties p = c->getProperties();
  if (pad || (padKnown && memcmp(p.btaddr, padAddr, 6))) {   // one pad only, and only the remembered one
    Serial.printf("gamepad %02X:%02X:%02X:%02X:%02X:%02X refused\n", p.btaddr[0], p.btaddr[1], p.btaddr[2], p.btaddr[3], p.btaddr[4], p.btaddr[5]);
    c->disconnect();
    return;
  }
  if (!padKnown) { memcpy(padAddr, p.btaddr, 6); padKnown = true; padSaveAddr(); }
  pad = c; padPrevBtn = 0; startDown = selectDown = 0;
  Serial.printf("gamepad connected: %s\n", c->getModelName().c_str());
  rumble(c, 200);
}

void onPadDisconnected(ControllerPtr c) {
  if (c != pad) return;
  pad = nullptr; padThr = padTrn = 0; freezeArm();
  Serial.println("gamepad lost -> its drive input zero, arm holds");
}

float stick(int v) {   // Bluepad32 axes are -512..511; 12 % dead zone
  float f = tw::clampf(v / 511.0f, -1, 1);
  return fabsf(f) < 0.12f ? 0 : (f - (f > 0 ? 0.12f : -0.12f)) / 0.88f;
}

void padTick(uint32_t now, float dt) {
  if (!pad || !pad->isConnected() || !pad->isGamepad()) { padThr = padTrn = 0; return; }
  uint16_t b = pad->buttons(), misc = pad->miscButtons();
  uint8_t dp = pad->dpad();
  uint16_t pressed = b & ~padPrevBtn;
  padPrevBtn = b;
  padThr = -stick(pad->axisY());               // stick up = forward
  padTrn = -stick(pad->axisX());               // stick left = turn left
  if (pressed & BUTTON_B) { stopAll(); rumble(pad, 120); }
  if (pressed & BUTTON_SHOULDER_R) speed = speed < 2 ? speed + 1 : 2;
  if (pressed & BUTTON_SHOULDER_L) speed = speed > 0 ? speed - 1 : 0;
  if (pressed & BUTTON_X) padFine = !padFine;
  // hold START 1 s: every calibrated joint on; hold SELECT 1 s: arm limp
  bool st = misc & MISC_BUTTON_START, se = misc & MISC_BUTTON_SELECT;
  if (st && !startDown) startDown = now;
  if (!st) startDown = 0;
  if (st && startDown && uint32_t(now - startDown) >= HOLD_MS) { setMask(usableMask()); startDown = 0; rumble(pad, 300); }
  if (se && !selectDown) selectDown = now;
  if (!se) selectDown = 0;
  if (se && selectDown && uint32_t(now - selectDown) >= HOLD_MS) { setMask(0); selectDown = 0; rumble(pad, 300); }
  float v[6] = {
      stick(pad->axisRX()),                                                  // pan: right = + (clockwise from above)
      -stick(pad->axisRY()),                                                 // shoulder: up = + (lean forward)
      float(((dp & DPAD_DOWN) ? 1 : 0) - ((dp & DPAD_UP) ? 1 : 0)),        // elbow: down = + (bend down)
      float(((b & BUTTON_A) ? 1 : 0) - ((b & BUTTON_Y) ? 1 : 0)),          // wrist flex: A = + (down)
      float(((dp & DPAD_RIGHT) ? 1 : 0) - ((dp & DPAD_LEFT) ? 1 : 0)),     // roll: right = + (clockwise)
      (pad->brake() - pad->throttle()) / 1023.0f};                           // gripper: L2 opens, R2 closes
  float k = (padFine ? PAD_FINE_DPS : PAD_DPS) * dt;
  for (int i = 0; i < 6; i++)
    if (on[i] && v[i] != 0) goal[i] = tw::clampf(goal[i] + v[i] * k, lo[i], hi[i]);
}
#endif

// ----------------------------------------------------------------- web (route A and the touch controls)
extern const char PAGE[];

void addArr(String &s, const char *key, const float *a) {
  s += ",\""; s += key; s += "\":[";
  for (int i = 0; i < 6; i++) { if (i) s += ','; s += String(a[i], 1); }
  s += "]";
}

void sendJson(bool full) {
  String s = "{\"bat\":" + String(batteryV(), 2) + ",\"on\":" + String(onMask()) + ",\"sp\":" + String(speed);
  addArr(s, "q", cur);
  addArr(s, "g", goal);
#if TW_PAD
  s += ",\"pad\":\"" + (pad ? String(pad->getModelName().c_str()) : String("")) + "\",\"padmem\":" + String(padKnown ? 1 : 0);
#endif
  if (full) {
    s += ",\"ssid\":\"" + String(ssid) + "\",\"cal\":" + String(calOk ? 1 : 0) + ",\"pca\":" + String(pcaOk ? 1 : 0) +
         ",\"bt\":" + String(TW_PAD) + ",\"usable\":" + String(usableMask());
    addArr(s, "lo", lo);
    addArr(s, "hi", hi);
  }
  s += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// /c?d=thr,turn [&s=speed] [&m=on_mask] [&q=q0,...,q5]   or   /c?stop=1      (thr, turn in -100..100, turn > 0 = left)
// q and m are only sent when the phone changed them, so the robot's gamepad can move the arm at the same time.
void handleCmd() {
  lastCmd = millis();
  phone = true;
  if (server.hasArg("stop")) { phoneThr = phoneTrn = 0; stopAll(); sendJson(false); return; }
  int thr = 0, trn = 0;
  if (sscanf(server.arg("d").c_str(), "%d,%d", &thr, &trn) == 2) { phoneThr = thr / 100.0f; phoneTrn = trn / 100.0f; }
  else phoneThr = phoneTrn = 0;
  if (server.hasArg("s")) speed = constrain(server.arg("s").toInt(), 0, 2);
  float q[6];
  if (server.hasArg("q") && sscanf(server.arg("q").c_str(), "%f,%f,%f,%f,%f,%f", &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]) == 6)
    for (int i = 0; i < 6; i++)
      if (isfinite(q[i])) goal[i] = tw::clampf(q[i], lo[i], hi[i]);
  if (server.hasArg("m")) setMask(server.arg("m").toInt());
  sendJson(false);
}

void setupWeb() {
  server.on("/", [] { server.send_P(200, "text/html", PAGE); });
  server.on("/cfg", [] { sendJson(true); });
  server.on("/c", handleCmd);
#if TW_PAD
  server.on("/forgetpad", [] {   // the next pad that connects becomes the remembered one
    padKnown = false; padSaveAddr();
    if (pad) pad->disconnect();
    BP32.forgetBluetoothKeys();
    sendJson(false);
  });
#endif
  server.onNotFound([] { server.sendHeader("Location", "/"); server.send(302, "text/plain", ""); });
  server.begin();
}

// ----------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  Preferences p;
  if (p.begin("tw-armpins", true)) { sdaPin = p.getInt("sda", sdaPin); sclPin = p.getInt("scl", sclPin); oePin = p.getInt("oe", oePin); p.end(); }
  pinMode(oePin, OUTPUT); digitalWrite(oePin, HIGH);   // servo outputs off until a joint is switched on
  pinMode(LED_PIN, OUTPUT);
  for (int k = 0; k < 2; k++) {
    pinMode(DIR_PIN[k], OUTPUT); digitalWrite(DIR_PIN[k], LOW);
    pwmSetup(k); pwmWrite(k, 0);
  }
  analogSetPinAttenuation(BATT_PIN, ADC_11db);

  calOk = tw::armcal_load(cal);
  if (!calOk) tw::armcal_defaults(cal);
  for (int i = 0; i < 6; i++) {   // angle range = the calibrated safe ends
    float a = (cal.min_us[i] - cal.zero_us[i]) / (cal.us_per_deg[i] * cal.sign[i]);
    float b = (cal.max_us[i] - cal.zero_us[i]) / (cal.us_per_deg[i] * cal.sign[i]);
    lo[i] = fminf(a, b); hi[i] = fmaxf(a, b);
    goal[i] = cur[i] = tw::clampf(0, lo[i], hi[i]);
    on[i] = false;
  }

  Wire.begin(sdaPin, sclPin); Wire.setClock(100000); Wire.setTimeOut(20);
  prescale = uint8_t(lroundf(PCA_CLOCK_HZ / (4096.0f * 50)) - 1);
  pcaOk = reg(0, 0x10) && reg(0xFE, prescale) && reg(1, 4) && reg(0, 0x20);
  delay(2);
  pcaOk = pcaOk && reg(0xFC, 0) && reg(0xFD, 0x10);    // ALL_LED_OFF: every channel limp

  tw::device_name(team::TEAM_NAME, ssid, sizeof ssid);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, team::WIFI_PASSWORD, 6, 0, 1);    // channel 6, visible, one phone
  setupWeb();
#if TW_PAD
  if (p.begin("tw-pad", true)) { padKnown = p.getBytes("addr", padAddr, 6) == 6; p.end(); }
  BP32.setup(&onPadConnected, &onPadDisconnected);
  BP32.enableVirtualDevice(false);                    // no mouse/touchpad devices, gamepads only
#endif
  Serial.printf("THENAR REMOTE  AP %s  http://%s  PCA9685 %s  arm calibration %s (done mask %02X)  robot gamepad %s\n", ssid,
                WiFi.softAPIP().toString().c_str(), pcaOk ? "ok" : "MISSING", calOk ? "ok" : "MISSING", calOk ? cal.done_mask : 0,
                TW_PAD ? "ON (BLE)" : "off (normal core build)");
}

void loop() {
  server.handleClient();
#if TW_PAD
  BP32.update();
#endif
  uint32_t now = millis();
  if (uint32_t(now - lastTick) < 20) { delay(1); return; }   // 50 Hz; delay lets the radio tasks run
  float dt = (now - lastTick) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;
  lastTick = now;
  if (phone && uint32_t(now - lastCmd) > PHONE_TIMEOUT_MS) {
    phone = false; phoneThr = phoneTrn = 0;
#if TW_PAD
    if (!pad) freezeArm();                   // a connected robot gamepad keeps control of the arm
#else
    freezeArm();
#endif
    Serial.println("phone lost -> its drive input zero, arm holds");
  }
#if TW_PAD
  padTick(now, dt);
#endif

  // the gamepad stick wins while it is pushed, otherwise the phone; STOP holds until both are centred
  float thr = phoneThr, trn = phoneTrn;
  if (padThr != 0 || padTrn != 0) { thr = padThr; trn = padTrn; }
  bool centred = fabsf(thr) < tw::DRIVE_DEADBAND && fabsf(trn) < tw::DRIVE_DEADBAND;
  if (stopLatch && centred) stopLatch = false;
  if (stopLatch) thr = trn = 0;
  setDrive(thr, trn);

  float a = tw::DRIVE_ACCEL_PER_S * dt;
  for (int k = 0; k < 2; k++) {
    float t = wheelT[k], w = wheel[k];
    if (t * w < 0) w = 0;                    // reversing: stop first
    else if (fabsf(t) <= fabsf(w)) w = t;    // slowing / stopping: immediate
    else w += tw::clampf(t - w, -a, a);      // speeding up: ramped
    wheel[k] = w;
    motorWrite(k, w);
  }
  float step = ARM_DPS * dt;
  for (int i = 0; i < 6; i++) {
    if (!on[i] || cur[i] == goal[i]) continue;
    cur[i] += tw::clampf(goal[i] - cur[i], -step, step);
    servoWrite(i, cur[i]);
  }
  updateOe();
#if TW_PAD
  bool linked = phone || pad;
#else
  bool linked = phone;
#endif
  digitalWrite(LED_PIN, linked ? HIGH : ((now / 250) & 1));
}

// ----------------------------------------------------------------- the phone page
const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Thenar Walker</title>
<style>
:root{--bg:#0f141a;--card:#18202a;--line:#2a3542;--fg:#e8edf2;--dim:#93a3b5;--acc:#f08a3c;--ok:#3ccf7a;--bad:#ff5a5a}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,-apple-system,Segoe UI,sans-serif;user-select:none;-webkit-user-select:none}
header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:8px;padding:10px 14px;background:#0b0f14ee;border-bottom:1px solid var(--line)}
header b{font-size:17px;letter-spacing:.5px;flex:1}
.pill{font-size:12px;padding:3px 9px;border-radius:99px;background:var(--card);color:var(--dim);white-space:nowrap}
.pill.ok{color:#08130c;background:var(--ok)}.pill.bad{color:#fff;background:var(--bad)}
#stop{width:100%;padding:16px;font-size:20px;font-weight:800;border:0;border-radius:12px;background:var(--bad);color:#fff}
main{padding:12px 14px 40px;display:flex;flex-direction:column;gap:12px;max-width:640px;margin:auto}
section{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:12px}
h2{margin:0 0 10px;font-size:13px;text-transform:uppercase;letter-spacing:1px;color:var(--dim)}
#pad{position:relative;width:min(78vw,260px);aspect-ratio:1;margin:6px auto;border-radius:50%;background:radial-gradient(#202a36,#141b23);border:2px solid var(--line);touch-action:none}
#knob{position:absolute;left:50%;top:50%;width:34%;aspect-ratio:1;margin:-17% 0 0 -17%;border-radius:50%;background:var(--acc);box-shadow:0 4px 16px #0008;pointer-events:none}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}
button{font:inherit;color:var(--fg);background:#232d39;border:1px solid var(--line);border-radius:9px;padding:9px 12px;min-width:44px}
button:active{filter:brightness(1.4)}
button.sel{background:var(--acc);color:#1a0d02;border-color:var(--acc);font-weight:700}
button:disabled{opacity:.35}
.seg{display:flex;flex:1}.seg button{flex:1;border-radius:0}.seg button:first-child{border-radius:9px 0 0 9px}.seg button:last-child{border-radius:0 9px 9px 0}
.j{border-top:1px solid var(--line);padding:10px 0 4px}
.j:first-of-type{border-top:0}
.jh{display:flex;align-items:center;gap:8px}
.jh .n{flex:1}.jh .n small{color:var(--dim)}
.jh .v{font:600 18px ui-monospace,Consolas,monospace;min-width:70px;text-align:right}
.tog{min-width:62px}.tog.on{background:var(--ok);color:#08130c;border-color:var(--ok);font-weight:700}
input[type=range]{width:100%;height:34px;accent-color:var(--acc);margin:6px 0}
.steps{display:grid;grid-template-columns:repeat(4,1fr);gap:6px}
.note{color:var(--dim);font-size:12.5px;line-height:1.45;margin:8px 0 0}
.warn{color:#ffb36b}
table{width:100%;border-collapse:collapse;font-size:13px}td{padding:4px 2px;border-top:1px solid var(--line)}td:first-child{color:var(--acc);font-weight:600;white-space:nowrap;padding-right:10px}
</style></head><body>
<header><b>THENAR WALKER</b><span id="gp" class="pill">no pad</span><span id="link" class="pill">connecting</span><span id="bat" class="pill">-- V</span></header>
<main>
<button id="stop">STOP</button>
<section><h2>Drive</h2>
<div id="pad"><div id="knob"></div></div>
<div class="row"><span style="color:var(--dim)">Speed</span><div class="seg" id="speed"><button data-s="0">Slow</button><button data-s="1" class="sel">Mid</button><button data-s="2">Fast</button></div></div>
<p class="note">Drag the orange knob. Let go = wheels stop. Losing Wi-Fi for 0.5 s also stops the wheels.</p>
</section>
<section><h2>Arm</h2>
<div class="row" style="margin-bottom:6px"><button id="zero">All to 0°</button><button id="allon">All joints ON</button><button id="limp">Limp all</button></div>
<div class="row" style="margin-bottom:6px"><span style="color:var(--dim)">Gripper</span><button id="gopen">Open</button><button id="gclose">Close</button></div>
<div id="joints"></div>
<p class="note warn" id="calnote"></p>
<p class="note">A joint is limp until you switch it ON — then it goes straight to its slider angle, so hold the arm near there first. Sliders stop at the safe ends you set in the calibrator.</p>
</section>
<section><h2>Gamepad</h2>
<p class="note" id="gpnote" style="margin-top:0"></p>
<table>
<tr><td>Left stick</td><td>drive (up = forward, sideways = turn)</td></tr>
<tr><td>Right stick</td><td>J1 base pan (left/right), J2 shoulder (up/down)</td></tr>
<tr><td>D-pad ↑ ↓</td><td>J3 elbow</td></tr>
<tr><td>D-pad ← →</td><td>J5 wrist roll</td></tr>
<tr><td>Y / A</td><td>J4 wrist flex up / down</td></tr>
<tr><td>L2 / R2</td><td>J6 gripper open / close</td></tr>
<tr><td>L1 / R1</td><td>speed down / up</td></tr>
<tr><td>B</td><td>STOP</td></tr>
<tr><td>X</td><td>precision arm (slow) on/off</td></tr>
<tr><td>hold START 1 s</td><td>arm ON (all calibrated joints — support the arm)</td></tr>
<tr><td>hold SELECT 1 s</td><td>arm limp</td></tr>
</table>
<div class="row" style="margin-top:10px" id="btrow" hidden><button id="forget">Forget robot gamepad</button></div>
</section>
</main>
<script>
const NAMES=[["J1","Base pan"],["J2","Shoulder lift"],["J3","Elbow"],["J4","Wrist flex"],["J5","Wrist roll"],["J6","Gripper"]];
const PAD_DPS=60,PAD_FINE_DPS=20,HOLD_MS=1000;
let cfg=null,q=[0,0,0,0,0,0],mask=0,sp=1,jx=0,jy=0,stopReq=false,busy=false,fails=0;
let qEdit=0,mEdit=0,sEdit=0,qSent=0,mSent=0,sSent=0,touching=-1;
const $=id=>document.getElementById(id);
function clamp(v,a,b){return Math.min(b,Math.max(a,v))}
function editQ(){qEdit++;show()}
function build(){
  const box=$("joints");box.innerHTML="";
  NAMES.forEach(([j,n],i)=>{
    const ok=(cfg.usable>>i)&1,d=document.createElement("div");d.className="j";
    d.innerHTML=`<div class="jh"><span class="n"><b>${j}</b> ${n}<br><small>${ok?cfg.lo[i].toFixed(0)+"° … "+cfg.hi[i].toFixed(0)+"°":"not calibrated"}</small></span>
<span class="v" id="v${i}">--</span><button class="tog" id="t${i}" ${ok?"":"disabled"}>OFF</button></div>
<input type="range" id="r${i}" min="${cfg.lo[i]}" max="${cfg.hi[i]}" step="0.5" value="${q[i]}" ${ok?"":"disabled"}>
<div class="steps">${[-5,-1,1,5].map(s=>`<button data-j="${i}" data-s="${s}" ${ok?"":"disabled"}>${s>0?"+":""}${s}°</button>`).join("")}</div>`;
    box.appendChild(d);
    const r=$("r"+i);
    r.oninput=e=>{q[i]=+e.target.value;editQ()};
    r.onpointerdown=()=>touching=i;r.onpointerup=r.onpointercancel=()=>touching=-1;
    $("t"+i).onclick=()=>{mask^=1<<i;mEdit++;show()};
  });
  box.querySelectorAll(".steps button").forEach(b=>b.onclick=()=>{const i=+b.dataset.j;q[i]=clamp(q[i]+ +b.dataset.s,cfg.lo[i],cfg.hi[i]);editQ()});
  $("calnote").textContent=!cfg.pca?"PCA9685 not found — check SDA/SCL wiring and servo board power.":
    !cfg.cal?"No arm calibration on the robot: open the calibrator on the laptop and press Save to robot, then reboot.":
    (cfg.usable!=63?"Joints marked not calibrated stay off — finish them in the calibrator and Save to robot.":"");
  $("btrow").hidden=!cfg.bt;
}
function show(){
  for(let i=0;i<6;i++){const r=$("r"+i);if(!r)continue;if(touching!==i)r.value=q[i];
    $("v"+i).textContent=q[i].toFixed(1)+"°";const t=$("t"+i),on=(mask>>i)&1;t.textContent=on?"ON":"OFF";t.classList.toggle("on",!!on)}
  document.querySelectorAll("#speed button").forEach(x=>x.classList.toggle("sel",+x.dataset.s===sp));
}
// ---- gamepad paired to the phone (browser Gamepad API, needs a secure context)
let gpFine=false,gpPrev=[],gpHold={9:0,8:0},gpLast=performance.now(),gpThr=0,gpTrn=0,gpName="";
function dz(v){return Math.abs(v)<0.12?0:(v-Math.sign(v)*0.12)/0.88}
function readGamepad(){
  const now=performance.now(),dt=Math.min(0.2,(now-gpLast)/1000);gpLast=now;gpThr=gpTrn=0;
  const g=[...(navigator.getGamepads?navigator.getGamepads():[])].find(p=>p&&p.connected);
  gpName=g?g.id:"";if(!g||!cfg)return;
  const B=i=>g.buttons[i]?g.buttons[i].value:0,P=i=>B(i)>0.5,edge=i=>P(i)&&!gpPrev[i];
  gpThr=-dz(g.axes[1]||0);gpTrn=-dz(g.axes[0]||0);
  if(edge(1)){stopReq=true;setKnob(0,0)}
  if(edge(5)&&sp<2){sp++;sEdit++}
  if(edge(4)&&sp>0){sp--;sEdit++}
  if(edge(2))gpFine=!gpFine;
  for(const b of[9,8]){if(P(b)){if(!gpHold[b])gpHold[b]=now;else if(now-gpHold[b]>=HOLD_MS){mask=b===9?cfg.usable:0;mEdit++;gpHold[b]=Infinity}}else gpHold[b]=0}
  const v=[dz(g.axes[2]||0),-dz(g.axes[3]||0),B(13)-B(12),B(0)-B(3),B(15)-B(14),B(6)-B(7)];
  const k=(gpFine?PAD_FINE_DPS:PAD_DPS)*dt;let moved=false;
  for(let i=0;i<6;i++)if(((mask>>i)&1)&&v[i]){q[i]=clamp(q[i]+v[i]*k,cfg.lo[i],cfg.hi[i]);moved=true}
  if(moved)editQ();
  gpPrev=g.buttons.map(b=>b.pressed);
}
async function tick(){
  readGamepad();
  if(busy||!cfg)return;busy=true;
  let thr=jy,trn=-jx;if(gpThr||gpTrn){thr=gpThr;trn=gpTrn}
  let url;const sq=qEdit,sm=mEdit,ss=sEdit;
  if(stopReq){url="/c?stop=1";stopReq=false}
  else{url=`/c?d=${Math.round(thr*100)},${Math.round(trn*100)}`;
    if(ss!==sSent)url+=`&s=${sp}`;if(sm!==mSent)url+=`&m=${mask}`;if(sq!==qSent)url+=`&q=${q.map(v=>v.toFixed(1)).join(",")}`}
  const ac=new AbortController(),to=setTimeout(()=>ac.abort(),450);
  try{const r=await(await fetch(url,{signal:ac.signal,cache:"no-store"})).json();fails=0;qSent=sq;mSent=sm;sSent=ss;
    $("link").textContent="connected";$("link").className="pill ok";
    $("bat").textContent=r.bat>1?r.bat.toFixed(1)+" V":"-- V";
    // the robot is the truth (its gamepad may be moving things too) unless the phone changed something meanwhile
    if(qEdit===sq&&touching<0)q=r.g.slice();
    if(mEdit===sm)mask=r.on;
    if(sEdit===ss)sp=r.sp;
    const rp=r.pad||"",pp=gpName?"phone: "+gpName.split("(")[0].trim().slice(0,18):"";
    $("gp").textContent=rp?"robot pad":(pp?"phone pad":"no pad");$("gp").className="pill"+(rp||pp?" ok":"");
    $("gpnote").textContent=(rp?"Gamepad on the robot: "+rp+". ":"")+(pp?"Gamepad on this phone: "+gpName+". ":"")+
      (!rp&&!pp?(window.isSecureContext?"Pair a controller with this phone (Bluetooth) and press any button"+(cfg.bt?", or pair it straight to the robot (BLE pads).":"."):
       "To use a controller paired to this phone, Chrome needs chrome://flags → “Insecure origins treated as secure” → http://192.168.4.1 → Relaunch."+(cfg.bt?" Or pair a BLE pad straight to the robot.":"")):"");
    show();
  }catch(e){if(++fails>2){$("link").textContent="NO LINK";$("link").className="pill bad"}}
  clearTimeout(to);busy=false;
}
async function load(){
  try{cfg=await(await fetch("/cfg",{cache:"no-store"})).json();q=cfg.g.slice();mask=cfg.on;sp=cfg.sp;build();show()}
  catch(e){setTimeout(load,1000)}
}
// touch joystick
const pad=$("pad"),knob=$("knob");let pid=null;
function setKnob(x,y){jx=x;jy=y;knob.style.transform=`translate(${x*120}%,${-y*120}%)`}
function padMove(e){const b=pad.getBoundingClientRect(),r=b.width/2;let x=(e.clientX-b.left-r)/(r*0.8),y=-(e.clientY-b.top-r)/(r*0.8);
  const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}setKnob(x,y)}
pad.addEventListener("pointerdown",e=>{pid=e.pointerId;try{pad.setPointerCapture(pid)}catch(_){}padMove(e)});
pad.addEventListener("pointermove",e=>{if(e.pointerId===pid)padMove(e)});
["pointerup","pointercancel","lostpointercapture"].forEach(t=>pad.addEventListener(t,e=>{if(e.pointerId===pid){pid=null;setKnob(0,0)}}));
document.querySelectorAll("#speed button").forEach(b=>b.onclick=()=>{sp=+b.dataset.s;sEdit++;show()});
$("stop").onclick=()=>{setKnob(0,0);stopReq=true};
$("zero").onclick=()=>{if(!cfg)return;for(let i=0;i<6;i++)q[i]=clamp(0,cfg.lo[i],cfg.hi[i]);editQ()};
$("allon").onclick=()=>{if(cfg){mask=cfg.usable;mEdit++;show()}};
$("limp").onclick=()=>{mask=0;mEdit++;show()};
$("gopen").onclick=()=>{if(cfg){q[5]=cfg.hi[5];editQ()}};
$("gclose").onclick=()=>{if(cfg){q[5]=cfg.lo[5];editQ()}};
$("forget").onclick=async()=>{if(confirm("Forget the robot's gamepad? The next one that connects will be accepted."))await fetch("/forgetpad",{cache:"no-store"})};
document.addEventListener("visibilitychange",()=>{if(document.hidden)setKnob(0,0)});
load();setInterval(tick,100);
</script></body></html>)HTML";
