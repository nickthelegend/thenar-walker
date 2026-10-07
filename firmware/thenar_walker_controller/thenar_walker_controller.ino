// Thenar Walker — operator controller = the thenar-arms L1 encoder LEADER + a 2-axis joystick.
// One operator: right hand on the leader arm (6 x AS5600 through a TCA9548A, unchanged from
// thenar-arms), left hand on the joystick (drive) with an ENABLE toggle and a SPEED button.
// Joins the robot's "<TeamName>_TF" access point and streams signed commands at 50 Hz.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Preferences.h>
#include <ThenarLink.h>

constexpr int SDA_PIN = 21, SCL_PIN = 22, LED_PIN = 2;
constexpr int JOY_X_PIN = 34, JOY_Y_PIN = 35;      // ADC1 only (ADC2 is unusable with Wi-Fi)
constexpr int ENABLE_PIN = 32, SPEED_PIN = 33;     // to GND, internal pull-ups
constexpr uint8_t MUX = 0x70, ENC = 0x36;
constexpr float HOME[6] = {0, -25, 35, 0, 0, 20};  // leader ZERO calibration pose (thenar-arms)

Preferences prefs;
uint16_t zeros[6] = {};
int8_t signs[6] = {1, 1, 1, 1, 1, 1};
bool calibrated = false;
WiFiUDP udp;
IPAddress robotIp(192, 168, 4, 1);
uint32_t session = 0, seq = 0, nonce = 0, lastSend = 0, lastHello = 0, lastStatus = 0;
uint8_t speedMode = 1, lastRobotState = 0;
int joyX0 = 2048, joyY0 = 2048;
bool speedPrev = true;
char ssid[32], line[64];
size_t used = 0;

int readReg(uint8_t a, uint8_t r) {
  Wire.beginTransmission(a); Wire.write(r);
  if (Wire.endTransmission(false)) return -1;
  if (Wire.requestFrom(a, uint8_t(1)) != 1) return -1;
  return Wire.read();
}

bool encoder(int ch, uint16_t &raw) {
  Wire.beginTransmission(MUX); Wire.write(uint8_t(1 << ch));
  if (Wire.endTransmission()) return false;
  int s = readReg(ENC, 0x0B);
  if (s < 0 || !(s & 0x20) || (s & 0x18)) return false;   // magnet missing / too weak / too strong
  Wire.beginTransmission(ENC); Wire.write(0x0C);
  if (Wire.endTransmission(false)) return false;
  if (Wire.requestFrom(ENC, uint8_t(2)) != 2) return false;
  raw = ((Wire.read() & 15) << 8) | Wire.read();
  return true;
}

float wrapped(uint16_t raw, uint16_t zero) { return (((int(raw) - int(zero) + 2048) & 4095) - 2048) * 360.f / 4096; }

int8_t axis(int pin, int center) {
  int v = analogRead(pin) - center;
  float f = tw::clampf(v / 1900.0f, -1, 1);
  return int8_t(lroundf(f * 100));
}

void serialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c != '\n') { if (used < sizeof line - 1) line[used++] = c; continue; }
    line[used] = 0; used = 0;
    uint16_t raw[6]; bool ok = true;
    if (!strcmp(line, "ZERO")) {   // leader held at thenar-arms HOME pose
      for (int i = 0; i < 6 && ok; i++) ok = encoder(i, raw[i]);
      if (ok) { memcpy(zeros, raw, sizeof zeros); prefs.putBytes("zeros", zeros, sizeof zeros); calibrated = true; Serial.println("ZERO saved"); }
      else Serial.println("FAULT encoder");
    } else if (!strncmp(line, "SIGN ", 5)) {
      int j, s; if (sscanf(line + 5, "%d %d", &j, &s) == 2 && j >= 0 && j < 6 && (s == 1 || s == -1)) { signs[j] = s; prefs.putBytes("signs", signs, sizeof signs); Serial.println("SIGN saved"); }
    } else if (!strcmp(line, "JOYZERO")) {
      joyX0 = analogRead(JOY_X_PIN); joyY0 = analogRead(JOY_Y_PIN); Serial.println("joystick centred");
    } else {
      Serial.println("COMMANDS: ZERO | SIGN axis +/-1 | JOYZERO");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(ENABLE_PIN, INPUT_PULLUP);
  pinMode(SPEED_PIN, INPUT_PULLUP);
  analogSetPinAttenuation(JOY_X_PIN, ADC_11db); analogSetPinAttenuation(JOY_Y_PIN, ADC_11db);
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(20);
  prefs.begin("thenar-L1", false);   // same namespace as the thenar-arms leader firmware
  calibrated = prefs.getBytesLength("zeros") == sizeof zeros;
  if (calibrated) prefs.getBytes("zeros", zeros, sizeof zeros);
  if (prefs.getBytesLength("signs") == sizeof signs) prefs.getBytes("signs", signs, sizeof signs);
  joyX0 = analogRead(JOY_X_PIN); joyY0 = analogRead(JOY_Y_PIN);   // stick released at power-up
  tw::device_name(team::TEAM_NAME, ssid, sizeof ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, team::WIFI_PASSWORD);
  udp.begin(tw::UDP_PORT);
  Serial.printf("THENAR WALKER CONTROLLER -> %s  leader %s\n", ssid, calibrated ? "calibrated" : "NOT CALIBRATED (send ZERO at HOME pose)");
}

void receive() {
  int n = udp.parsePacket();
  if (n <= 0) return;
  uint8_t buf[64];
  if (n > int(sizeof buf)) { udp.clear(); return; }
  udp.read(buf, n);
  tw::Welcome w; tw::Status s;
  if (tw::parse(buf, n, tw::MAGIC_WELCOME, team::LINK_KEY, w) && w.ctrl_nonce == nonce) {
    session = w.session; seq = 0; lastStatus = millis();
    Serial.printf("PAIRED session %08lx\n", (unsigned long)session);
  } else if (tw::parse(buf, n, tw::MAGIC_STATUS, team::LINK_KEY, s) && s.session == session) {
    lastStatus = millis(); lastRobotState = s.state;
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint > 1000) { lastPrint = millis(); Serial.printf("robot state %u reason %u batt %u mV\n", s.state, s.reason, s.batt_mv); }
  }
}

void loop() {
  serialCommands();
  receive();
  uint32_t now = millis();
  if (WiFi.status() != WL_CONNECTED) { session = 0; digitalWrite(LED_PIN, (now / 100) & 1); return; }
  if (!session || uint32_t(now - lastStatus) > 1000) {          // (re)pair
    if (uint32_t(now - lastHello) > 300) {
      lastHello = now; session = 0; nonce = esp_random();
      tw::Hello h{tw::MAGIC_HELLO, nonce, {0}};
      tw::sign(h, team::LINK_KEY);
      udp.beginPacket(robotIp, tw::UDP_PORT); udp.write((uint8_t *)&h, sizeof h); udp.endPacket();
    }
    return;
  }
  bool sp = digitalRead(SPEED_PIN);
  if (!sp && speedPrev) speedMode = (speedMode + 1) % 3;
  speedPrev = sp;
  if (uint32_t(now - lastSend) < 20) return;                     // 50 Hz
  lastSend = now;
  tw::Cmd c{};
  c.magic = tw::MAGIC_CMD; c.session = session; c.seq = ++seq; c.speed = speedMode;
  bool enable = digitalRead(ENABLE_PIN) == LOW;
  uint16_t raw[6]; bool ok = calibrated;
  float q[6];
  for (int i = 0; i < 6 && ok; i++) {
    ok = encoder(i, raw[i]);
    q[i] = HOME[i] + signs[i] * wrapped(raw[i], zeros[i]);
  }
  c.flags = (enable && ok) ? (tw::F_ENABLE | tw::F_DRIVE | tw::F_ARM) : 0;   // sensor fault => robot HOLDs
  c.throttle = axis(JOY_Y_PIN, joyY0);
  c.turn = int8_t(-axis(JOY_X_PIN, joyX0));
  for (int i = 0; i < 6; i++) c.q_cdeg[i] = ok ? int16_t(lroundf(tw::clampf(q[i], -300, 300) * 100)) : 0;
  tw::sign(c, team::LINK_KEY);
  udp.beginPacket(robotIp, tw::UDP_PORT); udp.write((uint8_t *)&c, sizeof c); udp.endPacket();
  digitalWrite(LED_PIN, lastRobotState == tw::ACTIVE);
}
