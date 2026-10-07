// Thenar Walker — robot ESP32 (on the bot).
// Wi-Fi access point "<TeamName>_TF" (WPA2, one station max), authenticated UDP commands,
// 4 x JGA25-370 12 V motors on 1 x Cytron MDD10A (2 per channel), 6 x MG996R arm servos on a PCA9685.
// Safety: see ThenarLink/tw_robot.h — link loss stops the wheels and freezes the arm with
// servo PWM still on (gripper keeps holding); the hardware kill switch cuts all power.
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <ThenarLink.h>
#include "calibration.h"

#ifdef TW_EXAMPLE_CREDENTIALS
#warning "Example credentials in use: copy ThenarLink/src/team_config.example.h to team_config.h"
#endif

// ----------------------------------------------------------------- pins (see docs/WIRING.md)
constexpr int SDA_PIN = 21, SCL_PIN = 22, PCA_OE_PIN = 13, LED_PIN = 2, BATT_PIN = 34;
// Cytron MDD10A Rev2.0, sign-magnitude: PWM pin = speed, DIR pin = direction; PWM low = brake.
// Channel 1 = left side (FL + RL wired in parallel), channel 2 = right side (FR + RR in parallel).
constexpr int PWM_PIN[2] = {25, 16};          // MDD10A PWM1, PWM2
constexpr int DIR_PIN[2] = {26, 17};          // MDD10A DIR1, DIR2
constexpr int SIDE_SIGN[2] = {1, -1};         // right-side motors face the other way
constexpr int PWM_HZ = 20000, PWM_BITS = 10;
constexpr float BATT_DIVIDER = (100.0f + 27.0f) / 27.0f;   // 100k / 27k divider to GPIO34
constexpr uint8_t PCA_ADDR = 0x40;

WiFiUDP udp;
tw::Robot robot;
tw::CmdGate gate;
IPAddress ctrlIp;
uint16_t ctrlPort = 0;
uint32_t lastTick = 0, lastStatus = 0;
uint8_t prescale = 0;
bool pcaOk = false;
char ssid[32];

bool pcaWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(PCA_ADDR); Wire.write(reg); Wire.write(v);
  return Wire.endTransmission() == 0;
}

bool servoWrite(int ch, float q) {
  float us = SERVO_ZERO_US[ch] + q * US_PER_DEGREE[ch] * SERVO_SIGN[ch];
  if (!(us >= PULSE_MIN_US[ch] && us <= PULSE_MAX_US[ch])) return false;
  uint16_t count = lroundf(us * PCA_CLOCK_HZ / (1e6f * (prescale + 1)));
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(uint8_t(6 + 4 * ch)); Wire.write(0); Wire.write(0);
  Wire.write(count & 0xFF); Wire.write(count >> 8);
  return Wire.endTransmission() == 0;
}

void motorWrite(int side, float v) {   // side 0 = left, 1 = right; v in -1..1
  v = tw::clampf(v * SIDE_SIGN[side], -1, 1);
  digitalWrite(DIR_PIN[side], v < 0 ? HIGH : LOW);
  ledcWrite(PWM_PIN[side], lroundf(fabsf(v) * ((1 << PWM_BITS) - 1)));
}

uint16_t batteryMv() {
  return uint16_t(analogReadMilliVolts(BATT_PIN) * BATT_DIVIDER);
}

void handlePacket() {
  int n = udp.parsePacket();
  if (n <= 0) return;
  uint8_t buf[64];
  if (n > int(sizeof buf)) { udp.clear(); return; }
  udp.read(buf, n);
  tw::Hello h;
  if (tw::parse(buf, n, tw::MAGIC_HELLO, team::LINK_KEY, h)) {
    // authenticated controller (re)pairs: fresh random session invalidates any old packets
    uint32_t session = esp_random() | 1;
    gate.new_session(session);
    ctrlIp = udp.remoteIP(); ctrlPort = udp.remotePort();
    tw::Welcome w{tw::MAGIC_WELCOME, h.ctrl_nonce, session, {0}};
    tw::sign(w, team::LINK_KEY);
    udp.beginPacket(ctrlIp, ctrlPort); udp.write((uint8_t *)&w, sizeof w); udp.endPacket();
    Serial.printf("PAIRED %s:%u session %08lx\n", ctrlIp.toString().c_str(), ctrlPort, (unsigned long)session);
    return;
  }
  tw::Cmd c;
  if (udp.remoteIP() != ctrlIp || udp.remotePort() != ctrlPort) return;   // not our controller
  if (gate.accept(buf, n, team::LINK_KEY, c)) robot.on_command(millis(), c);
}

void setup() {
  pinMode(PCA_OE_PIN, OUTPUT); digitalWrite(PCA_OE_PIN, HIGH);   // servo outputs off until engaged
  pinMode(LED_PIN, OUTPUT);
  for (int k = 0; k < 2; k++) {
    pinMode(DIR_PIN[k], OUTPUT); digitalWrite(DIR_PIN[k], LOW);
    ledcAttach(PWM_PIN[k], PWM_HZ, PWM_BITS); ledcWrite(PWM_PIN[k], 0);
  }
  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(20);
  prescale = uint8_t(lroundf(PCA_CLOCK_HZ / (4096.0f * 50)) - 1);
  pcaOk = pcaWrite(0, 0x10) && pcaWrite(0xFE, prescale) && pcaWrite(1, 4) && pcaWrite(0, 0x20);
  tw::device_name(team::TEAM_NAME, ssid, sizeof ssid);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, team::WIFI_PASSWORD, 6, 0, 1);   // channel 6, visible, max 1 station
  udp.begin(tw::UDP_PORT);
  Serial.printf("THENAR WALKER BOT  AP %s  %s  PCA9685 %s  servo calibration %s\n", ssid,
                WiFi.softAPIP().toString().c_str(), pcaOk ? "ok" : "MISSING", FOLLOWER_CALIBRATED ? "ok" : "PENDING (arm stays off)");
}

void loop() {
  handlePacket();
  uint32_t now = millis();
  if (uint32_t(now - lastTick) < 10) return;     // 100 Hz control loop
  lastTick = now;
  tw::Outputs o = robot.tick(now);
  for (int k = 0; k < 2; k++) motorWrite(k, o.wheel[k]);
  bool armOk = o.servo_pwm_on && pcaOk && FOLLOWER_CALIBRATED;
  if (armOk) {
    for (int i = 0; i < 6; i++)
      if (!servoWrite(i, o.q[i])) { armOk = false; break; }
  }
  // PWM stays on through HOLD so the gripper keeps its grip; only a bus/pulse fault turns it off
  digitalWrite(PCA_OE_PIN, armOk ? LOW : HIGH);
  digitalWrite(LED_PIN, robot.state == tw::ACTIVE ? HIGH : ((now / 250) & 1));
  if (ctrlPort && uint32_t(now - lastStatus) >= 100) {
    lastStatus = now;
    tw::Status s{tw::MAGIC_STATUS, gate.session, gate.last_seq, robot.state, robot.reason, batteryMv(), {0}};
    tw::sign(s, team::LINK_KEY);
    udp.beginPacket(ctrlIp, ctrlPort); udp.write((uint8_t *)&s, sizeof s); udp.endPacket();
  }
}
