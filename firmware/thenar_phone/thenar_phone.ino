// Thenar Walker — phone control (ESP32-S3 + PCA9685 arm + Cytron MDD10A wheels).
// The robot is a Wi-Fi access point "<Team>_TF" (WPA2 password from team_config.h, one phone at a time) and serves
// the control page at http://192.168.4.1 — joystick for the wheels, a slider per arm joint.
// Arm angles use the calibration saved by docs/calibrator ("Save to robot", NVS "tw-armcal"); a joint that was not
// marked done there cannot be switched on, and every angle is clamped to that joint's calibrated safe ends.
// Safety: every joint starts limp; switching a joint on makes it go to its slider angle at once (support the arm).
//         Phone silent for 500 ms -> wheels stop and the arm freezes where it is, PWM kept on so the gripper holds.
//         The KCD4 kill switch still cuts all power.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <ThenarLink.h>
#include <tw_armcal.h>

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

WebServer server(80);
tw::ArmCal cal;
bool calOk = false, pcaOk = false, phone = false;
uint8_t prescale = 121;
float lo[6], hi[6], goal[6], cur[6];
bool on[6];
float wheelT[2] = {0, 0}, wheel[2] = {0, 0};
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

void motorWrite(int side, float v) {   // side 0 = left, 1 = right; v in -1..1
  v = tw::clampf(v * SIDE_SIGN[side], -1, 1);
  digitalWrite(DIR_PIN[side], v < 0 ? HIGH : LOW);
  ledcWrite(PWM_PIN[side], lroundf(fabsf(v) * ((1 << PWM_BITS) - 1)));
}

float batteryV() { return analogReadMilliVolts(BATT_PIN) * BATT_DIVIDER / 1000.0f; }

// ----------------------------------------------------------------- control
void setDrive(float thr, float trn, int sp) {   // same mixing as tw::Robot (turn > 0 = left)
  sp = sp < 0 ? 0 : (sp > 2 ? 2 : sp);
  thr = tw::clampf(thr, -1, 1); trn = tw::clampf(trn, -1, 1);
  if (fabsf(thr) < tw::DRIVE_DEADBAND) thr = 0;
  if (fabsf(trn) < tw::DRIVE_DEADBAND) trn = 0;
  if (thr == 0 && trn != 0) {          // spin in place: breakaway floor + turn scale
    float mag = tw::SPIN_MIN + (tw::TURN_SCALE[sp] - tw::SPIN_MIN) * fabsf(trn);
    wheelT[0] = trn > 0 ? -mag : mag;
    wheelT[1] = -wheelT[0];
  } else {
    float l = thr * tw::SPEED_SCALE[sp] - trn * tw::TURN_SCALE[sp], r = thr * tw::SPEED_SCALE[sp] + trn * tw::TURN_SCALE[sp];
    float m = fmaxf(1.0f, fmaxf(fabsf(l), fabsf(r)));
    wheelT[0] = l / m; wheelT[1] = r / m;
  }
}

void freeze() {   // wheels stop now, every joint holds where it is
  wheelT[0] = wheelT[1] = wheel[0] = wheel[1] = 0;
  for (int i = 0; i < 6; i++) goal[i] = cur[i];
}

bool jointUsable(int i) { return calOk && ((cal.done_mask >> i) & 1) && lo[i] < hi[i]; }

void setJoint(int i, bool want) {
  want = want && pcaOk && jointUsable(i);
  if (want && !on[i]) { cur[i] = goal[i]; on[i] = true; servoWrite(i, cur[i]); }   // first pulse: jumps to the slider
  else if (!want && on[i]) { on[i] = false; chanOff(i); }
}

// ----------------------------------------------------------------- web
extern const char PAGE[];

void sendJson(bool full) {
  String s = "{\"bat\":" + String(batteryV(), 2) + ",\"on\":";
  int m = 0;
  for (int i = 0; i < 6; i++) m |= on[i] << i;
  s += m;
  s += ",\"q\":[";
  for (int i = 0; i < 6; i++) { if (i) s += ','; s += String(cur[i], 1); }
  s += "]";
  if (full) {
    s += ",\"ssid\":\"" + String(ssid) + "\",\"cal\":" + (calOk ? 1 : 0) + ",\"pca\":" + (pcaOk ? 1 : 0) + ",\"usable\":";
    int u = 0;
    for (int i = 0; i < 6; i++) u |= jointUsable(i) << i;
    s += u;
    s += ",\"lo\":[";
    for (int i = 0; i < 6; i++) { if (i) s += ','; s += String(lo[i], 1); }
    s += "],\"hi\":[";
    for (int i = 0; i < 6; i++) { if (i) s += ','; s += String(hi[i], 1); }
    s += "],\"goal\":[";
    for (int i = 0; i < 6; i++) { if (i) s += ','; s += String(goal[i], 1); }
    s += "]";
  }
  s += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

// /c?d=thr,turn,speed&m=on_mask&q=q0,...,q5   or   /c?stop=1   (thr, turn in -100..100)
void handleCmd() {
  lastCmd = millis();
  phone = true;
  if (server.hasArg("stop")) {
    freeze();
  } else {
    int thr = 0, trn = 0, sp = 0;
    if (sscanf(server.arg("d").c_str(), "%d,%d,%d", &thr, &trn, &sp) == 3) setDrive(thr / 100.0f, trn / 100.0f, sp);
    else wheelT[0] = wheelT[1] = 0;
    float q[6];
    if (sscanf(server.arg("q").c_str(), "%f,%f,%f,%f,%f,%f", &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]) == 6)
      for (int i = 0; i < 6; i++)
        if (isfinite(q[i])) goal[i] = tw::clampf(q[i], lo[i], hi[i]);
    if (server.hasArg("m")) {
      int m = server.arg("m").toInt();
      for (int i = 0; i < 6; i++) setJoint(i, (m >> i) & 1);
      updateOe();
    }
  }
  sendJson(false);
}

void setupWeb() {
  server.on("/", [] { server.send_P(200, "text/html", PAGE); });
  server.on("/cfg", [] { sendJson(true); });
  server.on("/c", handleCmd);
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
    ledcAttach(PWM_PIN[k], PWM_HZ, PWM_BITS); ledcWrite(PWM_PIN[k], 0);
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
  WiFi.setSleep(false);
  setupWeb();
  Serial.printf("THENAR PHONE  AP %s  http://%s  PCA9685 %s  arm calibration %s (done mask %02X)\n", ssid,
                WiFi.softAPIP().toString().c_str(), pcaOk ? "ok" : "MISSING", calOk ? "ok" : "MISSING", calOk ? cal.done_mask : 0);
}

void loop() {
  server.handleClient();
  uint32_t now = millis();
  if (uint32_t(now - lastTick) < 20) return;      // 50 Hz
  float dt = (now - lastTick) / 1000.0f;
  if (dt > 0.1f) dt = 0.1f;
  lastTick = now;
  if (phone && uint32_t(now - lastCmd) > PHONE_TIMEOUT_MS) { phone = false; freeze(); Serial.println("phone lost -> stop + hold"); }

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
  digitalWrite(LED_PIN, phone ? HIGH : ((now / 250) & 1));
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
header{position:sticky;top:0;z-index:5;display:flex;align-items:center;gap:10px;padding:10px 14px;background:#0b0f14ee;border-bottom:1px solid var(--line)}
header b{font-size:17px;letter-spacing:.5px;flex:1}
.pill{font-size:12px;padding:3px 9px;border-radius:99px;background:var(--card);color:var(--dim)}
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
</style></head><body>
<header><b>THENAR WALKER</b><span id="link" class="pill">connecting</span><span id="bat" class="pill">-- V</span></header>
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
</main>
<script>
const NAMES=[["J1","Base pan"],["J2","Shoulder lift"],["J3","Elbow"],["J4","Wrist flex"],["J5","Wrist roll"],["J6","Gripper"]];
let cfg=null,q=[0,0,0,0,0,0],mask=0,sp=1,jx=0,jy=0,stopReq=false,busy=false,fails=0;
const $=id=>document.getElementById(id);
function clamp(v,a,b){return Math.min(b,Math.max(a,v))}
function build(){
  const box=$("joints");box.innerHTML="";
  NAMES.forEach(([j,n],i)=>{
    const ok=(cfg.usable>>i)&1,d=document.createElement("div");d.className="j";
    d.innerHTML=`<div class="jh"><span class="n"><b>${j}</b> ${n}<br><small>${ok?cfg.lo[i].toFixed(0)+"° … "+cfg.hi[i].toFixed(0)+"°":"not calibrated"}</small></span>
<span class="v" id="v${i}">--</span><button class="tog" id="t${i}" ${ok?"":"disabled"}>OFF</button></div>
<input type="range" id="r${i}" min="${cfg.lo[i]}" max="${cfg.hi[i]}" step="0.5" value="${q[i]}" ${ok?"":"disabled"}>
<div class="steps">${[-5,-1,1,5].map(s=>`<button data-j="${i}" data-s="${s}" ${ok?"":"disabled"}>${s>0?"+":""}${s}°</button>`).join("")}</div>`;
    box.appendChild(d);
    $("r"+i).oninput=e=>{q[i]=+e.target.value;show()};
    $("t"+i).onclick=()=>{mask^=1<<i;show()};
  });
  box.querySelectorAll(".steps button").forEach(b=>b.onclick=()=>{const i=+b.dataset.j;q[i]=clamp(q[i]+ +b.dataset.s,cfg.lo[i],cfg.hi[i]);show()});
  $("calnote").textContent=!cfg.pca?"PCA9685 not found — check SDA/SCL wiring and servo board power.":
    !cfg.cal?"No arm calibration on the robot: open the calibrator on the laptop and press Save to robot, then reboot.":
    (cfg.usable!=63?"Joints marked not calibrated stay off — finish them in the calibrator and Save to robot.":"");
}
function show(){
  for(let i=0;i<6;i++){const r=$("r"+i);if(!r)continue;if(document.activeElement!==r)r.value=q[i];
    $("v"+i).textContent=q[i].toFixed(1)+"°";const t=$("t"+i),on=(mask>>i)&1;t.textContent=on?"ON":"OFF";t.classList.toggle("on",!!on)}
}
async function tick(){
  if(busy||!cfg)return;busy=true;
  const url=stopReq?"/c?stop=1":`/c?d=${Math.round(jy*100)},${Math.round(-jx*100)},${sp}&m=${mask}&q=${q.map(v=>v.toFixed(1)).join(",")}`;
  const wasStop=stopReq,sent=mask;stopReq=false;
  const ac=new AbortController(),to=setTimeout(()=>ac.abort(),450);
  try{const r=await(await fetch(url,{signal:ac.signal,cache:"no-store"})).json();fails=0;
    $("link").textContent="connected";$("link").className="pill ok";
    $("bat").textContent=r.bat>1?r.bat.toFixed(1)+" V":"-- V";
    if(wasStop){q=r.q.slice();show()}
    const refused=wasStop?0:sent&~r.on;   // a joint the robot would not switch on (not calibrated / no PCA9685)
    if(refused&&(mask&refused)){mask&=~refused;show()}
  }catch(e){if(++fails>2){$("link").textContent="NO LINK";$("link").className="pill bad"}}
  clearTimeout(to);busy=false;
}
async function load(){
  try{cfg=await(await fetch("/cfg",{cache:"no-store"})).json();q=cfg.goal.slice();mask=cfg.on;build();show()}
  catch(e){setTimeout(load,1000)}
}
// joystick
const pad=$("pad"),knob=$("knob");let pid=null;
function setKnob(x,y){jx=x;jy=y;knob.style.transform=`translate(${x*120}%,${-y*120}%)`}
function padMove(e){const b=pad.getBoundingClientRect(),r=b.width/2;let x=(e.clientX-b.left-r)/(r*0.8),y=-(e.clientY-b.top-r)/(r*0.8);
  const m=Math.hypot(x,y);if(m>1){x/=m;y/=m}setKnob(x,y)}
pad.addEventListener("pointerdown",e=>{pid=e.pointerId;try{pad.setPointerCapture(pid)}catch(_){}padMove(e)});
pad.addEventListener("pointermove",e=>{if(e.pointerId===pid)padMove(e)});
["pointerup","pointercancel","lostpointercapture"].forEach(t=>pad.addEventListener(t,e=>{if(e.pointerId===pid){pid=null;setKnob(0,0)}}));
document.querySelectorAll("#speed button").forEach(b=>b.onclick=()=>{sp=+b.dataset.s;document.querySelectorAll("#speed button").forEach(x=>x.classList.toggle("sel",x===b))});
$("stop").onclick=()=>{setKnob(0,0);stopReq=true};
$("zero").onclick=()=>{for(let i=0;i<6;i++)if(cfg)q[i]=clamp(0,cfg.lo[i],cfg.hi[i]);show()};
$("allon").onclick=()=>{if(cfg){mask=cfg.usable;show()}};
$("limp").onclick=()=>{mask=0;show()};
$("gopen").onclick=()=>{if(cfg){q[5]=cfg.hi[5];show()}};
$("gclose").onclick=()=>{if(cfg){q[5]=cfg.lo[5];show()}};
document.addEventListener("visibilitychange",()=>{if(document.hidden)setKnob(0,0)});
load();setInterval(tick,100);
</script></body></html>)HTML";
