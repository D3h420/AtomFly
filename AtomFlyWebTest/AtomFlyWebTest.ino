// Minimal web-controlled test firmware for M5Stack Atom Fly
// Safety-first: test in a clear area. Emergency stop is implemented in firmware,
// but Wi-Fi control can never be guaranteed 100% (RF interference, device issues).
// Always keep a physical power-off option within reach.
// Uses official AtomFly control APIs (AtomFly.h/AtomFly.cpp).

#include <WiFi.h>
#include <WebServer.h>
#include <M5Atom.h>
#include "AtomFly.h"

// =======================
// Configuration (safe defaults; tune carefully)
// =======================
const char* kApSsid = "AtomFly-Test";
const char* kApPass = "atomfly123"; // 8+ chars for WPA2

const IPAddress kApIP(192, 168, 4, 1);
const IPAddress kApNetmask(255, 255, 255, 0);

const uint8_t  kStartPwm     = 95;   // Minimal spin-up PWM after START
const uint8_t  kPwmStep      = 10;   // UP/DOWN step size
const uint8_t  kMaxPwm       = 180;  // Safety cap (tune carefully)
const uint8_t  kYawDelta     = 18;   // Yaw bias added/subtracted during rotate
const uint8_t  kPitchDelta   = 18;   // Pitch bias for forward movement
const uint32_t kRotateMs     = 700;  // Time-based 90-deg test (tune per frame)
const uint32_t kForwardMs    = 450;  // Short forward "nudge" (tune per frame)

// Flip if left/right rotate direction is reversed
const int8_t kYawDirection = 1;

// =======================
// Embedded Web UI
// =======================
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>AtomFly Test Control</title>
<style>
:root { --bg1:#0f1317; --bg2:#1b2128; --card:#1f2730; --text:#f2f5f7; --muted:#9aa4af; --danger:#ff3b30; --accent:#3ad17a; }
* { box-sizing: border-box; }
body { margin:0; font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif; color:var(--text);
       background: radial-gradient(1200px 600px at 20% -10%, #223040 0%, var(--bg1) 55%, var(--bg2) 100%); }
main { max-width:560px; margin:0 auto; padding:16px; }
h1 { font-size:1.2rem; margin:8px 0 12px; letter-spacing:0.5px; }
#status { padding:10px 12px; border-radius:10px; background:var(--card); color:var(--muted);
          margin-bottom:12px; border:1px solid #2b3440; }
button { width:100%; padding:18px 14px; margin:8px 0; font-size:1.05rem; border-radius:12px;
         border:1px solid #2b3440; background:#2b3440; color:var(--text); }
button:active { transform:scale(0.98); }
.stop { background:var(--danger); border-color:#b3231c; font-weight:700; font-size:1.2rem; padding:22px 14px; }
.start { background:linear-gradient(180deg, #3ad17a 0%, #2aa85f 100%); border-color:#1f7a48; font-weight:700; }
.label { display:block; font-size:1.05rem; }
.desc { display:block; font-size:0.82rem; color:#d1d7de; opacity:0.9; margin-top:4px; }
.grid { display:grid; gap:8px; }
.note { margin-top:12px; color:var(--muted); font-size:0.9rem; }
</style>
</head>
<body>
<main>
<h1>AtomFly Test Control</h1>
<div id="status">Status: ładowanie...</div>
<button class="stop" onclick="sendCmd('stop')">🚨 EMERGENCY STOP 🚨</button>
<div class="grid">
  <button class="start" onclick="sendCmd('start')">
    <span class="label">START</span>
    <span class="desc">uruchamia silniki</span>
  </button>
  <button onclick="sendCmd('up')">
    <span class="label">UP</span>
    <span class="desc">podnosi drona w górę</span>
  </button>
  <button onclick="sendCmd('down')">
    <span class="label">DOWN</span>
    <span class="desc">opuszcza drona w dół</span>
  </button>
  <button onclick="sendCmd('forward')">
    <span class="label">PRZÓD</span>
    <span class="desc">krótki ruch do przodu</span>
  </button>
  <button onclick="sendCmd('rotate_left')">
    <span class="label">OBRÓT 90° W LEWO</span>
  </button>
  <button onclick="sendCmd('rotate_right')">
    <span class="label">OBRÓT 90° W PRAWO</span>
  </button>
</div>
<div class="note">Najważniejsze: STOP działa natychmiast i rozbraja drona. Zadbaj o fizyczne odcięcie zasilania w razie potrzeby.</div>
</main>
<script>
const statusEl = document.getElementById('status');

async function sendCmd(cmd) {
  try {
    const res = await fetch('/' + cmd, { cache: 'no-store' });
    const text = await res.text();
    statusEl.textContent = 'Komenda: ' + cmd + ' | ' + text;
  } catch (e) {
    statusEl.textContent = 'Błąd komendy: ' + cmd;
  }
}

async function poll() {
  try {
    const res = await fetch('/status', { cache: 'no-store' });
    if (!res.ok) return;
    const s = await res.json();
    statusEl.textContent = 'Status: ' + (s.armed ? 'UZBROJONY' : 'ROZBROJONY') +
      (s.estop ? ' | STOP' : '') + ' | PWM ' + s.pwm + ' | ' + s.action;
  } catch (e) {}
}

setInterval(poll, 800);
poll();
</script>
</body>
</html>
)HTML";

// =======================
// Firmware state
// =======================
WebServer server(80);

enum Action { ACTION_NONE, ACTION_ROTATE_LEFT, ACTION_ROTATE_RIGHT, ACTION_FORWARD };

bool armed = false;
bool estop_latched = false;
uint8_t base_pwm = 0;

Action action = ACTION_NONE;
uint32_t action_start_ms = 0;
uint32_t action_end_ms = 0;

// =======================
// Helpers
// =======================
bool timePassed(uint32_t now, uint32_t end_ms) {
  return (int32_t)(now - end_ms) >= 0;
}

void applyMotors(uint8_t base, int16_t yaw_delta, int16_t pitch_delta) {
  // Yaw: A/C vs B/D. Pitch: front vs rear.
  // If forward/back is reversed, swap front/rear motors below.
  const uint8_t motor_front_left  = AtomFly::kMotor_A;
  const uint8_t motor_front_right = AtomFly::kMotor_B;
  const uint8_t motor_rear_right  = AtomFly::kMotor_C;
  const uint8_t motor_rear_left   = AtomFly::kMotor_D;

  int16_t delta_yaw = yaw_delta * kYawDirection;
  int16_t pwm_fl = base;
  int16_t pwm_fr = base;
  int16_t pwm_rr = base;
  int16_t pwm_rl = base;

  // Yaw influence
  pwm_fl += delta_yaw;
  pwm_rr += delta_yaw;
  pwm_fr -= delta_yaw;
  pwm_rl -= delta_yaw;

  // Pitch influence (forward: reduce front, increase rear)
  pwm_fl -= pitch_delta;
  pwm_fr -= pitch_delta;
  pwm_rr += pitch_delta;
  pwm_rl += pitch_delta;

  pwm_fl = constrain(pwm_fl, 0, 255);
  pwm_fr = constrain(pwm_fr, 0, 255);
  pwm_rr = constrain(pwm_rr, 0, 255);
  pwm_rl = constrain(pwm_rl, 0, 255);

  fly.WritePWM(motor_front_left,  (uint8_t)pwm_fl);
  fly.WritePWM(motor_front_right, (uint8_t)pwm_fr);
  fly.WritePWM(motor_rear_right,  (uint8_t)pwm_rr);
  fly.WritePWM(motor_rear_left,   (uint8_t)pwm_rl);
}

void emergencyStop() {
  action = ACTION_NONE;
  base_pwm = 0;
  armed = false;
  estop_latched = true;
  fly.WriteAllPWM(0);
}

const char* actionName() {
  switch (action) {
    case ACTION_ROTATE_LEFT:  return "rotate_left";
    case ACTION_ROTATE_RIGHT: return "rotate_right";
    case ACTION_FORWARD:      return "forward";
    default:                  return "idle";
  }
}

// =======================
// HTTP handlers
// =======================
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleStart() {
  estop_latched = false; // explicit re-arm
  armed = true;
  action = ACTION_NONE;
  base_pwm = kStartPwm;
  server.send(200, "text/plain", "STARTED");
}

void handleRotateLeft() {
  if (!armed || estop_latched) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_ROTATE_LEFT;
  action_start_ms = millis();
  action_end_ms = action_start_ms + kRotateMs;
  server.send(200, "text/plain", "ROTATE_LEFT");
}

void handleRotateRight() {
  if (!armed || estop_latched) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_ROTATE_RIGHT;
  action_start_ms = millis();
  action_end_ms = action_start_ms + kRotateMs;
  server.send(200, "text/plain", "ROTATE_RIGHT");
}

void handleUp() {
  if (!armed || estop_latched) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_NONE;
  uint16_t next_pwm = base_pwm + kPwmStep;
  base_pwm = (next_pwm > kMaxPwm) ? kMaxPwm : (uint8_t)next_pwm;
  server.send(200, "text/plain", "UP");
}

void handleDown() {
  if (!armed || estop_latched) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_NONE;
  int16_t next_pwm = (int16_t)base_pwm - (int16_t)kPwmStep;
  if (next_pwm <= 0) {
    base_pwm = 0;
    armed = false; // stop motors when fully down
  } else {
    base_pwm = (uint8_t)next_pwm;
  }
  server.send(200, "text/plain", "DOWN");
}

void handleForward() {
  if (!armed || estop_latched) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_FORWARD;
  action_start_ms = millis();
  action_end_ms = action_start_ms + kForwardMs;
  server.send(200, "text/plain", "FORWARD");
}

void handleStop() {
  emergencyStop();
  server.send(200, "text/plain", "STOPPED");
}

void handleStatus() {
  String json = "{";
  json += "\"armed\":"; json += (armed ? "true" : "false"); json += ",";
  json += "\"estop\":"; json += (estop_latched ? "true" : "false"); json += ",";
  json += "\"action\":\""; json += actionName(); json += "\",";
  json += "\"pwm\":"; json += String(base_pwm);
  json += "}";
  server.send(200, "application/json", json);
}

// =======================
// Setup / Loop
// =======================
void setup() {
  M5.begin(true, false, true);
  Serial.begin(115200);

  fly.begin();
  if (fly.initFly() != 0) {
    Serial.println("AtomFly init failed");
    while (1) {
      fly.WriteAllPWM(0);
      delay(1000);
    }
  }
  fly.WriteAllPWM(0);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); // reduce latency for control links
  WiFi.softAPConfig(kApIP, kApIP, kApNetmask);
  WiFi.softAP(kApSsid, kApPass);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_GET, handleStart);
  server.on("/up", HTTP_GET, handleUp);
  server.on("/down", HTTP_GET, handleDown);
  server.on("/forward", HTTP_GET, handleForward);
  server.on("/rotate_left", HTTP_GET, handleRotateLeft);
  server.on("/rotate_right", HTTP_GET, handleRotateRight);
  server.on("/stop", HTTP_ANY, handleStop);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound([]() { server.send(404, "text/plain", "Not Found"); });
  server.begin();
}

void loop() {
  server.handleClient();
  M5.update();

  if (M5.Btn.wasPressed()) {
    emergencyStop();
  }

  if (estop_latched) {
    fly.WriteAllPWM(0);
    return;
  }

  if (!armed) {
    fly.WriteAllPWM(0);
    return;
  }

  uint32_t now = millis();
  int16_t yaw_delta = 0;
  int16_t pitch_delta = 0;

  if (action == ACTION_ROTATE_LEFT || action == ACTION_ROTATE_RIGHT) {
    if (timePassed(now, action_end_ms)) {
      action = ACTION_NONE;
    } else {
      yaw_delta = (action == ACTION_ROTATE_RIGHT) ? kYawDelta : -kYawDelta;
    }
  } else if (action == ACTION_FORWARD) {
    if (timePassed(now, action_end_ms)) {
      action = ACTION_NONE;
    } else {
      pitch_delta = kPitchDelta;
    }
  }

  applyMotors(base_pwm, yaw_delta, pitch_delta);
}
