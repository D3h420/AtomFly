// Minimal web-controlled test firmware for M5Stack Atom Fly
// Safety-first: test in a clear area. Emergency stop is implemented in firmware,
// but Wi-Fi control can never be guaranteed 100% (RF interference, device issues).
// Always keep a physical power-off option within reach.
// Uses official AtomFly control APIs (AtomFly.h/AtomFly.cpp).

#include <WiFi.h>
#include <WebServer.h>
#include <M5Atom.h>
#include <math.h>
#include "AtomFly.h"

// =======================
// Configuration (safe defaults; tune carefully)
// =======================
const char* kApSsid = "AtomFly-Test";
const char* kApPass = "atomfly123"; // 8+ chars for WPA2

const IPAddress kApIP(192, 168, 4, 1);
const IPAddress kApNetmask(255, 255, 255, 0);

const uint8_t  kIdlePwm      = 55;   // Idle PWM after START (should NOT lift)
const uint8_t  kPwmStep      = 10;   // UP/DOWN step size
const uint8_t  kMaxPwm       = 180;  // Safety cap (tune carefully)
const uint8_t  kPwmSlewStep  = 2;    // PWM change per slew step
const uint16_t kPwmSlewMs    = 20;   // Slew interval in ms
const float    kImuAlpha     = 0.98f; // Complementary filter (gyro vs accel)
const float    kYawHoldKp    = 1.1f; // PWM per deg/s (gyro Z). Tune to stop spin.
const int16_t  kYawHoldMax   = 22;   // Max yaw correction
const float    kYawHoldDeadband = 1.5f; // deg/s deadband to avoid small bias
const uint8_t  kYawHoldMinPwm = 35;  // Only apply yaw-hold above this PWM
const uint16_t kGyroCalibMs   = 600; // Gyro Z bias calibration time
const uint16_t kGyroSampleMs  = 5;   // Gyro sample interval during calibration
const float    kLevelKp       = 1.2f; // PWM per deg (roll/pitch)
const float    kLevelKi       = 0.02f; // PWM per deg*s (auto-trim)
const float    kLevelKd       = 0.08f; // PWM per deg/s
const int16_t  kLevelMax      = 18;   // Max roll/pitch correction
const float    kLevelDeadband = 0.8f; // deg deadband
const uint8_t  kLevelMinPwm   = 45;   // Only apply leveling above this PWM
const float    kLevelIMax     = 40.0f; // Integral clamp (deg*s)
const float    kAltStepCm     = 20.0f; // UP/DOWN step
const float    kAltMinCm      = 5.0f;  // Minimum target altitude
const float    kAltMaxCm      = 120.0f; // Maximum target altitude
const uint16_t kAltSampleMs   = 50;   // ToF sampling period
const uint16_t kAltCtrlMs     = 50;   // Altitude control period
const float    kAltFilterAlpha = 0.25f; // ToF low-pass filter
const float    kAltKp         = 0.8f; // PWM per cm
const float    kAltKi         = 0.06f; // PWM per cm*s (integrator)
const float    kAltKd         = 0.25f; // PWM per (cm/s)
const uint16_t kTofMinMm      = 30;   // Ignore below 3 cm
const uint16_t kTofMaxMm      = 2000; // Ignore beyond 200 cm
const uint16_t kAltTimeoutMs  = 400;  // ToF timeout before disabling hold
const uint8_t  kAltMinPwm     = 70;   // Minimum PWM for altitude hold
const uint8_t  kTakeoffPwm    = 90;   // Initial PWM for takeoff
const uint8_t  kTakeoffMaxPwm = 130;  // Max PWM during takeoff ramp
const uint8_t  kTakeoffRampStep = 3;  // PWM step during takeoff ramp
const uint16_t kTakeoffTimeoutMs = 2200; // Max time to try takeoff
const float    kAirborneMinCm = 8.0f; // Consider airborne above this height
const uint8_t  kYawDelta     = 18;   // Yaw bias added/subtracted during rotate
const float    kForwardTiltDeg = 6.0f; // Forward tilt angle during "forward"
const uint32_t kRotateMs     = 700;  // Time-based 90-deg test (tune per frame)
const uint32_t kForwardMs    = 450;  // Short forward "nudge" (tune per frame)

// Flip if left/right rotate direction is reversed
const int8_t kYawDirection = 1;
const int8_t kYawHoldDirection = 1; // Flip if yaw-hold makes spin worse
const int8_t kRollDirection = 1;    // Flip if roll leveling makes it worse
const int8_t kPitchDirection = 1;   // Flip if pitch leveling makes it worse

// Motor trim offsets to counter small imbalances (FL, FR, RR, RL).
// Use small values like -3..+3. Defaults to 0.
const int8_t kMotorTrim[4] = { 0, 0, 0, 0 };

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
    <span class="desc">uruchamia silniki (niski ciąg)</span>
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
      (s.estop ? ' | STOP' : '') +
      ' | PWM ' + s.pwm +
      ' | ALT ' + (s.alt_hold ? 'HOLD' : 'MANUAL') +
      ' ' + s.alt_cm + 'cm -> ' + s.alt_target_cm + 'cm' +
      ' | ' + s.action;
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
uint8_t base_pwm = 0;   // current PWM actually applied
uint8_t target_pwm = 0; // requested PWM (ramps to this)
uint32_t last_slew_ms = 0;

bool gyro_calibrating = false;
uint32_t gyro_calib_start_ms = 0;
uint32_t gyro_calib_last_ms = 0;
float gyro_z_bias = 0.0f;
float gyro_x_bias = 0.0f;
float gyro_y_bias = 0.0f;
float gyro_z_acc = 0.0f;
float gyro_x_acc = 0.0f;
float gyro_y_acc = 0.0f;
uint16_t gyro_z_samples = 0;
float level_roll_bias = 0.0f;
float level_pitch_bias = 0.0f;
float level_roll_acc = 0.0f;
float level_pitch_acc = 0.0f;
uint16_t level_samples = 0;
float roll_deg = 0.0f;
float pitch_deg = 0.0f;
bool imu_inited = false;
uint32_t last_imu_ms = 0;
float roll_i = 0.0f;
float pitch_i = 0.0f;
bool alt_hold = false;
float target_alt_cm = 0.0f;
float alt_cm = 0.0f;
bool alt_valid = false;
uint32_t last_tof_ms = 0;
uint32_t last_tof_update_ms = 0;
uint32_t last_alt_ctrl_ms = 0;
float alt_throttle_base = 0.0f;
float last_alt_cm = 0.0f;
bool takeoff_active = false;
uint32_t takeoff_start_ms = 0;

Action action = ACTION_NONE;
uint32_t action_start_ms = 0;
uint32_t action_end_ms = 0;

// =======================
// Helpers
// =======================
bool timePassed(uint32_t now, uint32_t end_ms) {
  return (int32_t)(now - end_ms) >= 0;
}

void readImuMapped(float& acc_x, float& acc_y, float& acc_z,
                   float& roll_rate, float& pitch_rate, float& yaw_rate) {
  float ax = 0.0f, ay = 0.0f, az = 0.0f;
  float gx = 0.0f, gy = 0.0f, gz = 0.0f;
  fly.getAccelData(&ax, &ay, &az);
  fly.getGyroData(&gx, &gy, &gz);

  // AtomFly library uses raw axes for attitude; keep raw mapping here.
  acc_x = ax;
  acc_y = ay;
  acc_z = az;

  // Standard body rates: X=roll, Y=pitch, Z=yaw
  roll_rate  = gx;
  pitch_rate = gy;
  yaw_rate   = gz;
}

void updateTof(uint32_t now) {
  if (!timePassed(now, last_tof_ms + kAltSampleMs)) return;
  last_tof_ms = now;

  uint16_t dist_mm = fly.readTOF();
  if (dist_mm < kTofMinMm || dist_mm > kTofMaxMm) {
    return;
  }

  float cm = dist_mm * 0.1f;
  if (!alt_valid) {
    alt_cm = cm;
    alt_valid = true;
  } else {
    alt_cm = alt_cm + kAltFilterAlpha * (cm - alt_cm);
  }
  last_tof_update_ms = now;
}

void applyMotors(uint8_t base, int16_t yaw_delta, int16_t pitch_delta, int16_t roll_delta) {
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

  // Roll influence (positive roll_delta lifts right, lowers left)
  pwm_fl -= roll_delta;
  pwm_rl -= roll_delta;
  pwm_fr += roll_delta;
  pwm_rr += roll_delta;

  // Per-motor trim (small corrections only)
  pwm_fl += kMotorTrim[0];
  pwm_fr += kMotorTrim[1];
  pwm_rr += kMotorTrim[2];
  pwm_rl += kMotorTrim[3];

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
  target_pwm = 0;
  alt_hold = false;
  takeoff_active = false;
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
  target_pwm = 0;
  base_pwm = 0;
  gyro_calibrating = true;
  gyro_calib_start_ms = millis();
  gyro_calib_last_ms = 0;
  gyro_z_acc = 0.0f;
  gyro_x_acc = 0.0f;
  gyro_y_acc = 0.0f;
  gyro_z_samples = 0;
  level_roll_acc = 0.0f;
  level_pitch_acc = 0.0f;
  level_samples = 0;
  roll_i = 0.0f;
  pitch_i = 0.0f;
  imu_inited = false;
  last_imu_ms = 0;
  alt_hold = false;
  target_alt_cm = 0.0f;
  alt_valid = false;
  last_tof_ms = 0;
  last_tof_update_ms = 0;
  last_alt_ctrl_ms = 0;
  alt_throttle_base = kIdlePwm;
  last_alt_cm = 0.0f;
  takeoff_active = false;
  takeoff_start_ms = 0;
  server.send(200, "text/plain", "STARTED");
}

void handleRotateLeft() {
  if (!armed || estop_latched || gyro_calibrating) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_ROTATE_LEFT;
  action_start_ms = millis();
  action_end_ms = action_start_ms + kRotateMs;
  server.send(200, "text/plain", "ROTATE_LEFT");
}

void handleRotateRight() {
  if (!armed || estop_latched || gyro_calibrating) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_ROTATE_RIGHT;
  action_start_ms = millis();
  action_end_ms = action_start_ms + kRotateMs;
  server.send(200, "text/plain", "ROTATE_RIGHT");
}

void handleUp() {
  if (!armed || estop_latched || gyro_calibrating) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_NONE;
  if (alt_hold) {
    target_alt_cm += kAltStepCm;
    target_alt_cm = constrain(target_alt_cm, kAltMinCm, kAltMaxCm);
  } else {
    if (alt_valid && alt_cm > kAltMinCm) {
      alt_hold = true;
      target_alt_cm = alt_cm + kAltStepCm;
      target_alt_cm = constrain(target_alt_cm, kAltMinCm, kAltMaxCm);
    } else {
      takeoff_active = true;
      takeoff_start_ms = millis();
      target_pwm = max(target_pwm, kTakeoffPwm);
      alt_throttle_base = max(alt_throttle_base, (float)kTakeoffPwm);
      last_tof_update_ms = millis();
    }
  }
  server.send(200, "text/plain", "UP");
}

void handleDown() {
  if (!armed || estop_latched || gyro_calibrating) {
    server.send(409, "text/plain", "IGNORED");
    return;
  }
  action = ACTION_NONE;
  if (alt_hold) {
    target_alt_cm -= kAltStepCm;
    if (target_alt_cm <= kAltMinCm) {
      target_alt_cm = kAltMinCm;
      alt_hold = false;
      target_pwm = kIdlePwm;
    }
  } else {
    int16_t next_pwm = (int16_t)target_pwm - (int16_t)kPwmStep;
    if (next_pwm <= 0) {
      target_pwm = 0;
      armed = false; // stop motors when fully down
    } else {
      target_pwm = (uint8_t)next_pwm;
    }
  }
  server.send(200, "text/plain", "DOWN");
}

void handleForward() {
  if (!armed || estop_latched || gyro_calibrating) {
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
  json += "\"calib\":"; json += (gyro_calibrating ? "true" : "false"); json += ",";
  json += "\"action\":\""; json += actionName(); json += "\",";
  json += "\"pwm\":"; json += String(base_pwm); json += ",";
  json += "\"target_pwm\":"; json += String(target_pwm); json += ",";
  json += "\"alt_hold\":"; json += (alt_hold ? "true" : "false"); json += ",";
  json += "\"takeoff\":"; json += (takeoff_active ? "true" : "false"); json += ",";
  json += "\"alt_cm\":"; json += String(alt_cm, 1); json += ",";
  json += "\"alt_target_cm\":"; json += String(target_alt_cm, 1); json += ",";
  json += "\"roll_deg\":"; json += String(roll_deg, 1); json += ",";
  json += "\"pitch_deg\":"; json += String(pitch_deg, 1); json += ",";
  json += "\"gyro_z_bias\":"; json += String(gyro_z_bias, 2); json += ",";
  json += "\"level_roll_bias\":"; json += String(level_roll_bias, 2); json += ",";
  json += "\"level_pitch_bias\":"; json += String(level_pitch_bias, 2);
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
    base_pwm = 0;
    alt_hold = false;
    takeoff_active = false;
    return;
  }

  if (!armed) {
    fly.WriteAllPWM(0);
    base_pwm = 0;
    alt_hold = false;
    takeoff_active = false;
    return;
  }

  uint32_t now = millis();
  int16_t yaw_delta = 0;
  int16_t pitch_delta = 0;
  int16_t roll_delta = 0;
  float pitch_target_deg = 0.0f;
  bool rotating = false;

  if (gyro_calibrating) {
    if (timePassed(now, gyro_calib_start_ms + kGyroCalibMs)) {
      if (gyro_z_samples > 0) {
        gyro_x_bias = gyro_x_acc / (float)gyro_z_samples;
        gyro_y_bias = gyro_y_acc / (float)gyro_z_samples;
        gyro_z_bias = gyro_z_acc / (float)gyro_z_samples;
      } else {
        gyro_x_bias = 0.0f;
        gyro_y_bias = 0.0f;
        gyro_z_bias = 0.0f;
      }
      if (level_samples > 0) {
        level_roll_bias = level_roll_acc / (float)level_samples;
        level_pitch_bias = level_pitch_acc / (float)level_samples;
      } else {
        level_roll_bias = 0.0f;
        level_pitch_bias = 0.0f;
      }
      gyro_calibrating = false;
      target_pwm = kIdlePwm;
    } else if (timePassed(now, gyro_calib_last_ms + kGyroSampleMs)) {
      float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
      float roll_rate = 0.0f, pitch_rate = 0.0f, yaw_rate = 0.0f;
      readImuMapped(acc_x, acc_y, acc_z, roll_rate, pitch_rate, yaw_rate);
      gyro_x_acc += roll_rate;
      gyro_y_acc += pitch_rate;
      gyro_z_acc += yaw_rate;
      gyro_z_samples++;

      float pitch = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 57.2958f;
      float roll = atan2f(acc_y, acc_z) * 57.2958f;
      level_pitch_acc += pitch;
      level_roll_acc += roll;
      level_samples++;
      gyro_calib_last_ms = now;
    }
    fly.WriteAllPWM(0);
    base_pwm = 0;
    return;
  }

  if (action == ACTION_ROTATE_LEFT || action == ACTION_ROTATE_RIGHT) {
    if (timePassed(now, action_end_ms)) {
      action = ACTION_NONE;
    } else {
      yaw_delta = (action == ACTION_ROTATE_RIGHT) ? kYawDelta : -kYawDelta;
      rotating = true;
    }
  } else if (action == ACTION_FORWARD) {
    if (timePassed(now, action_end_ms)) {
      action = ACTION_NONE;
    } else {
      pitch_target_deg = -kForwardTiltDeg;
    }
  }

  float acc_x = 0.0f, acc_y = 0.0f, acc_z = 0.0f;
  float roll_rate = 0.0f, pitch_rate = 0.0f, yaw_rate = 0.0f;
  readImuMapped(acc_x, acc_y, acc_z, roll_rate, pitch_rate, yaw_rate);
  roll_rate -= gyro_x_bias;
  pitch_rate -= gyro_y_bias;
  yaw_rate -= gyro_z_bias;

  float roll_acc = atan2f(acc_y, acc_z) * 57.2958f;
  float pitch_acc = atan2f(-acc_x, sqrtf(acc_y * acc_y + acc_z * acc_z)) * 57.2958f;
  roll_acc -= level_roll_bias;
  pitch_acc -= level_pitch_bias;

  float dt = 0.0f;
  if (last_imu_ms > 0) {
    dt = (now - last_imu_ms) / 1000.0f;
  }
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;

  if (!imu_inited) {
    roll_deg = roll_acc;
    pitch_deg = pitch_acc;
    imu_inited = true;
  } else {
    roll_deg = kImuAlpha * (roll_deg + roll_rate * dt) + (1.0f - kImuAlpha) * roll_acc;
    pitch_deg = kImuAlpha * (pitch_deg + pitch_rate * dt) + (1.0f - kImuAlpha) * pitch_acc;
  }
  last_imu_ms = now;

  updateTof(now);

  bool leveling_active = (base_pwm >= kTakeoffPwm) || alt_hold || takeoff_active;

  // Yaw hold (stabilize rotation) when not actively rotating
  if (!rotating && leveling_active && base_pwm >= kYawHoldMinPwm) {
    float yaw_use = yaw_rate;
    if (fabsf(yaw_use) < kYawHoldDeadband) {
      yaw_use = 0.0f;
    }
    int16_t yaw_hold = (int16_t)constrain((int)(-yaw_use * kYawHoldKp), -kYawHoldMax, kYawHoldMax);
    yaw_hold *= kYawHoldDirection;
    yaw_delta += yaw_hold;
  }

  // Leveling with auto-trim (roll always, pitch follows target)
  if (leveling_active && base_pwm >= kLevelMinPwm) {
    bool airborne = alt_valid && (alt_cm > kAirborneMinCm);
    float roll_err = -roll_deg;
    if (fabsf(roll_err) < kLevelDeadband) {
      roll_err = 0.0f;
    } else if (airborne) {
      roll_i += roll_err * dt;
      roll_i = constrain(roll_i, -kLevelIMax, kLevelIMax);
    }
    if (!airborne) roll_i = 0.0f;
    float roll_cmd = (kLevelKp * roll_err) + (kLevelKi * roll_i) - (kLevelKd * roll_rate);
    int16_t roll_correction = (int16_t)constrain((int)roll_cmd, -kLevelMax, kLevelMax);
    roll_delta += (roll_correction * kRollDirection);

    float pitch_err = pitch_target_deg - pitch_deg;
    if (fabsf(pitch_err) < kLevelDeadband) {
      pitch_err = 0.0f;
    } else if (action != ACTION_FORWARD && airborne) {
      pitch_i += pitch_err * dt;
      pitch_i = constrain(pitch_i, -kLevelIMax, kLevelIMax);
    }
    if (!airborne) pitch_i = 0.0f;
    float pitch_cmd = (kLevelKp * pitch_err) + (kLevelKi * pitch_i) - (kLevelKd * pitch_rate);
    int16_t pitch_correction = (int16_t)constrain((int)pitch_cmd, -kLevelMax, kLevelMax);
    pitch_delta += (pitch_correction * kPitchDirection);
  }

  // Takeoff ramp (if ToF not yet valid)
  if (takeoff_active) {
    if (alt_valid && alt_cm > kAltMinCm) {
      alt_hold = true;
      takeoff_active = false;
      target_alt_cm = alt_cm + kAltStepCm;
      target_alt_cm = constrain(target_alt_cm, kAltMinCm, kAltMaxCm);
      last_alt_ctrl_ms = 0;
    } else if (timePassed(now, takeoff_start_ms + kTakeoffTimeoutMs)) {
      takeoff_active = false;
    } else if (timePassed(now, last_alt_ctrl_ms + kAltCtrlMs)) {
      uint16_t next_pwm = target_pwm + kTakeoffRampStep;
      target_pwm = (next_pwm > kTakeoffMaxPwm) ? kTakeoffMaxPwm : (uint8_t)next_pwm;
      alt_throttle_base = max(alt_throttle_base, (float)target_pwm);
      last_alt_ctrl_ms = now;
    }
  }

  // Altitude hold (ToF)
  if (alt_hold) {
    if (timePassed(now, last_tof_update_ms + kAltTimeoutMs)) {
      alt_hold = false;
      target_pwm = kIdlePwm;
    } else if (alt_valid && timePassed(now, last_alt_ctrl_ms + kAltCtrlMs)) {
      float dt_alt = (last_alt_ctrl_ms > 0) ? (now - last_alt_ctrl_ms) / 1000.0f : (kAltCtrlMs / 1000.0f);
      if (dt_alt <= 0.0f || dt_alt > 0.2f) dt_alt = kAltCtrlMs / 1000.0f;

      float err = target_alt_cm - alt_cm;
      float alt_vel = (alt_cm - last_alt_cm) / dt_alt;
      alt_throttle_base += kAltKi * err * dt_alt;
      alt_throttle_base = constrain(alt_throttle_base, (float)kAltMinPwm, (float)kMaxPwm);

      float desired = alt_throttle_base + (kAltKp * err) - (kAltKd * alt_vel);
      desired = constrain(desired, (float)kAltMinPwm, (float)kMaxPwm);
      target_pwm = (uint8_t)desired;

      last_alt_cm = alt_cm;
      last_alt_ctrl_ms = now;
    }
  }

  if (timePassed(now, last_slew_ms + kPwmSlewMs)) {
    if (base_pwm < target_pwm) {
      uint16_t next_pwm = base_pwm + kPwmSlewStep;
      base_pwm = (next_pwm > target_pwm) ? target_pwm : (uint8_t)next_pwm;
    } else if (base_pwm > target_pwm) {
      int16_t next_pwm = (int16_t)base_pwm - (int16_t)kPwmSlewStep;
      base_pwm = (next_pwm < (int16_t)target_pwm) ? target_pwm : (uint8_t)next_pwm;
    }
    last_slew_ms = now;
  }

  applyMotors(base_pwm, yaw_delta, pitch_delta, roll_delta);
}
