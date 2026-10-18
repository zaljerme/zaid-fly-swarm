// =============================================================================
//  Zaid Fly — swarm drone flight controller
//  Board: Seeed XIAO ESP32-S3 on the Zaid Fly carrier PCB (MPU6050 + 4x SI2300)
//
//  Every drone runs this same binary. Its swarm slot (0..19) lives in NVS and
//  is set once over USB serial:   id 7
//
//  Loop (1 kHz, fixed rate):
//    read IMU -> attitude filter -> take newest radio command -> safety state
//    machine -> angle PID -> rate PID -> X-quad mixer -> 20 kHz PWM
// =============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include "config.h"
#include "imu.h"
#include "pid.h"
#include "radio.h"
#include "swarm_protocol.h"

// ---- LEDC shim (Arduino-ESP32 2.x vs 3.x API) --------------------------------
#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void motorAttach(uint8_t i) { ledcAttach(PIN_MOTOR[i], MOTOR_PWM_HZ, MOTOR_PWM_BITS); }
static void motorWrite(uint8_t i, uint32_t d) { ledcWrite(PIN_MOTOR[i], d); }
#else
static void motorAttach(uint8_t i) { ledcSetup(i, MOTOR_PWM_HZ, MOTOR_PWM_BITS); ledcAttachPin(PIN_MOTOR[i], i); }
static void motorWrite(uint8_t i, uint32_t d) { ledcWrite(i, d); }
#endif

static Imu         imu;
static Preferences prefs;
static uint8_t     nodeId = 0;
static FlightState state = STATE_BOOT;

static Pid pidRoll (RATE_P_RP, RATE_I_RP, RATE_D_RP, I_LIMIT);
static Pid pidPitch(RATE_P_RP, RATE_I_RP, RATE_D_RP, I_LIMIT);
static Pid pidYaw  (RATE_P_Y,  RATE_I_Y,  RATE_D_Y,  I_LIMIT);

static RadioCmd  rc = {};               // newest command for this drone
static uint32_t  failsafeStartMs = 0;
static uint16_t  failsafeThrottle = 0;
static float     vbatMv = 4000;
static uint16_t  worstLoopUs = 0;

// -----------------------------------------------------------------------------
static void motorsOff() { for (uint8_t i = 0; i < 4; i++) motorWrite(i, 0); }

static void setLeds(uint32_t now) {
  bool blink  = (now / 100) % 2;
  bool slow   = (now / 500) % 2;
  bool red = false, green = false;
  switch (state) {
    case STATE_CALIBRATING:   red = blink; green = !blink; break;
    case STATE_DISARMED:      green = slow;               break;
    case STATE_ARMED:         green = true;               break;
    case STATE_FAILSAFE_LAND: red = blink;                break;
    case STATE_KILLED:        red = true;                 break;
    case STATE_LOW_BATTERY:   red = slow; green = true;   break;
    default: break;
  }
  if (rc.cmd.flags & CMD_BLINK) { red = green = blink; }  // "which one is #7?"
  digitalWrite(PIN_LED_RED, red);
  digitalWrite(PIN_LED_GREEN, green);
  digitalWrite(PIN_HEADLIGHT, (rc.cmd.flags & CMD_HEADLIGHT) ? HIGH : LOW);
  digitalWrite(PIN_TAILLIGHT, (rc.cmd.flags & CMD_TAILLIGHT) ? HIGH : LOW);
}

static void readBattery() {
  float mv = analogReadMilliVolts(PIN_VBAT) * VBAT_DIVIDER;
  vbatMv += 0.02f * (mv - vbatMv);       // slow LPF, rejects motor noise
}

static void calibrate() {
  state = STATE_CALIBRATING;
  motorsOff();
  uint32_t t0 = millis();
  while (millis() - t0 < 300) setLeds(millis());   // let it settle
  imu.calibrate(1000);
  pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
  state = STATE_DISARMED;
}

static void handleSerial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    buf[n] = 0; n = 0;
    if (!strncmp(buf, "id ", 3)) {
      int v = atoi(buf + 3);
      if (v >= 0 && v < SWARM_MAX_NODES) {
        prefs.putUChar("node", v);
        Serial.printf("node id set to %d, rebooting\n", v);
        delay(100);
        ESP.restart();
      }
      Serial.println("id must be 0..19");
    } else if (!strcmp(buf, "cal")) {
      if (state != STATE_ARMED) { calibrate(); Serial.println("calibrated"); }
    } else if (!strcmp(buf, "info")) {
      Serial.printf("Zaid Fly node %u | state %u | vbat %.0f mV | roll %.1f pitch %.1f | loop %u us | mac %s\n",
                    nodeId, state, vbatMv, imu.roll, imu.pitch, worstLoopUs, WiFi.macAddress().c_str());
    } else if (buf[0]) {
      Serial.println("commands: id <0-19> | cal | info");
    }
  }
}

// ---- Safety state machine ----------------------------------------------------
static void updateState(uint32_t nowMs, bool linkOk) {
  bool killReq = (rc.cmd.flags & CMD_KILL) || (rc.frameFlags & FRAME_ESTOP_ALL);
  bool armReq  = rc.cmd.flags & CMD_ARM;
  bool crashed = fabsf(imu.roll) > MAX_TILT_KILL_DEG || fabsf(imu.pitch) > MAX_TILT_KILL_DEG;

  if (state == STATE_CALIBRATING) return;

  if (killReq || (state == STATE_ARMED && crashed)) {
    state = STATE_KILLED;
    return;
  }

  switch (state) {
    case STATE_KILLED:
      // Stay dead until the operator explicitly disarms (clears ARM + KILL)
      if (!armReq && linkOk) state = STATE_DISARMED;
      break;

    case STATE_DISARMED:
    case STATE_LOW_BATTERY:
      if (vbatMv < VBAT_CRIT_MV && vbatMv > 2000) { state = STATE_LOW_BATTERY; break; }
      if (state == STATE_LOW_BATTERY && vbatMv > VBAT_LOW_MV) state = STATE_DISARMED;
      {  // calibrate on the rising edge of the flag only
        static bool prevCal = false;
        bool cal = rc.cmd.flags & CMD_CALIBRATE;
        if (cal && !prevCal) { prevCal = true; calibrate(); break; }
        prevCal = cal;
      }
      // Arm only on a fresh link with throttle at the bottom
      if (linkOk && armReq && rc.cmd.throttle < 50 && state == STATE_DISARMED) {
        pidRoll.reset(); pidPitch.reset(); pidYaw.reset();
        state = STATE_ARMED;
      }
      break;

    case STATE_ARMED:
      if (!armReq) { state = STATE_DISARMED; break; }
      if (!linkOk) {
        state = STATE_FAILSAFE_LAND;
        failsafeStartMs = nowMs;
        failsafeThrottle = rc.cmd.throttle;
      }
      break;

    case STATE_FAILSAFE_LAND:
      if (linkOk) { state = armReq ? STATE_ARMED : STATE_DISARMED; break; }
      if (nowMs - failsafeStartMs > FAILSAFE_LAND_MS) state = STATE_DISARMED;
      break;

    default: break;
  }
}

// ---- Control + mixer -----------------------------------------------------------
static void flightControl(uint32_t nowMs) {
  float thr, rollSp, pitchSp, yawRateSp;

  if (state == STATE_ARMED) {
    thr       = rc.cmd.throttle / 1000.0f;
    rollSp    = constrain(rc.cmd.roll_cd  / 100.0f, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    pitchSp   = constrain(rc.cmd.pitch_cd / 100.0f, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
    yawRateSp = rc.cmd.yaw_rate_dds / 10.0f;
  } else if (state == STATE_FAILSAFE_LAND) {
    // Level out and ramp the last throttle down to zero
    float k = 1.0f - (float)(nowMs - failsafeStartMs) / FAILSAFE_LAND_MS;
    thr = (failsafeThrottle / 1000.0f) * constrain(k, 0.0f, 1.0f) * 0.85f;
    rollSp = pitchSp = yawRateSp = 0;
  } else {
    motorsOff();
    return;
  }

  // Outer loop: angle error -> desired rate
  float rollRateSp  = constrain(ANGLE_P * (rollSp  - imu.roll),  -MAX_RATE_DPS, MAX_RATE_DPS);
  float pitchRateSp = constrain(ANGLE_P * (pitchSp - imu.pitch), -MAX_RATE_DPS, MAX_RATE_DPS);

  // Inner loop: rate PID (integrators frozen on the ground)
  bool integ = thr > 0.12f;
  float r = pidRoll.update (rollRateSp,  imu.gx, LOOP_DT, integ);
  float p = pidPitch.update(pitchRateSp, imu.gy, LOOP_DT, integ);
  float y = pidYaw.update  (yawRateSp,   imu.gz, LOOP_DT, integ);

  // Battery sag compensation: same command = same thrust on every drone,
  // whether its pack is at 4.2 V or 3.5 V. Keeps the swarm moving together.
  float comp = constrain(4000.0f / max(vbatMv, 3000.0f), 0.95f, 1.25f);

  float base = MOTOR_IDLE + thr * (MOTOR_MAX - MOTOR_IDLE) * comp;
  //   M1 FR (CCW)   M2 RR (CW)   M3 RL (CCW)   M4 FL (CW)
  //   +roll = right side down, +pitch = nose down, +yaw = clockwise
  float m[4] = {
    base - r - p + y,
    base - r + p - y,
    base + r + p + y,
    base + r - p - y,
  };

  // Airmode-lite: if one motor saturates, shift all down so attitude
  // authority is kept instead of clipping.
  float hi = max(max(m[0], m[1]), max(m[2], m[3]));
  if (hi > MOTOR_MAX) for (float& v : m) v -= hi - MOTOR_MAX;

  for (uint8_t i = 0; i < 4; i++) motorWrite(i, (uint32_t)constrain(m[i], (float)MOTOR_IDLE, (float)MOTOR_MAX));
}

static void sendTelemetry(uint32_t nowUs) {
  Telemetry t = {};
  t.magic           = SWARM_MAGIC;
  t.node_id         = nodeId;
  t.state           = state;
  t.echo_seq        = rc.seq;
  t.echo_gs_time_us = rc.gsTimeUs;
  t.rx_age_us       = (uint16_t)min<uint32_t>(nowUs - rc.rxMicros, 65535);
  t.vbat_mv         = (uint16_t)vbatMv;
  t.roll_cd         = (int16_t)(imu.roll * 100);
  t.pitch_cd        = (int16_t)(imu.pitch * 100);
  t.loop_us         = worstLoopUs;
  t.lost_frames     = Radio::takeLostFrames();
  t.rssi            = rc.rssi;
  Radio::sendTelemetry(t);
  worstLoopUs = 0;
}

// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 4; i++) { motorAttach(i); }
  motorsOff();
  for (uint8_t pin : {PIN_LED_RED, PIN_LED_GREEN, PIN_HEADLIGHT, PIN_TAILLIGHT}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  prefs.begin("zaidfly", false);
  nodeId = prefs.getUChar("node", 0);

  if (!imu.begin(PIN_SDA, PIN_SCL)) {
    // No IMU = never fly. Blink red forever, still answer serial.
    while (true) {
      digitalWrite(PIN_LED_RED, (millis() / 100) % 2);
      handleSerial();
      delay(10);
    }
  }
  calibrate();

  if (!Radio::begin(nodeId)) {
    state = STATE_KILLED;
  }
  for (int i = 0; i < 20; i++) readBattery();
  Serial.printf("Zaid Fly node %u ready\n", nodeId);
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastBattMs = 0;

  // Fixed-rate scheduler: busy-wait to the next 1 ms tick (no jitter from delay())
  while ((int32_t)(micros() - nextUs) < 0) {}
  uint32_t startUs = micros();
  nextUs += 1000000UL / LOOP_HZ;
  if ((int32_t)(startUs - nextUs) > 5000) nextUs = startUs;  // recover after a stall

  uint32_t nowMs = millis();

  imu.update(LOOP_DT);

  bool fresh = Radio::latest(rc);
  bool linkOk = (nowMs - Radio::lastRxMillis()) < LINK_TIMEOUT_MS;

  updateState(nowMs, linkOk);
  flightControl(nowMs);

  // TDMA telemetry: answer only in our own slot so 20 drones never collide
  if (fresh && (rc.frameFlags & FRAME_TELEMETRY_REQ) &&
      (rc.seq % SWARM_MAX_NODES) == nodeId) {
    sendTelemetry(micros());
  }

  if (nowMs - lastBattMs >= 20) { lastBattMs = nowMs; readBattery(); handleSerial(); }
  setLeds(nowMs);

  uint16_t took = micros() - startUs;
  if (took > worstLoopUs) worstLoopUs = took;
}
