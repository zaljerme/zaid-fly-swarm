// =============================================================================
//  Zaid Fly — swarm ground station
//
//  PC (swarm_controller.py) --USB--> this ESP32-S3 --ESP-NOW broadcast--> 20 drones
//
//  * Holds a table of 20 DroneCmd slots that the PC updates whenever it wants.
//  * Broadcasts the WHOLE table as one 172-byte frame every 5 ms (200 Hz),
//    driven by a hardware-timed loop — the PC's timing jitter never reaches
//    the drones.
//  * Every frame asks one drone (seq % 20) for telemetry -> 10 Hz per drone,
//    zero collisions. Round-trip time is measured from the echoed timestamp.
//  * PC goes quiet for > PC_TIMEOUT_MS -> stop transmitting, so every drone's
//    own failsafe levels out and lands it.
//  * E-STOP button (D1 to GND) -> FRAME_ESTOP_ALL, latched until reboot.
// =============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "swarm_protocol.h"

static constexpr uint8_t  PIN_ESTOP     = 2;     // D1, active low
static constexpr uint8_t  PIN_LED       = 21;    // XIAO S3 user LED (active low)
static constexpr uint32_t PC_TIMEOUT_MS = 300;
static constexpr uint32_t PERIOD_US     = 1000000UL / SWARM_TX_HZ;

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static SwarmFrame frame = {};
static uint32_t   lastPcMs = 0;
static bool       estopLatched = false;
static uint32_t   framesSent = 0;

// Telemetry ring buffer (filled in Wi-Fi task, drained in loop)
struct TelemetryRx { Telemetry t; uint32_t rxUs; };
static TelemetryRx   tq[32];
static volatile uint8_t tqHead = 0, tqTail = 0;

// -----------------------------------------------------------------------------
static void sendToPc(uint8_t type, const void* payload, uint8_t len) {
  uint8_t hdr[4] = {SERIAL_SYNC0, SERIAL_SYNC1, type, len};
  uint8_t buf[260];
  memcpy(buf, hdr, 4);
  memcpy(buf + 4, payload, len);
  buf[4 + len] = swarm_crc8(buf + 2, len + 2);
  Serial.write(buf, len + 5);
}

static void onTelemetry(const uint8_t* data, int len) {
  if (len != sizeof(Telemetry)) return;
  const Telemetry* t = (const Telemetry*)data;
  if (t->magic != SWARM_MAGIC || t->node_id >= SWARM_MAX_NODES) return;
  uint8_t next = (tqHead + 1) % 32;
  if (next == tqTail) return;            // full: drop, never block the Wi-Fi task
  tq[tqHead].t = *t;
  tq[tqHead].rxUs = micros();
  tqHead = next;
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t*, const uint8_t* d, int n) { onTelemetry(d, n); }
#else
static void onRecv(const uint8_t*, const uint8_t* d, int n) { onTelemetry(d, n); }
#endif

// ---- PC serial parser (state machine, never blocks) -----------------------------
static void handlePcPacket(uint8_t type, const uint8_t* p, uint8_t len) {
  lastPcMs = millis();
  switch (type) {
    case SER_SET_ALL:
      if (len == sizeof(frame.cmd)) memcpy(frame.cmd, p, len);
      break;
    case SER_SET_ONE:
      if (len == 1 + sizeof(DroneCmd) && p[0] < SWARM_MAX_NODES)
        memcpy(&frame.cmd[p[0]], p + 1, sizeof(DroneCmd));
      break;
    case SER_ESTOP:
      estopLatched = true;
      break;
    case SER_PING:
      sendToPc(SER_PONG, p, len);
      break;
  }
}

static void pollSerial() {
  static uint8_t st = 0, type = 0, len = 0, idx = 0;
  static uint8_t buf[256];
  while (Serial.available()) {
    uint8_t c = Serial.read();
    switch (st) {
      case 0: st = (c == SERIAL_SYNC0) ? 1 : 0; break;
      case 1: st = (c == SERIAL_SYNC1) ? 2 : (c == SERIAL_SYNC0 ? 1 : 0); break;
      case 2: type = c; st = 3; break;
      case 3: len = c; idx = 0; st = len ? 4 : 5; break;
      case 4: buf[idx++] = c; if (idx >= len) st = 5; break;
      case 5: {
        uint8_t chk[258];
        chk[0] = type; chk[1] = len;
        memcpy(chk + 2, buf, len);
        if (swarm_crc8(chk, len + 2) == c) handlePcPacket(type, buf, len);
        st = 0;
        break;
      }
    }
  }
}

// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(921600);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(SWARM_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_max_tx_power(80);                  // 20 dBm
  esp_now_init();
#if ESP_ARDUINO_VERSION_MAJOR < 3
  esp_wifi_config_espnow_rate(WIFI_IF_STA, SWARM_PHY_RATE);
#endif
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = SWARM_WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  frame.magic = SWARM_MAGIC;
  frame.version = SWARM_VERSION;
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastStatusMs = 0;

  pollSerial();
  if (digitalRead(PIN_ESTOP) == LOW) estopLatched = true;

  // Forward any telemetry to the PC with the measured round-trip time
  while (tqTail != tqHead) {
    struct { Telemetry t; uint32_t rtt_us; } __attribute__((packed)) out;
    out.t = tq[tqTail].t;
    out.rtt_us = tq[tqTail].rxUs - out.t.echo_gs_time_us;
    tqTail = (tqTail + 1) % 32;
    sendToPc(SER_TELEMETRY, &out, sizeof(out));
  }

  uint32_t nowMs = millis();
  if (nowMs - lastStatusMs >= 250) {
    lastStatusMs = nowMs;
    struct { uint32_t sent, age; } __attribute__((packed)) s = {framesSent, nowMs - lastPcMs};
    sendToPc(SER_STATUS, &s, sizeof(s));
  }

  // ---- 200 Hz broadcast tick ----
  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += PERIOD_US;
  if ((int32_t)(micros() - nextUs) > (int32_t)PERIOD_US) nextUs = micros() + PERIOD_US;

  bool pcAlive = lastPcMs && (nowMs - lastPcMs) < PC_TIMEOUT_MS;
  if (!pcAlive && !estopLatched) {
    digitalWrite(PIN_LED, HIGH);          // off: not transmitting -> drones failsafe-land
    return;
  }

  frame.seq++;
  frame.frame_flags = FRAME_TELEMETRY_REQ | (estopLatched ? FRAME_ESTOP_ALL : 0);
  frame.gs_time_us = micros();
  if (esp_now_send(BCAST, (const uint8_t*)&frame, sizeof(frame)) == ESP_OK) framesSent++;
  digitalWrite(PIN_LED, (frame.seq / 20) % 2 ? LOW : HIGH);   // ~5 Hz blink = live
}
