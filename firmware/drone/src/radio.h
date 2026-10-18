// =============================================================================
//  ESP-NOW receiver for the swarm broadcast frame.
//  The receive callback runs in the Wi-Fi task: it only copies this drone's
//  8-byte slot + timestamps into a double buffer. The 1 kHz control loop
//  picks it up on its next iteration (<= 1 ms later).
// =============================================================================
#pragma once
#include <Arduino.h>
#include "swarm_protocol.h"

struct RadioCmd {
  DroneCmd cmd;
  uint8_t  frameFlags;
  uint32_t seq;
  uint32_t gsTimeUs;
  uint32_t rxMicros;   // local micros() when the frame arrived
  int8_t   rssi;
};

namespace Radio {
  bool begin(uint8_t nodeId);
  // Returns true if a newer frame than the last call has arrived.
  bool latest(RadioCmd& out);
  uint32_t lastRxMillis();
  uint16_t takeLostFrames();
  void sendTelemetry(const Telemetry& t);
}
