// =============================================================================
//  Zaid Fly Swarm — shared radio protocol
//  Used by: firmware/drone, firmware/ground_station (and mirrored in
//  software/swarm_controller.py). Change it here and rebuild both sides.
// =============================================================================
//
//  Design goal: every drone in a 20-node swarm gets a fresh command with
//  no perceptible delay, and all drones get it at the *same* time.
//
//  How:
//   * ONE ESP-NOW broadcast frame carries the commands for ALL 20 drones.
//     One frame on air = one airtime slot = every drone updates together.
//     (20 unicast frames would take 20 slots + 20 ACK/retry cycles.)
//   * Broadcast = no ACK and no retries. A late packet is worse than a lost
//     packet; at 200 Hz the next one is only 5 ms away anyway.
//   * Fixed Wi-Fi channel, modem power-save OFF, fixed PHY rate.
//   * Telemetry uses a TDMA slot: drone N only answers the frame whose
//     sequence number % SWARM_MAX_NODES == N, so drones never talk over
//     each other or over the command stream.
// =============================================================================
#pragma once
#include <stdint.h>

#define SWARM_MAGIC          0x5A46   // "ZF"
#define SWARM_VERSION        1
#define SWARM_MAX_NODES      20
#define SWARM_WIFI_CHANNEL   6        // must match on every node
#define SWARM_TX_HZ          200      // ground station broadcast rate
// 6 Mbps OFDM: a 172-byte frame is ~0.3 ms on air (vs ~1.6 ms at the 1 Mbps
// default) while still being robust indoors. Use WIFI_PHY_RATE_1M_L for range.
#define SWARM_PHY_RATE       WIFI_PHY_RATE_6M

// ---- per-drone command flags -------------------------------------------------
enum : uint8_t {
  CMD_ARM        = 1 << 0,   // motors allowed to spin
  CMD_KILL       = 1 << 1,   // immediate motor cut (overrides everything)
  CMD_HEADLIGHT  = 1 << 2,
  CMD_TAILLIGHT  = 1 << 3,
  CMD_CALIBRATE  = 1 << 4,   // re-run gyro/accel calibration (disarmed only)
  CMD_BLINK      = 1 << 5,   // identify: blink status LEDs
};

// ---- frame-level flags ---------------------------------------------------------
enum : uint8_t {
  FRAME_TELEMETRY_REQ = 1 << 0,   // slot owner should reply with telemetry
  FRAME_ESTOP_ALL     = 1 << 1,   // global kill — every drone cuts motors
};

#pragma pack(push, 1)

// 8 bytes per drone. Angles in centi-degrees, yaw rate in deci-deg/s,
// throttle 0..1000.
struct DroneCmd {
  int16_t  roll_cd;       // +/- 3000 = +/- 30 deg
  int16_t  pitch_cd;
  int16_t  yaw_rate_dds;  // +/- 2000 = +/- 200 deg/s
  uint16_t throttle : 10; // 0..1000 (10 bits, low bits of the word)
  uint16_t flags    : 6;  // CMD_* bits 0..5 (high 6 bits)
};
static_assert(sizeof(DroneCmd) == 8, "DroneCmd must be 8 bytes");

// Broadcast frame: 12 + 20*8 = 172 bytes  (< 250 byte ESP-NOW limit)
struct SwarmFrame {
  uint16_t magic;         // SWARM_MAGIC
  uint8_t  version;       // SWARM_VERSION
  uint8_t  frame_flags;   // FRAME_* bits
  uint32_t seq;           // monotonically increasing
  uint32_t gs_time_us;    // ground station micros() at send — echoed back
  DroneCmd cmd[SWARM_MAX_NODES];
};
static_assert(sizeof(SwarmFrame) == 172, "unexpected SwarmFrame size");

// Unicast drone -> ground station, sent only in the drone's TDMA slot.
struct Telemetry {
  uint16_t magic;
  uint8_t  node_id;
  uint8_t  state;          // FlightState
  uint32_t echo_seq;       // seq of the frame that triggered this reply
  uint32_t echo_gs_time_us;// gs_time_us of that frame (for round-trip time)
  uint16_t rx_age_us;      // time between frame RX and this TX on the drone
  uint16_t vbat_mv;
  int16_t  roll_cd;
  int16_t  pitch_cd;
  uint16_t loop_us;        // worst control-loop time in last window
  uint16_t lost_frames;    // frames missed since last telemetry
  int8_t   rssi;
  uint8_t  reserved;
};
static_assert(sizeof(Telemetry) == 26, "unexpected Telemetry size");

#pragma pack(pop)

enum FlightState : uint8_t {
  STATE_BOOT = 0,
  STATE_CALIBRATING,
  STATE_DISARMED,
  STATE_ARMED,
  STATE_FAILSAFE_LAND,
  STATE_KILLED,
  STATE_LOW_BATTERY,
};

// ---- USB serial link: PC <-> ground station -----------------------------------
// PC -> GS packet:  0xA5 0x5A | type | len | payload[len] | crc8
// GS -> PC packet:  0xA5 0x5A | type | len | payload[len] | crc8
#define SERIAL_SYNC0 0xA5
#define SERIAL_SYNC1 0x5A

enum : uint8_t {
  SER_SET_ALL      = 0x01,  // payload: DroneCmd[20]  (160 bytes)
  SER_SET_ONE      = 0x02,  // payload: uint8 node, DroneCmd
  SER_ESTOP        = 0x03,  // payload: none — kill every drone NOW
  SER_PING         = 0x04,  // payload: uint32 — echoed back as SER_PONG
  SER_TELEMETRY    = 0x81,  // payload: Telemetry + uint32 rtt_us
  SER_PONG         = 0x84,
  SER_STATUS       = 0x85,  // payload: uint32 frames_sent, uint32 pc_age_ms
};

static inline uint8_t swarm_crc8(const uint8_t* d, uint16_t n) {
  uint8_t crc = 0;
  while (n--) {
    crc ^= *d++;
    for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}
