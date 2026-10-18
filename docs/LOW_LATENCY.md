# How Zaid Fly gets low-latency control for 20 drones

The main requirement was that **the operator never feels lag, and all 20 drones respond together.** This page explains each design choice and where it is in the code.

## 1. One frame for the whole swarm

[`firmware/common/swarm_protocol.h`](../firmware/common/swarm_protocol.h)

```c
struct DroneCmd {            // 8 bytes
  int16_t  roll_cd, pitch_cd, yaw_rate_dds;
  uint16_t throttle : 10;    // 0..1000
  uint16_t flags    : 6;     // ARM, KILL, lights, calibrate, blink
};
struct SwarmFrame {          // 12 + 20*8 = 172 bytes  (< 250 B ESP-NOW max)
  uint16_t magic; uint8_t version, frame_flags;
  uint32_t seq, gs_time_us;
  DroneCmd cmd[20];
};
```

With unicast, the ground station would send 20 frames and wait for 20 ACKs. Each frame pays for preamble, inter-frame spacing, ACK and channel contention, and drone 19 gets its command several milliseconds after drone 0. With **one broadcast frame** there is a single transmission (~0.35 ms at 6 Mbps) and every drone gets its command at the same time, so formations stay synchronized without any clock sync.

<img src="images/diagram-broadcast-vs-unicast.png" width="800">

## 2. Never wait for a retry

Broadcast frames get no ACK and no retransmission. For real-time control that's what I want: a command that arrives 10 ms late is stale anyway. At **200 Hz** a dropped frame costs 5 ms, and the drone keeps using the previous command in the meantime. The drone counts gaps in `seq` and reports them as `lost`, so link quality stays visible.

## 3. Radio settings that remove hidden delays

| Setting | Where | Why |
|---|---|---|
| `esp_wifi_set_ps(WIFI_PS_NONE)` | drone + GS | The default modem sleep can hold frames for up to one DTIM interval. |
| Fixed channel `SWARM_WIFI_CHANNEL` | both | No scanning and no channel hopping. |
| `esp_wifi_config_espnow_rate(..., WIFI_PHY_RATE_6M)` | both | 172 B takes ~0.35 ms on air, compared with ~1.6 ms at the 1 Mbps default. |
| `esp_wifi_set_max_tx_power(80)` | GS | 20 dBm for margin across a room. |

## 4. Hardware-timed transmitter

The PC is not a real-time system: USB and the OS scheduler add jitter. So the **PC only updates a table**, and the ground station sends that table on its **own microsecond-timed 200 Hz loop** ([`ground_station/src/main.cpp`](../firmware/ground_station/src/main.cpp)). PC jitter changes *which* command goes out, but never *when* it goes out.

## 5. The drone side: copy fast, use within 1 ms

* The ESP-NOW receive callback runs in the Wi-Fi task. It only `memcpy`s the drone's own 8-byte slot into a spin-lock-protected buffer and records `micros()`.
* The flight controller runs a **fixed 1 kHz loop**, busy-waiting on `micros()` with no `delay()` jitter, and picks up the newest command on its next tick, ≤ 1 ms later.
* IMU read (400 kHz I²C, 14 bytes), complementary filter, cascaded angle→rate PID and mixer take about 0.45 ms.
* Motors run 20 kHz PWM, so a new duty cycle takes effect within 50 µs.

## 6. Telemetry that never gets in the way (TDMA)

If 20 drones replied whenever they wanted, they would collide with each other and with the next command frame. Instead:

```c
if (fresh && (rc.frameFlags & FRAME_TELEMETRY_REQ) && (rc.seq % 20) == nodeId)
    sendTelemetry();
```

Each frame "owns" one drone. That drone replies in the ~4.5 ms gap before the next frame, and only one radio talks at a time. Every drone reports at 10 Hz, which is enough to watch the batteries, attitude, RSSI, packet loss and loop time of all 20.

## 7. Measuring it

Each telemetry packet echoes `gs_time_us` from the frame it answers. The ground station subtracts it from the arrival time to get the **round-trip time**. The drone also reports `rx_age_us`, the time between receiving the frame and sending the reply, so you can estimate one-way latency:

```
one_way ≈ (RTT − rx_age) / 2
```

```bash
python software/swarm_controller.py --port COM5 --mode latency
```

## 8. Latency budget

<img src="images/diagram-latency-budget.png" width="800">

| Stage | Typical | Worst |
|---|---|---|
| PC loop + USB | 1 ms | 10 ms |
| Wait for TX tick (200 Hz) | 2.5 ms | 5 ms |
| Airtime | 0.35 ms | 0.35 ms |
| Wi-Fi task → callback | 0.1 ms | 0.2 ms |
| Wait for control tick (1 kHz) | 0.5 ms | 1 ms |
| IMU + PID + mix | 0.45 ms | 0.45 ms |
| PWM | 0.05 ms | 0.05 ms |
| **Total** | **~5 ms** | **~17 ms** |

Human perception of control lag starts at about 50 ms, so even the worst case has a 3× margin.

## Ideas for v2

* Raise the TX rate to 400 Hz (the frame is short enough, and airtime use would be ~15%).
* Move the gamepad onto the ground station (a USB-host joystick) to remove the PC from the control path completely.
* Add optical-flow or UWB position hold so formations hold their positions on their own instead of only tracking attitude together.
