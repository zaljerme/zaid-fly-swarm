# Zaid Fly firmware

Two PlatformIO projects share one protocol header.

| Project | Runs on | Does |
|---|---|---|
| [`drone/`](drone/) | 20× XIAO ESP32-S3 on the Zaid Fly PCB | 1 kHz flight controller, ESP-NOW receiver, TDMA telemetry, failsafes |
| [`ground_station/`](ground_station/) | 1× XIAO ESP32-S3 on USB | USB ↔ ESP-NOW bridge with 200 Hz hardware-timed broadcast, E-STOP |
| [`common/swarm_protocol.h`](common/swarm_protocol.h) | both | Frame layout, flags, serial framing, CRC |

## Build and flash

```bash
pip install platformio
cd firmware/drone          && pio run -t upload
cd firmware/ground_station && pio run -t upload
```

The projects are pinned to `espressif32@6.9.0` (Arduino-ESP32 2.0.x). The code also has guards for the Arduino-ESP32 3.x API (LEDC and the ESP-NOW receive callback).

## Drone firmware layout

| File | |
|---|---|
| `src/config.h` | Pin map, PWM, PID gains, failsafe timings. **Tune here** |
| `src/imu.cpp` | MPU-6050 driver (±2000 dps, ±8 g, DLPF 42 Hz) + complementary filter that ignores the accelerometer under high g |
| `src/pid.h` | PID with derivative on measurement, D low-pass filter and clamped integrator |
| `src/radio.cpp` | ESP-NOW setup, swarm frame parsing, lost-frame counting, telemetry TX |
| `src/main.cpp` | Fixed-rate loop, safety state machine, angle→rate cascade, X-quad mixer, battery-sag compensation, LEDs, serial commands |

### State machine

```
BOOT → CALIBRATING → DISARMED ⇄ ARMED
                        ↑          │ link lost > 150 ms
                        └── FAILSAFE_LAND (level + ramp throttle to 0 over 2.5 s)
any state ── KILL flag / E-STOP / tilt > 70° ──► KILLED (until ARM is released)
```

### Serial commands (115200)

| Command | |
|---|---|
| `id N` | Set the swarm slot 0–19 (saved to NVS) and reboot |
| `cal` | Recalibrate the gyro and accelerometer (put the drone down level) |
| `info` | Node, state, battery, attitude, worst loop time, MAC |

### Tuning starting point

| Gain | Roll/Pitch | Yaw |
|---|---|---|
| Angle P | 6.0 | — |
| Rate P | 0.45 | 1.20 |
| Rate I | 0.80 | 0.60 |
| Rate D | 0.012 | 0 |

Tune one drone, then flash the same values to all 20. The printed frames are identical, so the gains carry over.
