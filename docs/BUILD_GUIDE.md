# Zaid Fly build guide: from parts to a 20-drone swarm

This guide covers how I built the swarm, step by step. Building 20 of anything changes how you work, so each section ends with tips for doing it in batches.

---

## 1. Order everything

1. **PCBs**: open `hardware/kicad/zaid-fly.kicad_pro` in KiCad 8, route the board (see [`hardware/README.md`](../hardware/README.md)), run DRC, and export Gerbers and the drill file. Order **25 boards** (5 spares), 2-layer, **1.0 mm** thick to save weight, with ENIG finish so the QFN pads stay flat. Order a **stencil** too. It saves hours of placing paste by hand.
2. **Components**: use [`hardware/bom/zaid-fly-bom.csv`](../hardware/bom/zaid-fly-bom.csv). The *Qty for 20 drones* column is ready to paste into an LCSC or DigiKey cart. Add about 10% spares of every 0603 part, because you will lose some to the carpet.
3. **Mechanical**: 88 motors (720 coreless, half CW and half CCW, usually red/blue wires for CW and black/white for CCW), 100 props, 30 1S LiPos with JST-PH, 21 XIAO ESP32-S3 boards, and low-profile pin headers.

<img src="images/photo-components-kit.png" width="300">

*One drone's worth of parts, sorted and labeled before soldering.*

## 2. Solder the carrier boards

Solder in this order, from the smallest and hardest parts to the largest:

| Step | Parts | How |
|---|---|---|
| 1 | U1 MPU-6050 (QFN-24) | Stencil paste, place with tweezers, hot air at ~330 °C until it self-aligns. Check pins 23 and 24 (SCL and SDA) for bridges under a magnifier. |
| 2 | C5–C8 next to the IMU | Same paste pass. |
| 3 | Q1–Q4 SI2300, D1–D4 1N4148WS | **Check diode polarity.** The cathode band goes to VBAT. |
| 4 | R1–R15, C1–C4 | 10k parts go everywhere except R10–R15 (LED resistors). |
| 5 | D5–D10 LEDs | The cathode mark goes toward GND. White LEDs go at the front, red at the back. |
| 6 | Pin headers + XIAO | Use low-profile headers so the XIAO sits above the SMD parts. |
| 7 | J2 BAT link | Two short wires from J2 to the BAT+ and BAT− pads under the XIAO. |

**Smoke test before the motors go on:**

* Measure resistance VBAT→GND. It should **not** be a short (>1 kΩ).
* Power from a current-limited bench supply at 3.7 V and 100 mA limit. Idle draw should be about 40–60 mA.
* Flash the firmware and type `info` in the serial monitor. The roll and pitch values should change when you tilt the board.

**Batch tips for 20:** paste and place all 20 IMUs in one session, reflow them all, then inspect them all. Doing each step on every board before moving on is much faster than finishing boards one at a time. Write the drone number on each board with a marker as soon as it passes the smoke test.

## 3. Print the frames

| Setting | Value |
|---|---|
| Material | PLA (light) or PETG (survives crashes better) |
| Layer height | 0.12 mm |
| Walls | 3–4 |
| Infill | 30% gyroid |
| Supports | None for the frame. Print the cover upside down |
| Time | ~35 min per frame and ~10 min per cover. A full plate holds about 6 sets |

Files: [`3d-models/stl/zaid-fly-frame.stl`](../3d-models/stl/zaid-fly-frame.stl) and [`zaid-fly-cover.stl`](../3d-models/stl/zaid-fly-cover.stl).

<img src="images/render-frame.png" width="320"> <img src="images/render-cover.png" width="320">

## 4. Assemble

1. Press the motors into the arm rings. **Diagonals match:** front-right (M1) and rear-left (M3) are **CCW**, and front-left (M4) and rear-right (M2) are **CW**.
2. Solder the motor wires to the M1–M4 pads. The `+` pad is VBAT and the other pad is the MOSFET drain.
3. Mount the PCB in the frame with **FRONT** (the headlight edge) facing forward. Use a dot of hot glue or foam tape.
4. Put the battery under the frame with a rubber band or Velcro.
5. Fit the props: **A props on CW motors and B props on CCW motors**. Check that each prop blows air down.
6. Snap the cover on.

<img src="images/render-assembly.png" width="520">

## 5. Flash and give each drone its ID

All 20 drones run **the same binary**. The swarm ID is stored in flash, not in the code.

```bash
cd firmware/drone
pio run -t upload
pio device monitor        # then type:  id 7   (0..19)
```

Put a sticker with the ID on each cover. Type `info` to check everything:

```
Zaid Fly node 7 | state 2 | vbat 4112 mV | roll 0.3 pitch -0.2 | loop 412 us | mac 34:85:18:...
```

Flash the ground station:

```bash
cd firmware/ground_station
pio run -t upload
```

## 6. First flight: one drone

1. Props **off**. Run `python software/swarm_controller.py --port COM5 --mode gamepad`.
2. Press **D-pad** to select only this drone (its LEDs blink), press **START** to arm, and raise throttle a little. Then, by hand, tilt the drone and check that the motors on the low side speed up. If they slow down, the IMU axis or motor order is wrong.
3. Props **on**, outside or in a net. Take off gently. If it oscillates, lower `RATE_P_RP` and `RATE_D_RP` in `firmware/drone/src/config.h`. If it drifts, re-run `cal` on a level surface.

## 7. Bring up the swarm

1. Power all 20 drones with props off. Run `--mode lights`. The table should show **20/20 online** and the headlight and taillight wave should travel across the swarm in order. If it doesn't, check that drone's ID.
2. Run `--mode latency` for a minute and write the numbers into the README table.
3. Props on. Use `--mode gamepad` with **Y = whole swarm**. Arm and lift all 20 together at low throttle.
4. Keep a hand on the **E-STOP**.

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| Red LED blinking fast at boot | IMU not found. Reflow U1 and check the R5/R6 pull-ups. |
| Drone not in the table | Wrong ID (two drones share a slot) or wrong Wi-Fi channel. Type `info`. |
| Flips on takeoff | Motor order or direction is wrong, or props are swapped. Compare with the diagram in section 4. |
| One motor never spins | SI2300 not soldered or diode in backwards (VBAT short). Check Q*n*/D*n*. |
| High `lost` count | Move the ground station up high, use the U.FL antenna, or change `SWARM_WIFI_CHANNEL` away from busy Wi-Fi. |
| Brown-out resets at full throttle | Battery too small or worn out. Use a fresh 250 mAh+ pack with a high C rating. |
