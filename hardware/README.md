# Zaid Fly carrier PCB (rev A)

<img src="../docs/images/pcb-top-render.png" width="320"> <img src="../docs/images/pcb-ratsnest-render.png" width="320">

28 × 38 mm, 2-layer, 1.0 mm FR-4. The XIAO ESP32-S3 sits on low-profile headers **above** the SMD parts, so the whole flight controller fits on a footprint not much bigger than the XIAO.

## Files

| Path | What |
|---|---|
| `kicad/zaid-fly.kicad_pro` | KiCad 8 project (open this) |
| `kicad/zaid-fly.kicad_sch` | Schematic. Every pin is tied to its net with a label |
| `kicad/zaid-fly.kicad_pcb` | Board: outline, all 45 footprints placed, nets assigned, GND pours on both layers |
| `kicad/ZaidFly.kicad_sym`, `kicad/ZaidFly.pretty/` | Project symbol and footprint libraries (registered in the local lib tables) |
| `kicad/NETLIST.md` | Readable net-by-net connection list |
| `bom/zaid-fly-bom.csv` | BOM with per-drone and ×20 quantities |
| `tools/generate_kicad.py` | **Source of truth**: netlist + placement → all files above + renders |

To change a part value, net or position, edit `generate_kicad.py` and run:

```bash
python hardware/tools/generate_kicad.py
```

It also runs a courtyard-overlap and board-edge check before writing anything.

## Circuit

| Block | Parts | Notes |
|---|---|---|
| MCU | U2 XIAO ESP32-S3 | Runs from the 1S LiPo through its BAT pads (J2 link) and built-in charger and LDO |
| IMU | U1 MPU-6050, C5–C8, R5–R7 | Decoupling per the datasheet (VDD 0.1 µF, REGOUT 0.1 µF, CPOUT 10 nF, VLOGIC 10 nF). AD0 pulled low → address 0x68 |
| Motor drivers ×4 | Q1–Q4 SI2300, D1–D4 1N4148WS, C1–C4 0.1 µF, R1–R4 10k | Low-side switch. The flyback diode and cap sit right across each motor. The 10k pull-down keeps the gates off during boot |
| Battery sense | R8/R9 10k/10k | VBAT ÷ 2 → GPIO9 (ADC1) |
| Lights | D5–D10 + R10–R15 | Status red and green, 2× white headlights, 2× red taillights |

## Routing notes

The board ships placed with nets assigned. Route it in KiCad, or run Freerouting through the KiCad plugin and then tidy up by hand:

* **Power** net class (already in the project): VBAT, GND, MOT1-…MOT4- at **0.6 mm**. Each motor pulls about 1 A peak.
* Keep each motor loop (VBAT → motor → MOSFET → GND) short and right next to its diode and cap.
* Run I²C (SDA and SCL) under the XIAO, away from the motor drain traces.
* Stitch the top and bottom GND pours with vias around the IMU, then press **B** to refill zones before DRC.
* Signals: 0.2 mm track and 0.15 mm clearance. Vias are 0.6/0.3 mm, which every common board house can make.

## Fab settings

2 layers · 1.0 mm · ENIG · green mask, white silk · order a stencil · 25 pcs.
