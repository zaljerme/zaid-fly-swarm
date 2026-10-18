#!/usr/bin/env python3
"""
Zaid Fly carrier PCB — single source of truth.

Defines the netlist + component placement once and generates:
  hardware/kicad/zaid-fly.kicad_pro / .kicad_sch / .kicad_pcb
  hardware/kicad/ZaidFly.kicad_sym + ZaidFly.pretty/*.kicad_mod (+ lib tables)
  hardware/bom/zaid-fly-bom.csv  and  hardware/kicad/NETLIST.md
  docs/images/pcb-top-render.png, pcb-netlist-render.png

Run:  python hardware/tools/generate_kicad.py
Then open hardware/kicad/zaid-fly.kicad_pro in KiCad 8, route the board
(or run Freerouting), refill zones (B) and export Gerbers.
"""
import csv
import math
import os
import uuid as _uuid

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
KDIR = os.path.join(ROOT, "hardware", "kicad")
IMG = os.path.join(ROOT, "docs", "images")
PROJECT = "zaid-fly"
NS = _uuid.UUID("5a1d0f1e-0000-4000-8000-5a1df1e00000")


def uid(*key):
    """Deterministic UUIDs -> regenerating gives a clean git diff."""
    return str(_uuid.uuid5(NS, "/".join(map(str, key))))


# =============================================================================
#  Footprints (pad name -> (x, y, w, h, shape, drill)) in footprint-local mm
# =============================================================================
def two_pad(pitch, w, h):
    return {"1": (-pitch / 2, 0, w, h, "roundrect", None), "2": (pitch / 2, 0, w, h, "roundrect", None)}


def qfn24():
    pads, p, e = {}, 0.5, 1.95
    for i in range(6):
        o = -1.25 + i * p
        pads[str(1 + i)] = (-e, o, 0.8, 0.28, "roundrect", None)
        pads[str(7 + i)] = (o, e, 0.28, 0.8, "roundrect", None)
        pads[str(13 + i)] = (e, -o, 0.8, 0.28, "roundrect", None)
        pads[str(19 + i)] = (-o, -e, 0.28, 0.8, "roundrect", None)
    pads["25"] = (0, 0, 2.6, 2.6, "roundrect", None)
    return pads


def xiao():
    pads = {}
    for i in range(7):
        pads[str(1 + i)] = (-7.62, -7.62 + i * 2.54, 2.6, 1.6, "oval", 1.0)
        pads[str(8 + i)] = (7.62, 7.62 - i * 2.54, 2.6, 1.6, "oval", 1.0)
    return pads


FOOTPRINTS = {
    "R_0603":   {"pads": two_pad(1.55, 0.9, 0.95), "body": (1.6, 0.8), "smd": True},
    "C_0603":   {"pads": two_pad(1.55, 0.9, 0.95), "body": (1.6, 0.8), "smd": True},
    "LED_0603": {"pads": two_pad(1.55, 0.9, 0.95), "body": (1.6, 0.8), "smd": True},
    "D_SOD-323": {"pads": {"1": (-1.15, 0, 0.7, 0.55, "roundrect", None),
                           "2": (1.15, 0, 0.7, 0.55, "roundrect", None)}, "body": (1.8, 1.3), "smd": True},
    "SOT-23":   {"pads": {"1": (-0.95, 1.05, 0.6, 0.95, "roundrect", None),
                          "2": (0.95, 1.05, 0.6, 0.95, "roundrect", None),
                          "3": (0, -1.05, 0.6, 0.95, "roundrect", None)}, "body": (2.9, 1.3), "smd": True},
    "QFN-24_4x4mm_P0.5mm": {"pads": qfn24(), "body": (4.0, 4.0), "smd": True},
    "XIAO_ESP32S3_THT": {"pads": xiao(), "body": (17.8, 21.0), "smd": False},
    "Pad_2x_1.7mm": {"pads": {"1": (0, -1.5, 1.7, 1.7, "circle", 0.9),
                              "2": (0, 1.5, 1.7, 1.7, "circle", 0.9)}, "body": (2.2, 5.0), "smd": False},
    "Pad_2x_1.7mm_H": {"pads": {"1": (-1.5, 0, 1.7, 1.7, "circle", 0.9),
                                "2": (1.5, 0, 1.7, 1.7, "circle", 0.9)}, "body": (5.0, 2.2), "smd": False},
}

# =============================================================================
#  Netlist: ref -> (symbol, value, footprint, {pad: net}, lcsc/part, x, y, rot)
#  Board origin (100,100) top-left, 28 x 38 mm, FRONT of the drone = top edge.
# =============================================================================
BX, BY, BW, BH, BR = 100.0, 100.0, 28.0, 38.0, 3.0
CX, CY = BX + BW / 2, 119.0          # XIAO centre

PARTS = {}


def part(ref, sym, value, fp, pins, mpn, x, y, rot=0):
    PARTS[ref] = dict(sym=sym, value=value, fp=fp, pins=pins, mpn=mpn, x=x, y=y, rot=rot)


# ---- MCU + IMU ---------------------------------------------------------------
part("U2", "XIAO_ESP32S3", "Seeed XIAO ESP32-S3", "XIAO_ESP32S3_THT",
     {"1": "GATE1", "2": "GATE2", "3": "GATE3", "4": "GATE4", "5": "SDA", "6": "SCL",
      "7": "LED_R", "8": "LED_G", "9": "HEAD", "10": "TAIL", "11": "VBAT_SENSE",
      "12": "+3V3", "13": "GND", "14": None},
     "Seeed 113991114", CX, CY)
part("U1", "MPU6050", "MPU-6050", "QFN-24_4x4mm_P0.5mm",
     {"1": "GND", "8": "+3V3", "9": "MPU_AD0", "10": "MPU_REGOUT", "11": "GND", "13": "+3V3",
      "18": "GND", "20": "MPU_CPOUT", "23": "SCL", "24": "SDA", "25": "GND",
      **{str(n): None for n in (2, 3, 4, 5, 6, 7, 12, 14, 15, 16, 17, 19, 21, 22)}},
     "TDK InvenSense MPU-6050 (LCSC C24112)", CX, CY + 2.5)
part("C5", "C", "0.1uF", "C_0603", {"1": "+3V3", "2": "GND"}, "0603 X7R 16V (C14663)", CX + 3.6, CY + 2.5, 90)
part("C6", "C", "0.1uF", "C_0603", {"1": "MPU_REGOUT", "2": "GND"}, "0603 X7R 16V (C14663)", CX + 1.2, CY + 6.0, 0)
part("C7", "C", "0.01uF", "C_0603", {"1": "MPU_CPOUT", "2": "GND"}, "0603 X7R 50V (C57112)", CX + 1.6, CY - 1.3, 0)
part("C8", "C", "0.01uF", "C_0603", {"1": "+3V3", "2": "GND"}, "0603 X7R 50V (C57112)", CX - 2.0, CY + 6.0, 0)
part("R5", "R", "10k", "R_0603", {"1": "+3V3", "2": "SDA"}, "0603 1% (C25804)", CX - 3.9, CY + 0.3, 90)
part("R6", "R", "10k", "R_0603", {"1": "+3V3", "2": "SCL"}, "0603 1% (C25804)", CX - 3.9, CY + 3.3, 90)
part("R7", "R", "10k", "R_0603", {"1": "MPU_AD0", "2": "GND"}, "0603 1% (C25804)", CX - 2.0, CY - 1.3, 0)
part("R8", "R", "10k", "R_0603", {"1": "VBAT", "2": "VBAT_SENSE"}, "0603 1% (C25804)", CX + 2.6, CY - 5.6, 90)
part("R9", "R", "10k", "R_0603", {"1": "VBAT_SENSE", "2": "GND"}, "0603 1% (C25804)", CX + 4.3, CY - 5.6, 90)
part("J2", "Conn_01x02", "XIAO BAT", "Pad_2x_1.7mm_H", {"1": "VBAT", "2": "GND"},
     "wire link to XIAO BAT+/BAT- pads", CX - 2.0, CY - 5.6, 0)

# ---- Battery input -------------------------------------------------------------
part("J1", "Conn_01x02", "1S LiPo", "Pad_2x_1.7mm_H", {"1": "VBAT", "2": "GND"},
     "JST-PH 2.0 pigtail, 1S 3.7V", CX, BY + 2.4, 0)


# ---- 4x brushed motor drivers (mirrored clusters) ----------------------------
#   M1 front-right  M2 rear-right  M3 rear-left  M4 front-left
def driver(n, sx, sy):
    """sx = +1 right side / -1 left side, sy = -1 front (top) / +1 rear (bottom)."""
    ex = CX + sx * (BW / 2 - 2.5)                       # motor pads near the corner
    ey = (BY + 5.0) if sy < 0 else (BY + BH - 5.0)
    part(f"M{n}", "Conn_01x02", "Motor", "Pad_2x_1.7mm",
         {"1": "VBAT", "2": f"MOT{n}-"} if sy < 0 else {"1": f"MOT{n}-", "2": "VBAT"},
         "720 coreless brushed motor (7x20 mm)", ex, ey)
    qx = CX + sx * 7.6
    part(f"Q{n}", "Q_NMOS_GSD", "SI2300", "SOT-23",
         {"1": f"GATE{n}", "2": "GND", "3": f"MOT{n}-"}, "SI2300 N-MOSFET (C10487)",
         qx, ey + (-0.6 if sy < 0 else 0.6), 0 if sy < 0 else 180)
    part(f"D{n}", "D", "1N4148WS", "D_SOD-323", {"1": "VBAT", "2": f"MOT{n}-"},
         "1N4148WS flyback (C2128)", qx, ey + (2.9 if sy < 0 else -2.9), 0)
    part(f"C{n}", "C", "0.1uF", "C_0603", {"1": "VBAT", "2": f"MOT{n}-"},
         "0603 X7R 16V (C14663)", CX + sx * 4.3, ey + 1.6 * sy, 90)
    part(f"R{n}", "R", "10k", "R_0603", {"1": f"GATE{n}", "2": "GND"},
         "0603 1% gate pull-down (C25804)", CX + sx * 4.3, ey - 1.6 * sy, 90)


driver(1, +1, -1)
driver(4, -1, -1)
driver(2, +1, +1)
driver(3, -1, +1)


# ---- LEDs ---------------------------------------------------------------------
def led(dref, rref, color, ohms, net, x, y, ry, mpn):
    part(dref, "LED", color, "LED_0603", {"1": "GND", "2": f"{dref}_A"}, mpn, x, y, 0)
    part(rref, "R", ohms, "R_0603", {"1": net, "2": f"{dref}_A"}, "0603 1%", x, ry, 0)


led("D7", "R12", "White", "100", "HEAD", CX - 2.4, BY + 4.6, BY + 6.4, "0603 white LED (C2290)")
led("D8", "R13", "White", "100", "HEAD", CX + 2.4, BY + 4.6, BY + 6.4, "0603 white LED (C2290)")
led("D9", "R14", "Red", "150", "TAIL", CX - 2.4, BY + BH - 1.6, BY + BH - 3.4, "0603 red LED (C2286)")
led("D10", "R15", "Red", "150", "TAIL", CX + 2.4, BY + BH - 1.6, BY + BH - 3.4, "0603 red LED (C2286)")
led("D5", "R10", "Red", "200", "LED_R", CX - 2.4, BY + BH - 5.4, BY + BH - 7.2, "0603 red LED (C2286)")
led("D6", "R11", "Green", "200", "LED_G", CX + 2.4, BY + BH - 5.4, BY + BH - 7.2, "0603 green LED (C72043)")

NETS = sorted({n for p in PARTS.values() for n in p["pins"].values() if n})
NETS = ["GND", "VBAT", "+3V3"] + [n for n in NETS if n not in ("GND", "VBAT", "+3V3")]
NET_ID = {n: i + 1 for i, n in enumerate(NETS)}

# =============================================================================
#  Schematic symbols: name -> (body rect (x1,y1,x2,y2), pins [(num, name, x, y, angle, type)])
#  Library coordinates (Y up), 2.54 mm grid.
# =============================================================================
SYMBOLS = {
    "R": ((-1.016, 2.54, 1.016, -2.54), [("1", "~", 0, 3.81, 270, "passive"), ("2", "~", 0, -3.81, 90, "passive")]),
    "C": ((-2.032, 0.762, 2.032, -0.762), [("1", "~", 0, 3.81, 270, "passive"), ("2", "~", 0, -3.81, 90, "passive")]),
    "LED": ((-1.27, 1.27, 1.27, -1.27), [("1", "K", -3.81, 0, 0, "passive"), ("2", "A", 3.81, 0, 180, "passive")]),
    "D": ((-1.27, 1.27, 1.27, -1.27), [("1", "K", -3.81, 0, 0, "passive"), ("2", "A", 3.81, 0, 180, "passive")]),
    "Q_NMOS_GSD": ((-1.27, 2.54, 2.54, -2.54), [("1", "G", -5.08, 0, 0, "input"),
                                                ("2", "S", 2.54, -5.08, 90, "passive"),
                                                ("3", "D", 2.54, 5.08, 270, "passive")]),
    "Conn_01x02": ((-1.27, 1.27, 1.27, -3.81), [("1", "Pin_1", -5.08, 0, 0, "passive"),
                                                ("2", "Pin_2", -5.08, -2.54, 0, "passive")]),
}
_xiao_names = ["D0/GPIO1", "D1/GPIO2", "D2/GPIO3", "D3/GPIO4", "D4/SDA/GPIO5", "D5/SCL/GPIO6",
               "D6/TX/GPIO43", "D7/RX/GPIO44", "D8/GPIO7", "D9/GPIO8", "D10/GPIO9", "3V3", "GND", "5V"]
SYMBOLS["XIAO_ESP32S3"] = ((-12.7, 10.16, 12.7, -10.16),
                           [(str(i + 1), _xiao_names[i], -17.78, 7.62 - i * 2.54, 0, "bidirectional") for i in range(7)] +
                           [(str(i + 8), _xiao_names[i + 7], 17.78, -7.62 + i * 2.54, 180,
                             "power_in" if i >= 4 else "bidirectional") for i in range(7)])
_mpu_names = {1: "CLKIN", 6: "AUX_DA", 7: "AUX_CL", 8: "VLOGIC", 9: "AD0", 10: "REGOUT", 11: "FSYNC", 12: "INT",
              13: "VDD", 18: "GND", 20: "CPOUT", 23: "SCL", 24: "SDA", 25: "EP"}
_mpu_left = [1, 8, 9, 10, 11, 13, 18, 20, 25]
_mpu_right = [24, 23, 12, 6, 7] + [2, 3, 4, 5, 14, 15, 16, 17, 19, 21, 22]
SYMBOLS["MPU6050"] = ((-10.16, 13.97, 10.16, -27.94),
                      [(str(n), _mpu_names.get(n, "NC"), -15.24, 12.7 - i * 2.54, 0,
                        "power_in" if n in (8, 13, 18) else "passive") for i, n in enumerate(_mpu_left)] +
                      [(str(n), _mpu_names.get(n, "NC"), 15.24, 12.7 - i * 2.54, 180,
                        "no_connect" if n not in _mpu_names else "passive") for i, n in enumerate(_mpu_right)])

SYM_INFO = {"R": ("R", "Resistor"), "C": ("C", "Capacitor"), "LED": ("D", "Light emitting diode"),
            "D": ("D", "Diode"), "Q_NMOS_GSD": ("Q", "N-channel MOSFET, G-S-D pinout (SOT-23)"),
            "Conn_01x02": ("J", "2-pin pads"), "XIAO_ESP32S3": ("U", "Seeed Studio XIAO ESP32-S3"),
            "MPU6050": ("U", "6-axis IMU, I2C")}


def q(s):
    return '"' + str(s).replace('"', '\\"') + '"'


def fmt(v):
    return ("%.4f" % v).rstrip("0").rstrip(".")


def symbol_def(name, libname):
    rect, pins = SYMBOLS[name]
    refp, desc = SYM_INFO[name]
    hide_names = name in ("R", "C")
    out = [f'(symbol {q(libname)}']
    if hide_names:
        out.append('  (pin_numbers hide) (pin_names (offset 0) hide)')
    else:
        out.append('  (pin_names (offset 1.016))')
    out.append('  (exclude_from_sim no) (in_bom yes) (on_board yes)')
    out.append(f'  (property "Reference" {q(refp)} (at {fmt(rect[2] + 1.5)} {fmt(rect[1])} 0) (effects (font (size 1.27 1.27)) (justify left)))')
    out.append(f'  (property "Value" {q(name)} (at {fmt(rect[2] + 1.5)} {fmt(rect[1] - 2.54)} 0) (effects (font (size 1.27 1.27)) (justify left)))')
    out.append('  (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    out.append('  (property "Datasheet" "~" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    out.append(f'  (property "Description" {q(desc)} (at 0 0 0) (effects (font (size 1.27 1.27)) hide))')
    out.append(f'  (symbol "{name}_0_1"')
    out.append(f'    (rectangle (start {fmt(rect[0])} {fmt(rect[1])}) (end {fmt(rect[2])} {fmt(rect[3])}) '
               f'(stroke (width 0.254) (type default)) (fill (type background)))')
    if name in ("LED", "D"):
        out.append('    (polyline (pts (xy -1.27 1.27) (xy -1.27 -1.27)) (stroke (width 0.254) (type default)) (fill (type none)))')
        out.append('    (polyline (pts (xy 1.27 1.27) (xy -1.27 0) (xy 1.27 -1.27) (xy 1.27 1.27)) (stroke (width 0.254) (type default)) (fill (type outline)))')
    out.append('  )')
    out.append(f'  (symbol "{name}_1_1"')
    for num, pname, x, y, ang, typ in pins:
        length = 3.81 if name in ("XIAO_ESP32S3", "MPU6050") else (1.27 if name in ("R", "C") else 2.54)
        out.append(f'    (pin {typ} line (at {fmt(x)} {fmt(y)} {ang}) (length {fmt(length)}) '
                   f'(name {q(pname)} (effects (font (size 1.016 1.016)))) '
                   f'(number {q(num)} (effects (font (size 1.016 1.016)))))')
    out.append('  )')
    out.append(')')
    return "\n".join(out)


# =============================================================================
#  Schematic
# =============================================================================
SCH_LAYOUT = {   # ref -> (x, y) on an A3 sheet, mm (multiples of 2.54)
    "U2": (101.6, 76.2), "U1": (195.58, 76.2),
    "C5": (149.86, 132.08), "C6": (160.02, 132.08), "C7": (170.18, 132.08), "C8": (180.34, 132.08),
    "R5": (243.84, 50.8), "R6": (254.0, 50.8), "R7": (264.16, 50.8),
    "R8": (243.84, 88.9), "R9": (254.0, 88.9), "J1": (40.64, 40.64), "J2": (40.64, 60.96),
}
for i, n in enumerate((1, 2, 3, 4)):
    x0 = 40.64 + i * 66.04
    SCH_LAYOUT[f"M{n}"] = (x0 + 33.02, 170.18)
    SCH_LAYOUT[f"Q{n}"] = (x0 + 15.24, 198.12)
    SCH_LAYOUT[f"D{n}"] = (x0 + 30.48, 187.96)
    SCH_LAYOUT[f"C{n}"] = (x0 + 45.72, 187.96)
    SCH_LAYOUT[f"R{n}"] = (x0 + 2.54, 208.28)
for i, (d, r) in enumerate((("D5", "R10"), ("D6", "R11"), ("D7", "R12"), ("D8", "R13"), ("D9", "R14"), ("D10", "R15"))):
    x0 = 304.8 + (i % 2) * 40.64
    y0 = 127.0 + (i // 2) * 30.48
    SCH_LAYOUT[r] = (x0, y0)
    SCH_LAYOUT[d] = (x0 + 15.24, y0 + 12.7)


def build_schematic():
    root = uid("sch-root")
    out = ['(kicad_sch (version 20231120) (generator "eeschema") (generator_version "8.0")',
           f'  (uuid {q(root)})', '  (paper "A3")',
           '  (title_block (title "Zaid Fly — swarm drone flight controller carrier") (date "2026-10-18") '
           '(rev "A") (company "Zaid Fly") (comment 1 "XIAO ESP32-S3 + MPU6050 + 4x SI2300 brushed motor drivers") '
           '(comment 2 "Generated by hardware/tools/generate_kicad.py — nets are connected by labels"))',
           '  (lib_symbols']
    for name in SYMBOLS:
        out.append("    " + symbol_def(name, f"ZaidFly:{name}").replace("\n", "\n    "))
    out.append('  )')

    for ref, p in sorted(PARTS.items()):
        sx, sy = SCH_LAYOUT[ref]
        su = uid("sym", ref)
        rect = SYMBOLS[p["sym"]][0]
        out.append(f'  (symbol (lib_id {q("ZaidFly:" + p["sym"])}) (at {fmt(sx)} {fmt(sy)} 0) (unit 1) '
                   f'(exclude_from_sim no) (in_bom yes) (on_board yes) (dnp no) (fields_autoplaced yes)')
        out.append(f'    (uuid {q(su)})')
        out.append(f'    (property "Reference" {q(ref)} (at {fmt(sx + rect[2] + 1.5)} {fmt(sy - rect[1])} 0) '
                   f'(effects (font (size 1.27 1.27)) (justify left)))')
        out.append(f'    (property "Value" {q(p["value"])} (at {fmt(sx + rect[2] + 1.5)} {fmt(sy - rect[1] + 2.54)} 0) '
                   f'(effects (font (size 1.27 1.27)) (justify left)))')
        out.append(f'    (property "Footprint" {q("ZaidFly:" + p["fp"])} (at {fmt(sx)} {fmt(sy)} 0) '
                   f'(effects (font (size 1.27 1.27)) hide))')
        out.append(f'    (property "Datasheet" "~" (at {fmt(sx)} {fmt(sy)} 0) (effects (font (size 1.27 1.27)) hide))')
        out.append(f'    (property "Description" {q(SYM_INFO[p["sym"]][1])} (at {fmt(sx)} {fmt(sy)} 0) '
                   f'(effects (font (size 1.27 1.27)) hide))')
        out.append(f'    (property "MPN" {q(p["mpn"])} (at {fmt(sx)} {fmt(sy)} 0) (effects (font (size 1.27 1.27)) hide))')
        for num, *_ in SYMBOLS[p["sym"]][1]:
            out.append(f'    (pin {q(num)} (uuid {q(uid("pin", ref, num))}))')
        out.append(f'    (instances (project {q(PROJECT)} (path {q("/" + root)} (reference {q(ref)}) (unit 1))))')
        out.append('  )')

        # a label (or no-connect flag) on every pin end
        for num, pname, px, py, ang, typ in SYMBOLS[p["sym"]][1]:
            x, y = sx + px, sy - py
            net = p["pins"].get(num)
            if net is None:
                out.append(f'  (no_connect (at {fmt(x)} {fmt(y)}) (uuid {q(uid("nc", ref, num))}))')
                continue
            la = (ang + 180) % 360
            just = "left" if la in (0, 90) else "right"
            out.append(f'  (label {q(net)} (at {fmt(x)} {fmt(y)} {la}) (fields_autoplaced yes) '
                       f'(effects (font (size 1.27 1.27)) (justify {just} bottom)) (uuid {q(uid("lbl", ref, num))}))')

    notes = [
        (25.4, 15.24, "ZAID FLY — 20-node ESP-NOW swarm drone  |  rev A"),
        (25.4, 22.86, "Battery: 1S LiPo -> VBAT (motors) and XIAO BAT+ pads (MCU, via XIAO LDO/charger)"),
        (40.64, 160.02, "Brushed motor drivers: SI2300 low-side switch, 1N4148WS flyback + 0.1uF across each motor, 10k gate pull-down"),
        (304.8, 116.84, "LEDs: status (D5 red, D6 green), headlights (D7/D8 white), taillights (D9/D10 red)"),
        (149.86, 124.46, "MPU6050 decoupling per datasheet: VDD 0.1uF, REGOUT 0.1uF, CPOUT 10nF, VLOGIC 10nF"),
    ]
    for x, y, t in notes:
        out.append(f'  (text {q(t)} (exclude_from_sim no) (at {fmt(x)} {fmt(y)} 0) '
                   f'(effects (font (size 1.778 1.778)) (justify left bottom)) (uuid {q(uid("txt", t))}))')
    out.append('  (sheet_instances (path "/" (page "1")))')
    out.append(')')
    return "\n".join(out) + "\n"


# =============================================================================
#  PCB
# =============================================================================
def rot(x, y, deg):
    a = math.radians(deg)
    # KiCad: positive angle = counter-clockwise on screen (Y down)
    return x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)


def pad_sexpr(name, pd, fp_rot, net, smd_layers, with_net=True, puid=None):
    x, y, w, h, shape, drill = pd
    angle = f" {fmt(fp_rot)}" if fp_rot else ""
    s = f'(pad {q(name)} '
    if drill:
        s += f'thru_hole {shape} (at {fmt(x)} {fmt(y)}{angle}) (size {fmt(w)} {fmt(h)}) (drill {fmt(drill)}) (layers "*.Cu" "*.Mask")'
    else:
        s += f'smd {shape} (at {fmt(x)} {fmt(y)}{angle}) (size {fmt(w)} {fmt(h)}) (layers {smd_layers})'
        if shape == "roundrect":
            s += ' (roundrect_rratio 0.25)'
    if with_net and net:
        s += f' (net {NET_ID[net]} {q(net)})'
    if puid:
        s += f' (uuid {q(puid)})'
    return s + ')'


def fp_graphics(fpname, layer_prefix="F"):
    bw, bh = FOOTPRINTS[fpname]["body"]
    g = []
    hx, hy = bw / 2, bh / 2
    if fpname == "XIAO_ESP32S3_THT":
        # module outline + USB-C at the front
        g.append(f'(fp_rect (start {-hx} {-hy - 0.25}) (end {hx} {hy - 0.25}) (stroke (width 0.12) (type solid)) (fill none) (layer "{layer_prefix}.SilkS"))')
        g.append(f'(fp_rect (start -4.5 {-hy - 1.6}) (end 4.5 {-hy + 4.4}) (stroke (width 0.12) (type solid)) (fill none) (layer "{layer_prefix}.SilkS"))')
        g.append(f'(fp_text user "USB-C" (at 0 {-hy + 1.4} 0) (layer "{layer_prefix}.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))')
    elif fpname not in ("Pad_2x_1.7mm", "Pad_2x_1.7mm_H"):
        g.append(f'(fp_rect (start {fmt(-hx)} {fmt(-hy)}) (end {fmt(hx)} {fmt(hy)}) (stroke (width 0.1) (type solid)) (fill none) (layer "{layer_prefix}.Fab"))')
    if fpname.startswith("QFN"):
        g.append(f'(fp_circle (center -2.6 -2.6) (end -2.45 -2.6) (stroke (width 0.25) (type solid)) (fill solid) (layer "{layer_prefix}.SilkS"))')
    if fpname in ("LED_0603", "D_SOD-323"):
        x = -FOOTPRINTS[fpname]["pads"]["1"][0] * -1 - 0.75
        g.append(f'(fp_line (start {fmt(x)} -0.6) (end {fmt(x)} 0.6) (stroke (width 0.15) (type solid)) (layer "{layer_prefix}.SilkS"))')
    cy = max(hy, max(abs(p[1]) + p[3] / 2 for p in FOOTPRINTS[fpname]["pads"].values())) + 0.25
    cx = max(hx, max(abs(p[0]) + p[2] / 2 for p in FOOTPRINTS[fpname]["pads"].values())) + 0.25
    g.append(f'(fp_rect (start {fmt(-cx)} {fmt(-cy)}) (end {fmt(cx)} {fmt(cy)}) (stroke (width 0.05) (type solid)) (fill none) (layer "{layer_prefix}.CrtYd"))')
    return g


SMD_F = '"F.Cu" "F.Paste" "F.Mask"'


def build_pcb():
    out = ['(kicad_pcb (version 20240108) (generator "pcbnew") (generator_version "8.0")',
           '  (general (thickness 1.0) (legacy_teardrops no))', '  (paper "A4")',
           '  (title_block (title "Zaid Fly carrier PCB") (date "2026-10-18") (rev "A") (company "Zaid Fly"))',
           '  (layers (0 "F.Cu" signal) (31 "B.Cu" signal) (32 "B.Adhes" user "B.Adhesive") (33 "F.Adhes" user "F.Adhesive")',
           '    (34 "B.Paste" user) (35 "F.Paste" user) (36 "B.SilkS" user "B.Silkscreen") (37 "F.SilkS" user "F.Silkscreen")',
           '    (38 "B.Mask" user) (39 "F.Mask" user) (40 "Dwgs.User" user "User.Drawings") (41 "Cmts.User" user "User.Comments")',
           '    (42 "Eco1.User" user "User.Eco1") (43 "Eco2.User" user "User.Eco2") (44 "Edge.Cuts" user) (45 "Margin" user)',
           '    (46 "B.CrtYd" user "B.Courtyard") (47 "F.CrtYd" user "F.Courtyard") (48 "B.Fab" user) (49 "F.Fab" user))',
           '  (setup (pad_to_mask_clearance 0) (allow_soldermask_bridges_in_footprints no))',
           '  (net 0 "")']
    for n in NETS:
        out.append(f'  (net {NET_ID[n]} {q(n)})')

    root = uid("sch-root")
    for ref, p in sorted(PARTS.items()):
        fp = FOOTPRINTS[p["fp"]]
        out.append(f'  (footprint {q("ZaidFly:" + p["fp"])} (layer "F.Cu") (uuid {q(uid("fp", ref))}) '
                   f'(at {fmt(p["x"])} {fmt(p["y"])}{" " + fmt(p["rot"]) if p["rot"] else ""})')
        ty = -(max(abs(v[1]) + v[3] / 2 for v in fp["pads"].values()) + 0.7)
        if p["fp"] == "XIAO_ESP32S3_THT":
            ty = -11.6
        out.append(f'    (property "Reference" {q(ref)} (at 0 {fmt(ty)} {fmt(p["rot"])}) (layer "F.SilkS") '
                   f'(uuid {q(uid("ref", ref))}) (effects (font (size 0.6 0.6) (thickness 0.1))))')
        out.append(f'    (property "Value" {q(p["value"])} (at 0 {fmt(-ty)} {fmt(p["rot"])}) (layer "F.Fab") '
                   f'(uuid {q(uid("val", ref))}) (effects (font (size 0.6 0.6) (thickness 0.1))))')
        out.append(f'    (path {q("/" + uid("sym", ref))}) (sheetname "Root") (sheetfile "{PROJECT}.kicad_sch")')
        out.append(f'    (attr {"smd" if fp["smd"] else "through_hole"})')
        for g in fp_graphics(p["fp"]):
            out.append("    " + g[:-1] + f' (uuid {q(uid("g", ref, g))}))')
        for pn, pd in fp["pads"].items():
            out.append("    " + pad_sexpr(pn, pd, p["rot"], p["pins"].get(pn), SMD_F, puid=uid("pad", ref, pn)))
        out.append('  )')

    # ---- board outline with rounded corners ----
    x0, y0, x1, y1, r = BX, BY, BX + BW, BY + BH, BR
    segs = [((x0 + r, y0), (x1 - r, y0)), ((x1, y0 + r), (x1, y1 - r)),
            ((x1 - r, y1), (x0 + r, y1)), ((x0, y1 - r), (x0, y0 + r))]
    for i, (a, b) in enumerate(segs):
        out.append(f'  (gr_line (start {fmt(a[0])} {fmt(a[1])}) (end {fmt(b[0])} {fmt(b[1])}) '
                   f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts") (uuid {q(uid("edge", i))}))')
    k = r * (1 - math.sqrt(0.5))
    arcs = [((x1 - r, y0), (x1 - k, y0 + k), (x1, y0 + r)), ((x1, y1 - r), (x1 - k, y1 - k), (x1 - r, y1)),
            ((x0 + r, y1), (x0 + k, y1 - k), (x0, y1 - r)), ((x0, y0 + r), (x0 + k, y0 + k), (x0 + r, y0))]
    for i, (a, m, b) in enumerate(arcs):
        out.append(f'  (gr_arc (start {fmt(a[0])} {fmt(a[1])}) (mid {fmt(m[0])} {fmt(m[1])}) (end {fmt(b[0])} {fmt(b[1])}) '
                   f'(stroke (width 0.1) (type default)) (layer "Edge.Cuts") (uuid {q(uid("arc", i))}))')

    # ---- silkscreen branding ----
    texts = [("ZAID FLY", CX, BY + BH - 9.3, "F.SilkS", 1.0, ""),
             ("FRONT", CX, BY + 9.0, "B.SilkS", 0.8, " (justify mirror)"),
             ("M1", BX + BW - 2.5, BY + 9.0, "F.SilkS", 0.7, ""), ("M4", BX + 2.5, BY + 9.0, "F.SilkS", 0.7, ""),
             ("M2", BX + BW - 2.5, BY + BH - 9.0, "F.SilkS", 0.7, ""), ("M3", BX + 2.5, BY + BH - 9.0, "F.SilkS", 0.7, ""),
             ("ZAID FLY  SWARM NODE  rev A", CX, CY, "B.SilkS", 1.0, " (justify mirror)"),
             ("ESP-NOW 200 Hz | 1 kHz loop", CX, CY + 2.2, "B.SilkS", 0.7, " (justify mirror)")]
    for t, x, y, layer, s, extra in texts:
        out.append(f'  (gr_text {q(t)} (at {fmt(x)} {fmt(y)} 0) (layer "{layer}") (uuid {q(uid("t", t))}) '
                   f'(effects (font (size {s} {s}) (thickness {fmt(s * 0.15)})){extra}))')

    # ---- GND pours on both layers ----
    for layer in ("B.Cu", "F.Cu"):
        out.append(f'  (zone (net {NET_ID["GND"]}) (net_name "GND") (layer "{layer}") (uuid {q(uid("zone", layer))}) '
                   f'(hatch edge 0.5) (connect_pads (clearance 0.25)) (min_thickness 0.2) (filled_areas_thickness no) '
                   f'(fill (thermal_gap 0.3) (thermal_bridge_width 0.35)) '
                   f'(polygon (pts (xy {fmt(x0)} {fmt(y0)}) (xy {fmt(x1)} {fmt(y0)}) (xy {fmt(x1)} {fmt(y1)}) (xy {fmt(x0)} {fmt(y1)}))))')
    out.append(')')
    return "\n".join(out) + "\n"


def write_libs():
    with open(os.path.join(KDIR, "ZaidFly.kicad_sym"), "w", encoding="utf-8") as f:
        f.write('(kicad_symbol_lib (version 20231120) (generator "zaidfly_gen") (generator_version "8.0")\n')
        for name in SYMBOLS:
            f.write("  " + symbol_def(name, name).replace("\n", "\n  ") + "\n")
        f.write(")\n")
    pdir = os.path.join(KDIR, "ZaidFly.pretty")
    os.makedirs(pdir, exist_ok=True)
    for name, fp in FOOTPRINTS.items():
        lines = [f'(footprint {q(name)} (version 20240108) (generator "zaidfly_gen") (generator_version "8.0") (layer "F.Cu")',
                 f'  (property "Reference" "REF**" (at 0 -2 0) (layer "F.SilkS") (effects (font (size 0.6 0.6) (thickness 0.1))))',
                 f'  (property "Value" {q(name)} (at 0 2 0) (layer "F.Fab") (effects (font (size 0.6 0.6) (thickness 0.1))))',
                 f'  (attr {"smd" if fp["smd"] else "through_hole"})']
        lines += ["  " + g for g in fp_graphics(name)]
        lines += ["  " + pad_sexpr(pn, pd, 0, None, SMD_F, with_net=False) for pn, pd in fp["pads"].items()]
        lines.append(")")
        with open(os.path.join(pdir, name + ".kicad_mod"), "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
    with open(os.path.join(KDIR, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n  (version 7)\n  (lib (name "ZaidFly")(type "KiCad")(uri "${KIPRJMOD}/ZaidFly.pretty")(options "")(descr "Zaid Fly footprints"))\n)\n')
    with open(os.path.join(KDIR, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n  (version 7)\n  (lib (name "ZaidFly")(type "KiCad")(uri "${KIPRJMOD}/ZaidFly.kicad_sym")(options "")(descr "Zaid Fly symbols"))\n)\n')


def write_project():
    import json
    pro = {
        "board": {"design_settings": {"rules": {"min_clearance": 0.15, "min_track_width": 0.15,
                                                 "min_via_diameter": 0.5, "min_through_hole_diameter": 0.3}}},
        "meta": {"filename": f"{PROJECT}.kicad_pro", "version": 1},
        "net_settings": {"classes": [
            {"name": "Default", "clearance": 0.15, "track_width": 0.2, "via_diameter": 0.6, "via_drill": 0.3},
            {"name": "Power", "clearance": 0.2, "track_width": 0.6, "via_diameter": 0.8, "via_drill": 0.4}],
            "meta": {"version": 3},
            "netclass_patterns": [{"netclass": "Power", "pattern": p} for p in
                                  ["VBAT", "GND", "MOT1-", "MOT2-", "MOT3-", "MOT4-"]]},
        "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[uid("sch-root"), "Root"]],
    }
    with open(os.path.join(KDIR, f"{PROJECT}.kicad_pro"), "w") as f:
        json.dump(pro, f, indent=2)


def write_bom_and_netlist():
    os.makedirs(os.path.join(ROOT, "hardware", "bom"), exist_ok=True)
    groups = {}
    for ref, p in PARTS.items():
        key = (p["value"], p["fp"], p["mpn"].split(" (")[0] if p["sym"] in ("R", "C") else p["mpn"])
        groups.setdefault(key, []).append(ref)
    natural = lambda r: (''.join(c for c in r if c.isalpha()), int(''.join(c for c in r if c.isdigit()) or 0))
    rows = sorted(groups.items(), key=lambda kv: natural(sorted(kv[1], key=natural)[0]))
    with open(os.path.join(ROOT, "hardware", "bom", "zaid-fly-bom.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["References", "Value", "Footprint", "Part / notes", "Qty per drone", "Qty for 20 drones"])
        for (val, fpn, mpn), refs in rows:
            refs = sorted(refs, key=natural)
            w.writerow([" ".join(refs), val, fpn, mpn, len(refs), len(refs) * 20])

    with open(os.path.join(KDIR, "NETLIST.md"), "w", encoding="utf-8") as f:
        f.write("# Zaid Fly carrier — netlist\n\n_Generated by `hardware/tools/generate_kicad.py`._\n\n")
        f.write("| Net | Connections |\n|---|---|\n")
        for n in NETS:
            conns = [f"{r}.{pn}" for r, p in sorted(PARTS.items(), key=lambda kv: natural(kv[0]))
                     for pn, nn in p["pins"].items() if nn == n]
            f.write(f"| `{n}` | {', '.join(conns)} |\n")


# =============================================================================
#  Renders (matplotlib) — top view of the board as generated
# =============================================================================
def render_pcb(path, show_ratsnest=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch, Rectangle, Circle

    fig, ax = plt.subplots(figsize=(7, 9), dpi=200)
    fig.patch.set_facecolor("#0d1117")
    ax.set_facecolor("#0d1117")
    ax.add_patch(FancyBboxPatch((BX, BY), BW, BH, boxstyle=f"round,pad=0,rounding_size={BR}",
                                fc="#0f5132", ec="#d4af37", lw=1.5))
    ax.add_patch(FancyBboxPatch((BX + 0.4, BY + 0.4), BW - 0.8, BH - 0.8, boxstyle=f"round,pad=0,rounding_size={BR - 0.4}",
                                fc="#14633e", ec="none"))
    copper, silk = "#e0b354", "#f2f2f2"
    pad_xy = {}
    for ref, p in PARTS.items():
        fp = FOOTPRINTS[p["fp"]]
        bw, bh = fp["body"]
        if p["fp"] == "XIAO_ESP32S3_THT":
            ax.add_patch(Rectangle((p["x"] - bw / 2, p["y"] - bh / 2 - 0.25), bw, bh, fill=False, ec=silk, lw=0.8, ls="--"))
            ax.add_patch(Rectangle((p["x"] - 4.5, p["y"] - bh / 2 - 1.85), 9, 6, fill=False, ec=silk, lw=0.6))
            ax.text(p["x"], p["y"] - bh / 2 + 1.15, "USB-C", color=silk, ha="center", va="center", fontsize=5)
            ax.text(p["x"], p["y"] + 8.3, "XIAO ESP32-S3", color=silk, ha="center", va="center", fontsize=5.5, alpha=0.8)
        for pn, (x, y, w, h, shape, drill) in fp["pads"].items():
            dx, dy = rot(x, y, p["rot"])
            gx, gy = p["x"] + dx, p["y"] + dy
            pad_xy.setdefault(p["pins"].get(pn), []).append((gx, gy))
            if p["rot"] % 180 == 90:
                w, h = h, w
            if shape in ("circle", "oval"):
                ax.add_patch(FancyBboxPatch((gx - w / 2, gy - h / 2), w, h, boxstyle=f"round,pad=0,rounding_size={min(w, h) / 2 - 0.01}",
                                            fc=copper, ec="none"))
                ax.add_patch(Circle((gx, gy), drill / 2, fc="#0d1117"))
            else:
                ax.add_patch(FancyBboxPatch((gx - w / 2, gy - h / 2), w, h, boxstyle="round,pad=0,rounding_size=0.08",
                                            fc="#c9cbd1" if not fp["smd"] else copper, ec="none"))
        if p["fp"] not in ("XIAO_ESP32S3_THT",):
            ty = max(abs(rot(v[0], v[1], p["rot"])[1]) + 0.5 for v in fp["pads"].values()) + 0.45
            if ref.startswith("M") or ref.startswith("J"):
                ty = -ty
            ax.text(p["x"], p["y"] - ty, ref, color=silk, ha="center", va="center", fontsize=3.6)
        if p["fp"].startswith("QFN"):
            ax.add_patch(Rectangle((p["x"] - 2, p["y"] - 2), 4, 4, fill=False, ec=silk, lw=0.6))
            ax.add_patch(Circle((p["x"] - 2.6, p["y"] - 2.6), 0.18, fc=silk))
        if p["fp"] == "SOT-23":
            ax.add_patch(Rectangle((p["x"] - 1.45, p["y"] - 0.65), 2.9, 1.3, fc="#222", ec="none"))
        if p["fp"] == "LED_0603":
            col = {"White": "#fffbe6", "Red": "#ff4d4d", "Green": "#39ff88"}[p["value"]]
            ax.add_patch(Rectangle((p["x"] - 0.4, p["y"] - 0.35), 0.8, 0.7, fc=col, ec="none", alpha=0.9))

    if show_ratsnest:
        import itertools
        for net, pts in pad_xy.items():
            if not net or net == "GND" or len(pts) < 2:
                continue
            # minimum spanning tree (Prim) — what KiCad draws as ratsnest
            done, rest = [pts[0]], pts[1:]
            while rest:
                a, b = min(((a, b) for a in done for b in rest), key=lambda ab: math.dist(*ab))
                col = "#7dd3fc" if net not in ("VBAT", "+3V3") else "#fca5a5"
                ax.plot([a[0], b[0]], [a[1], b[1]], color=col, lw=0.45, alpha=0.85)
                done.append(b)
                rest.remove(b)

    for t, x, y, s in (("ZAID FLY", CX, BY + BH - 9.3, 11), 
                       ("M1", BX + BW - 2.5, BY + 9.0, 6), ("M4", BX + 2.5, BY + 9.0, 6),
                       ("M2", BX + BW - 2.5, BY + BH - 9.0, 6), ("M3", BX + 2.5, BY + BH - 9.0, 6)):
        ax.text(x, y, t, color=silk, ha="center", va="center", fontsize=s, fontweight="bold")

    ax.set_xlim(BX - 2, BX + BW + 2)
    ax.set_ylim(BY + BH + 2, BY - 2)
    ax.set_aspect("equal")
    ax.axis("off")
    title = "Zaid Fly carrier PCB — rev A" + ("  (placement + ratsnest)" if show_ratsnest else "  (top)")
    ax.set_title(title, color="white", fontsize=10)
    fig.text(0.5, 0.03, f"{BW:.0f} × {BH:.0f} mm  ·  2-layer 1.0 mm FR-4  ·  {len(PARTS)} parts  ·  {len(NETS)} nets",
             color="#9aa4b2", ha="center", fontsize=7)
    fig.savefig(path, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def check_overlaps():
    """Courtyard overlap check so placement edits don't silently collide."""
    boxes = []
    for ref, p in PARTS.items():
        if p["fp"] == "XIAO_ESP32S3_THT":
            pads = [(ref, *rot(v[0], v[1], 0)) for v in FOOTPRINTS[p["fp"]]["pads"].values()]
            for i, v in enumerate(FOOTPRINTS[p["fp"]]["pads"].values()):
                boxes.append((f"{ref}.{i + 1}", p["x"] + v[0] - 1.3, p["y"] + v[1] - 0.8, p["x"] + v[0] + 1.3, p["y"] + v[1] + 0.8))
            continue
        xs, ys = [], []
        for x, y, w, h, *_ in FOOTPRINTS[p["fp"]]["pads"].values():
            for cx_, cy_ in ((x - w / 2, y - h / 2), (x + w / 2, y + h / 2), (x - w / 2, y + h / 2), (x + w / 2, y - h / 2)):
                dx, dy = rot(cx_, cy_, p["rot"])
                xs.append(p["x"] + dx)
                ys.append(p["y"] + dy)
        boxes.append((ref, min(xs) - 0.1, min(ys) - 0.1, max(xs) + 0.1, max(ys) + 0.1))
    problems = []
    for i, a in enumerate(boxes):
        if a[1] < BX + 0.3 or a[3] > BX + BW - 0.3 or a[2] < BY + 0.3 or a[4] > BY + BH - 0.3:
            problems.append(f"{a[0]} too close to board edge")
        for b in boxes[i + 1:]:
            if a[0].split(".")[0] == b[0].split(".")[0]:
                continue
            if a[1] < b[3] and b[1] < a[3] and a[2] < b[4] and b[2] < a[4]:
                problems.append(f"{a[0]} overlaps {b[0]}")
    return problems


def main():
    os.makedirs(KDIR, exist_ok=True)
    os.makedirs(IMG, exist_ok=True)
    problems = check_overlaps()
    for pr in problems:
        print("PLACEMENT:", pr)
    with open(os.path.join(KDIR, f"{PROJECT}.kicad_sch"), "w", encoding="utf-8") as f:
        f.write(build_schematic())
    with open(os.path.join(KDIR, f"{PROJECT}.kicad_pcb"), "w", encoding="utf-8") as f:
        f.write(build_pcb())
    write_libs()
    write_project()
    write_bom_and_netlist()
    render_pcb(os.path.join(IMG, "pcb-top-render.png"))
    render_pcb(os.path.join(IMG, "pcb-ratsnest-render.png"), show_ratsnest=True)
    print(f"{len(PARTS)} parts, {len(NETS)} nets, {len(problems)} placement issues")


if __name__ == "__main__":
    main()
