#!/usr/bin/env python3
"""
Zaid Fly — 20-drone swarm controller (PC side)

    PC  --USB-->  ground station ESP32-S3  --ESP-NOW broadcast-->  20 drones

Modes
  lights   : drones stay DISARMED, headlights/taillights run a wave pattern.
             Use this first to check that all 20 nodes are on the link.
  gamepad  : one gamepad flies the whole swarm in lock-step (same command to
             every drone), or a single drone selected with the D-pad.
  latency  : prints round-trip-time statistics per drone from telemetry.

Safety
  * SPACE / gamepad B / Ctrl+C  -> E-STOP: every motor cut immediately.
  * Close this program          -> ground station stops transmitting after
                                   300 ms and every drone failsafe-lands.

Usage
  python swarm_controller.py --port COM5 --mode lights
  python swarm_controller.py --port COM5 --mode gamepad
  python swarm_controller.py --port COM5 --mode latency
"""
import argparse
import statistics
import struct
import sys
import threading
import time

import serial  # pyserial

MAX_NODES = 20
SYNC = b"\xA5\x5A"

# --- protocol constants: keep in sync with firmware/common/swarm_protocol.h ---
CMD_ARM, CMD_KILL, CMD_HEADLIGHT, CMD_TAILLIGHT, CMD_CALIBRATE, CMD_BLINK = (1 << i for i in range(6))
SER_SET_ALL, SER_SET_ONE, SER_ESTOP, SER_PING = 0x01, 0x02, 0x03, 0x04
SER_TELEMETRY, SER_PONG, SER_STATUS = 0x81, 0x84, 0x85
STATES = ["BOOT", "CAL", "DISARMED", "ARMED", "FAILSAFE", "KILLED", "LOW_BAT"]

TELEM_FMT = "<HBBIIHHhhHHbBI"   # Telemetry (26 bytes) + uint32 rtt_us
TELEM_FIELDS = ("magic node state echo_seq echo_gs_us rx_age_us vbat_mv roll_cd "
                "pitch_cd loop_us lost rssi reserved rtt_us").split()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class DroneCmd:
    __slots__ = ("roll", "pitch", "yaw_rate", "throttle", "flags")

    def __init__(self):
        self.roll = self.pitch = self.yaw_rate = 0.0   # deg, deg, deg/s
        self.throttle = 0.0                            # 0..1
        self.flags = 0

    def pack(self) -> bytes:
        thr = max(0, min(1000, int(self.throttle * 1000)))
        word = (thr & 0x3FF) | ((self.flags & 0x3F) << 10)
        return struct.pack("<hhhH",
                           int(max(-3000, min(3000, self.roll * 100))),
                           int(max(-3000, min(3000, self.pitch * 100))),
                           int(max(-2000, min(2000, self.yaw_rate * 10))),
                           word)


class Swarm:
    """Owns the serial link, the 20 command slots and the latest telemetry."""

    def __init__(self, port: str, send_hz: float = 100.0):
        self.ser = serial.Serial(port, 921600, timeout=0.01)
        self.cmds = [DroneCmd() for _ in range(MAX_NODES)]
        self.telem = {}            # node -> dict
        self.rtt = {n: [] for n in range(MAX_NODES)}
        self.status = {}
        self.lock = threading.Lock()
        self.running = True
        self.period = 1.0 / send_hz
        threading.Thread(target=self._rx_loop, daemon=True).start()
        threading.Thread(target=self._tx_loop, daemon=True).start()

    # ---- framing ----
    def _send(self, typ: int, payload: bytes = b""):
        body = bytes([typ, len(payload)]) + payload
        self.ser.write(SYNC + body + bytes([crc8(body)]))

    def _tx_loop(self):
        nxt = time.perf_counter()
        while self.running:
            with self.lock:
                payload = b"".join(c.pack() for c in self.cmds)
            self._send(SER_SET_ALL, payload)
            nxt += self.period
            time.sleep(max(0.0, nxt - time.perf_counter()))

    def _rx_loop(self):
        buf = bytearray()
        while self.running:
            buf += self.ser.read(512)
            while True:
                i = buf.find(SYNC)
                if i < 0 or len(buf) < i + 5:
                    break
                typ, ln = buf[i + 2], buf[i + 3]
                if len(buf) < i + 5 + ln:
                    break
                body = bytes(buf[i + 2:i + 4 + ln])
                ok = crc8(body) == buf[i + 4 + ln]
                payload = body[2:]
                del buf[:i + 5 + ln]
                if ok:
                    self._handle(typ, payload)

    def _handle(self, typ, p):
        if typ == SER_TELEMETRY and len(p) == struct.calcsize(TELEM_FMT):
            t = dict(zip(TELEM_FIELDS, struct.unpack(TELEM_FMT, p)))
            t["t"] = time.time()
            self.telem[t["node"]] = t
            r = self.rtt[t["node"]]
            r.append(t["rtt_us"])
            del r[:-200]
        elif typ == SER_STATUS and len(p) == 8:
            sent, age = struct.unpack("<II", p)
            self.status = {"frames_sent": sent, "pc_age_ms": age}

    # ---- commands ----
    def set_all(self, **kw):
        with self.lock:
            for c in self.cmds:
                for k, v in kw.items():
                    setattr(c, k, v)

    def estop(self):
        for _ in range(5):
            self._send(SER_ESTOP)
        with self.lock:
            for c in self.cmds:
                c.throttle, c.flags = 0, CMD_KILL

    def close(self):
        self.estop()
        time.sleep(0.05)
        self.running = False
        self.ser.close()

    # ---- display ----
    def print_table(self):
        now = time.time()
        lines = ["\x1b[H\x1b[2J  ZAID FLY SWARM   frames sent: %s\n" % self.status.get("frames_sent", "-"),
                 " id  state      vbat   roll  pitch  rssi  rtt(ms)  lost  loop(us)  age"]
        online = 0
        for n in range(MAX_NODES):
            t = self.telem.get(n)
            if not t:
                lines.append(f" {n:2d}  ---")
                continue
            age = now - t["t"]
            online += age < 1.0
            lines.append(
                f" {n:2d}  {STATES[t['state']] if t['state'] < len(STATES) else '?':9s} "
                f"{t['vbat_mv'] / 1000:5.2f}V {t['roll_cd'] / 100:6.1f} {t['pitch_cd'] / 100:6.1f} "
                f"{t['rssi']:5d} {t['rtt_us'] / 1000:8.2f} {t['lost']:5d} {t['loop_us']:8d} {age:5.1f}s")
        lines.append(f"\n online: {online}/{MAX_NODES}    [SPACE / Ctrl+C = E-STOP]")
        print("\n".join(lines), flush=True)


# =============================================================================
def mode_lights(sw: Swarm):
    """Disarmed light wave: a quick way to see all 20 drones respond together."""
    t0 = time.time()
    while True:
        t = time.time() - t0
        with sw.lock:
            for n, c in enumerate(sw.cmds):
                c.throttle = 0
                phase = (t * 2 - n / MAX_NODES) % 1.0
                c.flags = (CMD_HEADLIGHT if phase < 0.5 else 0) | (CMD_TAILLIGHT if phase >= 0.5 else 0)
        sw.print_table()
        time.sleep(0.1)


def mode_gamepad(sw: Swarm):
    try:
        import pygame
    except ImportError:
        sys.exit("gamepad mode needs pygame:  pip install pygame")
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit("no gamepad found")
    js = pygame.joystick.Joystick(0)
    js.init()
    print("Gamepad:", js.get_name())
    print("START = arm all, BACK = disarm all, B = E-STOP, D-pad = select single drone, Y = whole swarm")

    armed, selected, thr = False, None, 0.0
    dz = lambda v: 0.0 if abs(v) < 0.08 else v
    last_print = 0
    while True:
        for e in pygame.event.get():
            if e.type == pygame.JOYBUTTONDOWN:
                if e.button == 7: armed = thr < 0.05          # START (only at zero throttle)
                if e.button == 6: armed = False               # BACK
                if e.button == 1: sw.estop(); armed = False   # B
                if e.button == 3: selected = None             # Y
            if e.type == pygame.JOYHATMOTION and e.value != (0, 0):
                selected = ((selected if selected is not None else -1) + e.value[0] - e.value[1]) % MAX_NODES

        # Mode 2: left stick = throttle (rate) / yaw, right stick = roll / pitch
        thr = max(0.0, min(1.0, thr - dz(js.get_axis(1)) * 0.02))
        yaw = dz(js.get_axis(0)) * 150
        roll = dz(js.get_axis(2)) * 25
        pitch = dz(js.get_axis(3)) * 25        # stick forward = nose down = forward
        flags = (CMD_ARM if armed else 0) | CMD_HEADLIGHT | CMD_TAILLIGHT

        with sw.lock:
            for n, c in enumerate(sw.cmds):
                if selected is None or n == selected:
                    c.roll, c.pitch, c.yaw_rate, c.throttle, c.flags = roll, pitch, yaw, thr, flags
                else:      # unselected drones: hold level, keep their throttle
                    c.roll = c.pitch = c.yaw_rate = 0
                    c.flags = (c.flags & ~CMD_BLINK)
                if selected is not None and n == selected:
                    c.flags |= CMD_BLINK
        if time.time() - last_print > 0.2:
            sw.print_table()
            print(f" target: {'ALL' if selected is None else selected}   throttle {thr:4.2f}   "
                  f"{'ARMED' if armed else 'disarmed'}")
            last_print = time.time()
        time.sleep(0.01)


def mode_latency(sw: Swarm):
    """Disarmed: keep the link alive and report round-trip time per drone."""
    sw.set_all(throttle=0, flags=0)
    while True:
        time.sleep(1.0)
        print("\x1b[H\x1b[2J  ROUND-TRIP LATENCY (ground station -> drone -> ground station)\n")
        print(" id   n    min    median   p95    max   (ms)   one-way est.")
        all_rtt = []
        for n in range(MAX_NODES):
            r = list(sw.rtt[n])
            if not r:
                print(f" {n:2d}   -")
                continue
            all_rtt += r
            s = sorted(r)
            p95 = s[int(len(s) * 0.95) - 1] if len(s) > 1 else s[0]
            rx_age = sw.telem[n]["rx_age_us"]
            one_way = (statistics.median(s) - rx_age) / 2
            print(f" {n:2d} {len(s):4d} {s[0] / 1000:6.2f} {statistics.median(s) / 1000:8.2f} "
                  f"{p95 / 1000:6.2f} {s[-1] / 1000:6.2f}          {one_way / 1000:6.2f}")
        if all_rtt:
            print(f"\n swarm median RTT: {statistics.median(all_rtt) / 1000:.2f} ms over {len(all_rtt)} samples")


def main():
    ap = argparse.ArgumentParser(description="Zaid Fly swarm controller")
    ap.add_argument("--port", required=True, help="ground station serial port, e.g. COM5 or /dev/ttyACM0")
    ap.add_argument("--mode", choices=["lights", "gamepad", "latency"], default="lights")
    args = ap.parse_args()

    sw = Swarm(args.port)
    if args.mode != "gamepad":
        threading.Thread(target=_space_estop, args=(sw,), daemon=True).start()
    try:
        {"lights": mode_lights, "gamepad": mode_gamepad, "latency": mode_latency}[args.mode](sw)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nE-STOP sent, closing.")
        sw.close()


def _space_estop(sw: Swarm):
    """SPACE in the terminal = E-STOP (Windows: msvcrt, others: just use Ctrl+C)."""
    try:
        import msvcrt
    except ImportError:
        return
    while sw.running:
        if msvcrt.kbhit() and msvcrt.getch() == b" ":
            sw.estop()
            print("\n*** E-STOP ***")
        time.sleep(0.02)


if __name__ == "__main__":
    main()
