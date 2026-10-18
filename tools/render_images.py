#!/usr/bin/env python3
"""
Renders the README images from the real project files:
  * 3d-models/stl/*.stl  -> shaded renders of the frame, cover and full assembly
  * a top-down "digital twin" of the 20-drone swarm grid
  * system architecture + latency budget diagrams

Run:  python tools/render_images.py      (needs numpy + matplotlib)
"""
import math
import os
import struct

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch, FancyArrowPatch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
STL = os.path.join(ROOT, "3d-models", "stl")
OUT = os.path.join(ROOT, "docs", "images")
BG = "#0d1117"
FG = "#e6edf3"
MUTED = "#8b949e"
ACCENT = "#2f81f7"


def load_stl(name):
    b = open(os.path.join(STL, name), "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def shade(tris, base_rgb, light=(0.4, -0.5, 0.85)):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    L = np.array(light) / np.linalg.norm(light)
    k = np.clip(np.abs(n @ L), 0, 1) * 0.65 + 0.35
    return np.clip(np.outer(k, base_rgb), 0, 1)


def setup_3d(fig, pos=111, elev=28, azim=-55):
    ax = fig.add_subplot(pos, projection="3d")
    ax.set_facecolor(BG)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    return ax


def add_mesh(ax, tris, rgb, alpha=1.0, light=(0.4, -0.5, 0.85)):
    pc = Poly3DCollection(tris, facecolors=shade(tris, rgb, light), edgecolors="none", alpha=alpha)
    ax.add_collection3d(pc)


def set_bounds(ax, lo, hi):
    c = (np.array(lo) + np.array(hi)) / 2
    r = (np.array(hi) - np.array(lo)).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))


def cylinder(cx, cy, z0, z1, r, n=24):
    t = np.linspace(0, 2 * np.pi, n + 1)
    tris = []
    for a, b in zip(t[:-1], t[1:]):
        p0 = (cx + r * np.cos(a), cy + r * np.sin(a))
        p1 = (cx + r * np.cos(b), cy + r * np.sin(b))
        tris.append([(*p0, z0), (*p1, z0), (*p1, z1)])
        tris.append([(*p0, z0), (*p1, z1), (*p0, z1)])
        tris.append([(cx, cy, z1), (*p0, z1), (*p1, z1)])
    return np.array(tris, float)


def disc(cx, cy, z, r, n=40):
    t = np.linspace(0, 2 * np.pi, n + 1)
    return np.array([[(cx, cy, z), (cx + r * np.cos(a), cy + r * np.sin(a), z),
                      (cx + r * np.cos(b), cy + r * np.sin(b), z)] for a, b in zip(t[:-1], t[1:])])


# Motor centres of the standard Zaid Fly frame (measured from the STL, see find_motor_mounts)
MOTORS = [(17.0, 17.0), (17.0, -17.0), (-17.0, -17.0), (-17.0, 17.0)]


def find_motor_mounts(frame):
    """Motor rings sit at the four arm tips: take the far-out vertices in each
    quadrant and use the centre of their bounding box."""
    v = frame.reshape(-1, 3)
    far = v[np.hypot(v[:, 0], v[:, 1]) > 0.75 * np.abs(v[:, :2]).max() * math.sqrt(2) * 0.75]
    out = []
    for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
        q = far[(np.sign(far[:, 0]) == sx) & (np.sign(far[:, 1]) == sy)]
        if len(q) < 10:
            return MOTORS
        out.append((float(q[:, 0].min() + q[:, 0].max()) / 2, float(q[:, 1].min() + q[:, 1].max()) / 2))
    return out


def render_part(name, out, rgb, title):
    tris = load_stl(name)
    fig = plt.figure(figsize=(6, 6), dpi=160)
    fig.patch.set_facecolor(BG)
    ax = setup_3d(fig)
    add_mesh(ax, tris, rgb)
    v = tris.reshape(-1, 3)
    set_bounds(ax, v.min(0), v.max(0))
    lo, hi = v.min(0), v.max(0)
    fig.text(0.5, 0.94, title, color=FG, ha="center", fontsize=13, fontweight="bold")
    fig.text(0.5, 0.06, f"{hi[0] - lo[0]:.1f} × {hi[1] - lo[1]:.1f} × {hi[2] - lo[2]:.1f} mm  ·  {len(tris):,} triangles  ·  {name}",
             color=MUTED, ha="center", fontsize=8)
    fig.savefig(os.path.join(OUT, out), facecolor=BG)
    plt.close(fig)


def render_assembly(frame, cover, motors, out):
    fig = plt.figure(figsize=(8, 6.5), dpi=170)
    fig.patch.set_facecolor(BG)
    ax = setup_3d(fig, elev=24, azim=-60)
    add_mesh(ax, frame, (0.93, 0.93, 0.92))
    add_mesh(ax, cover, (0.97, 0.97, 0.97))
    for mx, my in motors:
        add_mesh(ax, cylinder(mx, my, -4, 12.5, 3.5), (0.12, 0.12, 0.13))      # 720 motor can
        add_mesh(ax, cylinder(mx, my, 12.5, 15.5, 0.6), (0.7, 0.7, 0.7))      # shaft
        add_mesh(ax, disc(mx, my, 15.0, 15.5), (0.85, 0.92, 1.0), alpha=0.35)  # 31 mm prop disc
    # battery under the frame
    bx = np.array([[-8, -13, -6.5], [8, -13, -6.5], [8, 13, -6.5], [-8, 13, -6.5]])
    box = []
    for z in (-6.5, -0.6):
        b = bx.copy(); b[:, 2] = z
        box += [[b[0], b[1], b[2]], [b[0], b[2], b[3]]]
    for i in range(4):
        a, c = bx[i].copy(), bx[(i + 1) % 4].copy()
        a2, c2 = a.copy(), c.copy(); a2[2] = c2[2] = -0.6
        box += [[a, c, c2], [a, c2, a2]]
    add_mesh(ax, np.array(box, float), (0.85, 0.35, 0.15))
    set_bounds(ax, (-28, -28, -8), (28, 28, 16))
    fig.text(0.5, 0.93, "Zaid Fly — full assembly (render)", color=FG, ha="center", fontsize=14, fontweight="bold")
    fig.text(0.5, 0.05, "3D-printed frame + cover · 4× 720 coreless motors · 31 mm props · 1S LiPo · carrier PCB under the cover",
             color=MUTED, ha="center", fontsize=8)
    fig.savefig(os.path.join(OUT, out), facecolor=BG)
    plt.close(fig)


def top_view_polys(tris):
    """Project to XY, keep the upward-ish faces, sorted by height (painter's algorithm)."""
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    up = n[:, 2] > 0
    t = tris[up]
    order = np.argsort(t[:, :, 2].mean(1))
    return t[order]


def render_swarm(frame, cover, motors, out):
    from matplotlib.collections import PolyCollection
    ft, ct = top_view_polys(frame), top_view_polys(cover)
    fcol = shade(ft, (0.92, 0.92, 0.91), light=(0.3, 0.5, 1.0))
    ccol = shade(ct, (0.98, 0.98, 0.98), light=(0.3, 0.5, 1.0))
    fig, ax = plt.subplots(figsize=(10, 12.5), dpi=150)
    fig.patch.set_facecolor("#c9ccd1")
    ax.set_facecolor("#c9ccd1")
    rng = np.random.default_rng(20)
    pitch = 62
    for i in range(20):
        r, c = divmod(i, 4)
        ox, oy = c * pitch + (r % 2) * 14, -r * pitch
        ang = rng.uniform(-25, 25)
        R = np.array([[math.cos(math.radians(ang)), -math.sin(math.radians(ang))],
                      [math.sin(math.radians(ang)), math.cos(math.radians(ang))]])
        tf = lambda p: p @ R.T + (ox, oy)
        ax.add_collection(PolyCollection([tf(p[:, :2]) for p in ft], facecolors=fcol, edgecolors="none"))
        for mx, my in motors:
            p = tf(np.array([[mx, my]]))[0]
            ax.add_patch(Circle(p, 4.4, fc="#1b1b1d", ec="#000", lw=0.4, zorder=3))
            ax.add_patch(Circle(p, 15.5, fc=(0.8, 0.88, 1.0, 0.18), ec=(1, 1, 1, 0.35), lw=0.5, zorder=4))
            for k in range(2):
                a = math.radians(rng.uniform(0, 180) + k * 180)
                ax.plot([p[0], p[0] + 15 * math.cos(a)], [p[1], p[1] + 15 * math.sin(a)], color=(1, 1, 1, 0.55), lw=3, zorder=4,
                        solid_capstyle="round")
        ax.add_collection(PolyCollection([tf(p[:, :2]) for p in ct], facecolors=ccol, edgecolors="none", zorder=5))
        ax.text(ox, oy + 2.2, "ZAID", ha="center", va="center", fontsize=8, fontweight="bold", rotation=ang, zorder=6, color="#222")
        ax.text(ox, oy - 2.6, "FLY", ha="center", va="center", fontsize=8, fontweight="bold", rotation=ang, zorder=6, color="#222")
        led = "#ff3b3b" if i % 3 == 0 else "#2bff6f"
        p = tf(np.array([[0, -14.5]]))[0]
        ax.add_patch(Circle(p, 2.2, fc=led, ec="none", zorder=6, alpha=0.95))
        ax.add_patch(Circle(p, 5.0, fc=led, ec="none", zorder=5, alpha=0.25))
        p = tf(np.array([[16, -3]]))[0]
        ax.text(p[0] + 8, p[1] - 6, f"#{i:02d}", fontsize=8, color="#333", zorder=7, fontweight="bold")
    ax.set_aspect("equal")
    ax.set_xlim(-40, 3 * pitch + 55)
    ax.set_ylim(-4 * pitch - 40, 40)
    ax.axis("off")
    ax.set_title("Zaid Fly swarm — 20 nodes, IDs 00-19 (render from the STL files)", fontsize=13, color="#111")
    fig.savefig(os.path.join(OUT, out), facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)


def box(ax, x, y, w, h, title, sub="", fc="#161b22", ec=ACCENT):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.12", fc=fc, ec=ec, lw=1.6))
    ax.text(x + w / 2, y + h / 2 + (0.13 if sub else 0), title, color=FG, ha="center", va="center", fontsize=10.5, fontweight="bold")
    if sub:
        ax.text(x + w / 2, y + h / 2 - 0.2, sub, color=MUTED, ha="center", va="center", fontsize=7.5)


def arrow(ax, a, b, text="", color=ACCENT, rad=0.0, fs=7.5, off=(0, 0.12)):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, color=color, lw=1.4,
                                 connectionstyle=f"arc3,rad={rad}"))
    if text:
        ax.text((a[0] + b[0]) / 2 + off[0], (a[1] + b[1]) / 2 + off[1], text, color=color, ha="center", fontsize=fs)


def render_architecture(out):
    fig, ax = plt.subplots(figsize=(12, 6.2), dpi=160)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    box(ax, 0.2, 2.6, 2.4, 1.1, "PC / laptop", "swarm_controller.py\ngamepad · lights · latency")
    box(ax, 3.6, 2.6, 2.6, 1.1, "Ground station", "XIAO ESP32-S3 + E-STOP\n200 Hz hardware-timed TX")
    arrow(ax, (2.6, 3.15), (3.6, 3.15), "USB 100 Hz\nSET_ALL (160 B)", off=(0, 0.15))
    # drones
    for i in range(20):
        r, c = divmod(i, 5)
        x, y = 8.0 + c * 0.82, 5.0 - r * 1.25
        ax.add_patch(FancyBboxPatch((x, y), 0.66, 0.85, boxstyle="round,pad=0.01,rounding_size=0.1",
                                    fc="#0f2a1c", ec="#3fb950", lw=1.1))
        ax.text(x + 0.33, y + 0.55, f"{i:02d}", color=FG, ha="center", fontsize=8, fontweight="bold")
        ax.text(x + 0.33, y + 0.22, "slot", color=MUTED, ha="center", fontsize=5.5)
    ax.text(10.0, 6.05, "20 × Zaid Fly drones (same firmware, ID in NVS)", color=FG, ha="center", fontsize=10, fontweight="bold")
    arrow(ax, (6.2, 3.45), (7.9, 4.6), "ONE ESP-NOW broadcast / 5 ms\n172 B = header + 20 × 8 B slots\nno ACK · no retries · ch 6 · 6 Mbps",
          color="#3fb950", rad=-0.15, off=(-1.0, 0.75))
    arrow(ax, (7.9, 1.9), (6.2, 2.8), "telemetry: drone N replies only when\nseq % 20 == N  →  TDMA, zero collisions",
          color="#d29922", rad=-0.15, off=(-0.9, -0.85))
    # frame layout strip
    y0 = 0.35
    ax.text(0.2, y0 + 0.75, "SwarmFrame (172 bytes)", color=FG, fontsize=9, fontweight="bold")
    segs = [("magic", 2), ("ver", 1), ("flags", 1), ("seq", 4), ("t_us", 4)] + [(f"{i}", 8) for i in range(20)]
    x, scale = 0.2, 6.9 / 172
    for name, n in segs:
        w = n * scale
        col = "#1f6feb" if not name.isdigit() else ("#238636" if int(name) % 2 == 0 else "#2ea043")
        ax.add_patch(plt.Rectangle((x, y0), w, 0.45, fc=col, ec=BG, lw=0.6))
        if w > 0.2:
            ax.text(x + w / 2, y0 + 0.22, name, color="white", ha="center", va="center", fontsize=5.5)
        x += w
    ax.text(7.25, y0 + 0.22, "each slot: roll · pitch · yaw-rate · throttle(10 bit) + 6 flag bits", color=MUTED, fontsize=7.5, va="center")
    ax.set_xlim(0, 12.4)
    ax.set_ylim(0, 6.4)
    ax.axis("off")
    ax.set_title("Zaid Fly swarm — control link architecture", color=FG, fontsize=14, fontweight="bold")
    fig.savefig(os.path.join(OUT, out), facecolor=BG, bbox_inches="tight")
    plt.close(fig)


def render_latency_budget(out):
    """Worst-case design budget from stick movement to motor PWM change."""
    stages = [
        ("PC loop + USB CDC", 10.0, 1.0, "#6e7681"),
        ("wait for next 200 Hz TX tick", 5.0, 2.5, "#1f6feb"),
        ("ESP-NOW airtime @ 6 Mbps (172 B)", 0.35, 0.35, "#3fb950"),
        ("Wi-Fi RX task → callback copy", 0.2, 0.1, "#2ea043"),
        ("wait for next 1 kHz control tick", 1.0, 0.5, "#d29922"),
        ("IMU read + PID + mixer", 0.45, 0.45, "#db6d28"),
        ("PWM period (20 kHz)", 0.05, 0.05, "#f85149"),
    ]
    fig, ax = plt.subplots(figsize=(11, 3.8), dpi=160)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    for row, key, lab in ((1, 1, "worst case"), (0, 2, "typical")):
        x = 0
        for name, worst, typ, col in stages:
            w = worst if key == 1 else typ
            ax.barh(row, w, left=x, color=col, edgecolor=BG, height=0.55)
            x += w
        ax.text(x + 0.25, row, f"{x:.1f} ms", color=FG, va="center", fontsize=10, fontweight="bold")
    ax.set_yticks([0, 1], ["typical", "worst case"], color=FG)
    ax.tick_params(colors=MUTED)
    for s in ax.spines.values():
        s.set_color("#30363d")
    ax.set_xlabel("milliseconds from gamepad sample to motor output", color=MUTED)
    ax.axvline(50, color="#f85149", ls="--", lw=1)
    ax.text(49.5, 0.5, "~50 ms: where humans\nstart to notice lag", color="#f85149", ha="right", fontsize=8)
    ax.set_xlim(0, 52)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for *_, c in stages]
    ax.legend(handles, [s[0] for s in stages], loc="upper center", bbox_to_anchor=(0.5, -0.32), ncol=4, fontsize=7.5,
              frameon=False, labelcolor=FG)
    ax.set_title("Latency budget (design target, per drone — all 20 receive the same frame at the same time)",
                 color=FG, fontsize=11, fontweight="bold")
    fig.savefig(os.path.join(OUT, out), facecolor=BG, bbox_inches="tight")
    plt.close(fig)


def render_unicast_vs_broadcast(out):
    fig, ax = plt.subplots(figsize=(11, 3.4), dpi=160)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)
    # unicast: 20 x (frame 0.06 ms @ 6M for 20B + SIFS + ACK + backoff ~0.25 ms)
    x = 0
    for i in range(20):
        ax.barh(1, 0.13, left=x, color="#f85149", edgecolor=BG)
        ax.barh(1, 0.12, left=x + 0.13, color="#6e7681", edgecolor=BG)
        x += 0.25
    ax.text(x + 0.05, 1, f"~{x:.1f} ms, drone 19 hears its command {x:.1f} ms after drone 0\n(+ retries on any loss)",
            color=FG, va="center", fontsize=8.5)
    ax.barh(0, 0.35, left=0, color="#3fb950", edgecolor=BG)
    ax.text(0.45, 0, "0.35 ms, all 20 drones hear the same frame at the same instant", color=FG, va="center", fontsize=8.5)
    ax.set_yticks([0, 1], ["1 × broadcast\n(Zaid Fly)", "20 × unicast\n+ ACK"], color=FG)
    ax.tick_params(colors=MUTED)
    for s in ax.spines.values():
        s.set_color("#30363d")
    ax.set_xlim(0, 11)
    ax.set_xlabel("air time per control update (ms)", color=MUTED)
    ax.set_title("Why one broadcast frame instead of 20 messages", color=FG, fontsize=11, fontweight="bold")
    fig.savefig(os.path.join(OUT, out), facecolor=BG, bbox_inches="tight")
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    frame = load_stl("zaid-fly-frame.stl")
    cover = load_stl("zaid-fly-cover.stl")
    motors = find_motor_mounts(frame)
    print("motor mounts:", [(round(x, 1), round(y, 1)) for x, y in motors])
    render_part("zaid-fly-frame.stl", "render-frame.png", (0.93, 0.93, 0.92), "Zaid Fly frame")
    render_part("zaid-fly-cover.stl", "render-cover.png", (0.95, 0.95, 0.95), "Zaid Fly cover")
    render_part("zaid-fly-60mm-brushed-extended-frame.stl", "render-frame-60mm.png", (0.9, 0.9, 0.9),
                "Zaid Fly 60 mm extended-arm frame")
    render_assembly(frame, cover, motors, "render-assembly.png")
    render_swarm(frame, cover, motors, "render-swarm-grid.png")
    render_architecture("diagram-architecture.png")
    render_latency_budget("diagram-latency-budget.png")
    render_unicast_vs_broadcast("diagram-broadcast-vs-unicast.png")
    print("done")


if __name__ == "__main__":
    main()
