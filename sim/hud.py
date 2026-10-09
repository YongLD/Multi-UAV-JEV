# Copyright (c) 2026 the jev-drone authors
# Derived from RomanSlack/jev-drone (MIT); see licenses/jev-drone-MIT.txt.
"""Telemetry overlay: makes the Jev layer visible in the video."""
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BG      = (11, 14, 17)
PANEL   = (17, 21, 26)
LINE    = (38, 45, 54)
TEXT    = (222, 230, 238)
DIM     = (118, 132, 146)
ACCENT  = (255, 159, 28)      # Jev
CYAN    = (46, 196, 241)      # perception
GREEN   = (64, 214, 141)
RED     = (240, 78, 72)

MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"
_F = {}


def font(size, bold=False):
    k = (size, bold)
    if k not in _F:
        _F[k] = ImageFont.truetype(BOLD if bold else MONO, size)
    return _F[k]


def _depth_rgb(z, zmax=25.0):
    """Pixelated false-colour depth, the way a range sensor would look."""
    t = np.clip(z / zmax, 0, 1)
    r = np.clip(1.6 - 2.2 * t, 0, 1)
    g = np.clip(1.5 - abs(2.2 * t - 1.0) * 1.4, 0, 1)
    b = np.clip(2.2 * t - 0.5, 0, 1)
    return (np.dstack([r, g, b]) * 255).astype(np.uint8)


class Hud:
    W_PANEL = 440

    def __init__(self, view_w, view_h):
        self.vw, self.vh = view_w, view_h
        self.W, self.H = view_w + self.W_PANEL, view_h
        self.smooth = {}

    def _bar(self, dr, x, y, w, h, frac, col, bg=(28, 34, 41)):
        dr.rectangle([x, y, x + w, y + h], fill=bg)
        if frac > 0:
            dr.rectangle([x, y, x + max(2, int(w * min(frac, 1.0))), y + h], fill=col)

    def _lock_on(self, img):
        """The chase camera looks AT the aircraft, so it is always dead centre.
        Ring + corner brackets so a 30 cm quadrotor reads at a glance."""
        cx, cy = self.vw // 2, (self.vh - 54) // 2
        glow = Image.new("RGBA", (self.vw, self.vh), (0, 0, 0, 0))
        gd = ImageDraw.Draw(glow)
        for rad, a in ((84, 22), (74, 38), (64, 60), (55, 95)):
            gd.ellipse([cx - rad, cy - rad, cx + rad, cy + rad], outline=ACCENT + (a,), width=4)
        s, g = 54, 19
        for sx in (-1, 1):
            for sy in (-1, 1):
                x, y = cx + sx * s, cy + sy * s
                gd.line([x, y, x - sx * g, y], fill=ACCENT + (255,), width=3)
                gd.line([x, y, x, y - sy * g], fill=ACCENT + (255,), width=3)
        img.alpha_composite(glow)
        return img

    def draw(self, view, depth, scene, judg, tel):
        img = Image.new("RGBA", (self.W, self.H), BG + (255,))
        img.paste(Image.fromarray(view).convert("RGBA"), (0, 0))
        img = self._lock_on(img)
        dr = ImageDraw.Draw(img)
        px = self.vw
        dr.rectangle([px, 0, self.W, self.H], fill=PANEL)
        dr.line([px, 0, px, self.H], fill=LINE, width=1)
        x0, w = px + 22, self.W_PANEL - 44
        fresh = judg.get("age_s", 9) < 0.45 and judg["source"] == "jev"

        # ---- header ------------------------------------------------------
        y = 26
        dr.text((x0, y), "JEV", font=font(30, True), fill=ACCENT)
        dr.text((x0 + 62, y + 10), "SYSTEM ONE", font=font(13, True), fill=TEXT)
        dr.text((x0 + 62, y + 27), tel["model"], font=font(11), fill=DIM)
        dot = ACCENT if fresh else (70, 55, 30)
        dr.ellipse([self.W - 40, y + 12, self.W - 28, y + 24], fill=dot)
        y += 62
        dr.line([x0, y, x0 + w, y], fill=LINE)

        # ---- onboard camera ---------------------------------------------
        y += 16
        dr.text((x0, y), "ONBOARD CAMERA  64x48 DEPTH", font=font(11, True), fill=CYAN)
        y += 18
        iw = int(w * 0.78)
        dh = int(iw * depth.shape[0] / depth.shape[1])
        img.paste(Image.fromarray(_depth_rgb(depth)).resize((iw, dh), Image.NEAREST), (x0, y))
        dr.rectangle([x0, y, x0 + iw, y + dh], outline=LINE)
        t = scene["target"]
        if t["visible"]:                                   # lock reticle
            cx = x0 + iw * (0.5 - np.tan(np.deg2rad(t["bearing_deg"])) / (2 * tel["tan_h"]))
            cy = y + dh * 0.52
            dr.rectangle([cx - 13, cy - 13, cx + 13, cy + 13], outline=GREEN, width=2)
            dr.text((cx + 18, cy - 7), f"{t['range_m']:.1f}m", font=font(12, True), fill=GREEN)
        y += dh + 14

        # ---- free space --------------------------------------------------
        dr.text((x0, y), "FREE SPACE BY SECTOR", font=font(11, True), fill=CYAN)
        y += 18
        secs = scene["sector_range_m"]
        step = 15 if len(secs) <= 5 else 12
        for name, rng in secs.items():
            col = RED if rng < 4.0 else (ACCENT if rng < 10.0 else CYAN)
            dr.text((x0, y), name.replace("_", " ")[:11].ljust(11), font=font(10), fill=DIM)
            self._bar(dr, x0 + 84, y + 2, w - 136, 7, rng / 30.0, col)
            dr.text((x0 + w - 44, y), f"{min(rng,99):5.1f}", font=font(10), fill=TEXT)
            y += step
        # --- vertical free space: what decides over / under / around ----------
        y += 8
        dr.text((x0, y), "FREE SPACE AHEAD, STACKED", font=font(11, True), fill=CYAN)
        y += 16
        for lbl, key in (("if we climb", "free_ahead_above_m"),
                         ("straight on", "free_ahead_level_m"),
                         ("if we dive", "free_ahead_below_m")):
            v = scene.get(key)
            if v is None:
                continue
            col = RED if v < 5.0 else (ACCENT if v < 12.0 else GREEN)
            dr.text((x0, y), lbl.ljust(11), font=font(10), fill=DIM)
            self._bar(dr, x0 + 84, y + 2, w - 136, 7, v / 30.0, col)
            dr.text((x0 + w - 44, y), f"{v:5.1f}", font=font(10), fill=TEXT)
            y += 14
        y += 18
        dr.line([x0, y, x0 + w, y], fill=LINE)

        # ---- the judgment -------------------------------------------------
        y += 16
        dr.text((x0, y), "TACTICAL JUDGMENT", font=font(11, True), fill=ACCENT)
        if judg["source"] != "jev":
            dr.text((x0 + w - 62, y), "FALLBACK", font=font(11, True), fill=DIM)
        y += 22
        if "steer" in judg:                       # graded-score navigator
            for lbl, val, lo, hi, a, b in (
                    ("steer", judg["steer"], 0.0, 4.0, "hard left", "hard right"),
                    ("height", judg["height"], 0.0, 4.0, "dive", "climb")):
                dr.text((x0, y), lbl.upper(), font=font(11, True), fill=ACCENT)
                dr.text((x0 + 70, y), a, font=font(9), fill=DIM)
                dr.text((x0 + w - 56, y), b, font=font(9), fill=DIM)
                y += 14
                bx, bw = x0, w
                dr.rectangle([bx, y, bx + bw, y + 12], fill=(28, 34, 41))
                dr.line([bx + bw // 2, y - 2, bx + bw // 2, y + 14], fill=(70, 82, 95))
                frac = (val - lo) / (hi - lo)
                cx = bx + int(bw * min(max(frac, 0.0), 1.0))
                mid = bx + bw // 2
                dr.rectangle([min(cx, mid), y, max(cx, mid), y + 12], fill=ACCENT)
                dr.ellipse([cx - 6, y - 2, cx + 6, y + 14], fill=(255, 210, 120))
                dr.text((x0 + w - 40, y + 16), "%.2f" % val, font=font(10), fill=TEXT)
                y += 34
            y += 4
        probs = judg.get("probabilities") or {}
        order = [k for k in ("hold_course", "gap_left", "gap_right", "climb", "dive",
                             "brake", "reacquire") if k in probs]
        for k in order:
            p = probs.get(k, 0.0)
            self.smooth[k] = 0.65 * self.smooth.get(k, 0.0) + 0.35 * p   # ease the bars
            p = self.smooth[k]
            chosen = k == judg["maneuver"] and judg["source"] == "jev"
            col = ACCENT if chosen else (58, 70, 82)
            dr.text((x0, y), k.replace("_", " ").ljust(11), font=font(12, True if chosen else False),
                    fill=TEXT if chosen else DIM)
            self._bar(dr, x0 + 104, y + 3, w - 156, 9, p, col)
            dr.text((x0 + w - 40, y), f"{p*100:3.0f}%", font=font(11), fill=TEXT if chosen else DIM)
            y += 20
        y += 6

        meters = [("risk", judg["risk"], 2.0, RED),
                  ("confidence", judg.get("confidence", 0.0), 1.0, GREEN)]
        if "target_truly_lost" in judg:
            meters.append(("target lost", judg["target_truly_lost"], 1.0, ACCENT))
        for label, val, vmax, col in meters:
            dr.text((x0, y), label.ljust(11), font=font(11), fill=DIM)
            self._bar(dr, x0 + 104, y + 2, w - 156, 8, val / vmax, col)
            dr.text((x0 + w - 40, y), f"{val:.2f}", font=font(11), fill=TEXT)
            y += 18

        # ---- footer -------------------------------------------------------
        fy = self.H - 74
        dr.line([x0, fy - 12, x0 + w, fy - 12], fill=LINE)
        for i, s in enumerate([
            f"calls {tel['calls']:<4} reused {tel['skipped']}",
            f"latency {tel['lat']}  tokens {tel['tokens']}",
            f"control 500Hz   jev {tel['hz']:.0f}Hz",
        ]):
            dr.text((x0, fy + i * 16), s, font=font(11), fill=DIM)

        # ---- mission strip over the render ---------------------------------
        dr.rectangle([0, self.vh - 54, self.vw, self.vh], fill=(8, 10, 13))
        items = [("T", f"{tel['t']:5.1f}s"), ("STANDOFF", f"{tel['standoff']:4.1f}m"),
                 ("SPEED", f"{tel['speed']:4.1f}m/s"), ("NEAREST", f"{scene['nearest_obstacle_m']:4.1f}m"),
                 ("CONTACTS", f"{tel['hits']}")]
        cx = 24
        for k, v in items:
            dr.text((cx, self.vh - 42), k, font=font(10, True), fill=DIM)
            dr.text((cx, self.vh - 28), v, font=font(15, True),
                    fill=RED if (k == "CONTACTS" and tel["hits"]) else TEXT)
            cx += 150
        dr.text((self.vw - 330, self.vh - 36), tel["mode"], font=font(14, True),
                fill=ACCENT if tel["mode"].startswith("JEV") else DIM)
        if tel.get("climbing"):
            dr.text((self.vw - 330, self.vh - 52), "CLIMBING OVER BARRIER", font=font(11, True), fill=GREEN)
        return np.asarray(img.convert("RGB"))
