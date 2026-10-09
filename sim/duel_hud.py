"""City duel HUD built on the upstream jev-drone Hud visual system."""
from __future__ import annotations

import numpy as np
from PIL import Image, ImageDraw

from hud import Hud, _depth_rgb, font, BG, PANEL, LINE, TEXT, DIM, ACCENT, CYAN, GREEN, RED
from duel_tactics import ACTIONS


class DuelHud(Hud):
    """Reuse the original lock-on effect, font, palette and animated bar style."""

    def _shot_effect(self, image, effect):
        if not effect or effect["age_s"] > .7:
            return
        age = effect["age_s"]
        overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        color = GREEN if effect["hit"] else ACCENT
        start, end = effect.get("start_xy"), effect.get("end_xy")
        if effect["cooldown_ok"] and start and end and age < .3:
            alpha = int(255*(1-age/.3))
            draw.line((*start, *end), fill=color+(alpha//3,), width=13)
            draw.line((*start, *end), fill=color+(alpha,), width=4)
            draw.line((*start, *end), fill=(255, 255, 240, alpha), width=1)
            radius = 10+age*35
            draw.ellipse((start[0]-radius, start[1]-radius, start[0]+radius,
                          start[1]+radius), outline=color+(alpha,), width=3)
        if effect["hit"] and end:
            radius = 14+age*65
            alpha = int(255*(1-age/.7))
            draw.ellipse((end[0]-radius, end[1]-radius, end[0]+radius,
                          end[1]+radius), outline=GREEN+(alpha,), width=3)
            for theta in np.linspace(0, 2*np.pi, 8, endpoint=False):
                a = (end[0]+radius*np.cos(theta), end[1]+radius*np.sin(theta))
                b = (end[0]+(radius+12)*np.cos(theta), end[1]+(radius+12)*np.sin(theta))
                draw.line((*a, *b), fill=GREEN+(alpha,), width=2)
        label = "HIT" if effect["hit"] else "NO HIT" if effect["cooldown_ok"] else "COOLDOWN"
        label_x, label_y = effect.get("label_xy", (24, 80))
        draw.rounded_rectangle((label_x, label_y, label_x+311, label_y+46), radius=6, fill=(8, 15, 22, 215),
                               outline=color+(220,), width=2)
        draw.text((label_x+14, label_y+13), f"SIM FIRE #{effect['number']}  /  {label}",
                  font=font(16, True), fill=color+(255,))
        image.alpha_composite(overlay)

    def _enemy_lock(self, image, xy):
        if xy is None:
            return
        cx, cy = xy
        if not (30 <= cx < self.vw-30 and 30 <= cy < self.vh-84):
            return
        glow = Image.new("RGBA", (self.vw, self.vh), (0, 0, 0, 0))
        gd = ImageDraw.Draw(glow)
        for radius, alpha in ((58, 22), (48, 40), (39, 75), (32, 125)):
            gd.ellipse((cx-radius, cy-radius, cx+radius, cy+radius),
                       outline=RED+(alpha,), width=3)
        for sx in (-1, 1):
            for sy in (-1, 1):
                x, y = cx+sx*39, cy+sy*39
                gd.line((x, y, x-sx*13, y), fill=RED+(250,), width=2)
                gd.line((x, y, x, y-sy*13), fill=RED+(250,), width=2)
        image.alpha_composite(glow)

    def draw(self, view, depth, scene, decision, telemetry, enemy_xy=None):
        image = Image.new("RGBA", (self.W, self.H), BG+(255,))
        image.paste(Image.fromarray(view).convert("RGBA"), (0, 0))
        image = self._lock_on(image)  # upstream orange balance / pursuit ring
        self._enemy_lock(image, enemy_xy)
        self._shot_effect(image, telemetry.get("shot_effect"))
        draw = ImageDraw.Draw(image)
        px, x0, width = self.vw, self.vw+22, self.W_PANEL-44
        draw.rectangle((px, 0, self.W, self.H), fill=PANEL)
        draw.line((px, 0, px, self.H), fill=LINE)

        y = 24
        draw.text((x0, y), "JEV", font=font(30, True), fill=ACCENT)
        draw.text((x0+64, y+9), "CITY AIR DUEL", font=font(13, True), fill=TEXT)
        draw.text((x0+64, y+28), "NeoHorse-Jev-4B · CAMERA ONLY", font=font(10), fill=DIM)
        draw.ellipse((self.W-40, y+13, self.W-28, y+25), fill=GREEN if telemetry["errors"] == 0 else RED)
        y += 62
        draw.line((x0, y, x0+width, y), fill=LINE)

        y += 12
        draw.text((x0, y), f"ONBOARD CAMERA  {depth.shape[1]}x{depth.shape[0]} DEPTH", font=font(11, True), fill=CYAN)
        y += 18
        iw = 304
        ih = round(iw*depth.shape[0]/depth.shape[1])
        image.paste(Image.fromarray(_depth_rgb(depth, 35)).resize((iw, ih), Image.Resampling.NEAREST), (x0, y))
        draw.rectangle((x0, y, x0+iw, y+ih), outline=LINE)
        target = scene["target"]
        if target["visible"]:
            # Bearing/elevation originate in the camera image; reticle is purely display.
            cx = x0 + iw*(.5-np.tan(np.deg2rad(target["bearing_deg"]))/(2*telemetry["tan_h"]))
            cy = y+ih*(.5-np.tan(np.deg2rad(target["elevation_deg"]))/(2*np.tan(np.deg2rad(55))))
            draw.rectangle((cx-10, cy-10, cx+10, cy+10), outline=GREEN, width=2)
            draw.text((x0+iw+9, y+5), "E-01", font=font(11, True), fill=RED)
            draw.text((x0+iw+9, y+25), f"{target['range_m']:.1f}m", font=font(11), fill=TEXT)
            draw.text((x0+iw+9, y+43), f"h {target['relative_height_m']:+.1f}", font=font(10), fill=TEXT)
        y += ih+13

        draw.text((x0, y), "FREE SPACE BY SECTOR", font=font(11, True), fill=CYAN)
        y += 17
        for name, distance in scene["sector_range_m"].items():
            color = RED if distance < 4 else ACCENT if distance < 10 else CYAN
            draw.text((x0, y), name.replace("_", " ").ljust(11), font=font(10), fill=DIM)
            self._bar(draw, x0+100, y+2, width-148, 7, distance/45., color)
            draw.text((x0+width-42, y), f"{distance:4.0f}m", font=font(10), fill=TEXT)
            y += 15
        y += 4
        draw.text((x0, y), "FREE SPACE BY HEIGHT", font=font(11, True), fill=CYAN)
        y += 17
        for label, key in (("UP", "free_ahead_above_m"),
                           ("LEVEL", "free_ahead_level_m"),
                           ("DOWN", "free_ahead_below_m")):
            distance = scene[key]
            color = RED if distance < 4 else ACCENT if distance < 10 else GREEN
            draw.text((x0, y), label.ljust(11), font=font(10), fill=DIM)
            self._bar(draw, x0+100, y+2, width-148, 7, distance/45., color)
            draw.text((x0+width-42, y), f"{distance:4.0f}m", font=font(10), fill=TEXT)
            y += 15
        y += 8
        draw.line((x0, y, x0+width, y), fill=LINE)
        y += 11
        draw.text((x0, y), f"JEV ACTION · ALL {len(ACTIONS)} OPTIONS", font=font(11, True), fill=ACCENT)
        y += 20
        probs = decision.get("probabilities") or {}
        chosen = decision.get("action")
        for action in ACTIONS:
            p = float(probs.get(action, 0.0))
            self.smooth[action] = .6*self.smooth.get(action, 0.0)+.4*p
            p = self.smooth[action]
            selected = action == chosen
            color = ACCENT if selected else (58, 70, 82)
            draw.text((x0, y), action.replace("_", " ").ljust(12), font=font(11, selected),
                      fill=TEXT if selected else DIM)
            self._bar(draw, x0+114, y+3, width-162, 9, p, color)
            draw.text((x0+width-42, y), f"{p*100:3.0f}%", font=font(10), fill=TEXT if selected else DIM)
            y += 19
        y += 7
        draw.line((x0, y, x0+width, y), fill=LINE)
        y += 10
        draw.text((x0, y), f"hits {telemetry['hits']}/10   shots {telemetry['shots']}   rain x{telemetry['rain_factor']:.2f}",
                  font=font(11), fill=GREEN if telemetry["hits"] else DIM)
        y += 17
        draw.text((x0, y), f"calls {telemetry['calls']}   latency {telemetry['latency_s']:.2f}s   errors {telemetry['errors']}",
                  font=font(11), fill=DIM)

        strip_y = self.vh-54
        draw.rectangle((0, strip_y, self.vw, self.vh), fill=(8, 10, 13))
        fields = (("T", f"{telemetry['time_s']:.1f}s"),
                  ("TARGET", f"{target['range_m']:.1f}m" if target["visible"] else "LOST"),
                  ("SPEED", f"{telemetry['speed_mps']:.1f}m/s"),
                  ("ALTITUDE", f"{telemetry['altitude_m']:.1f}m"),
                  ("HITS", f"{telemetry['hits']}/10"),
                  ("CONTACTS", str(telemetry["collisions"])))
        xx = 22
        for label, value in fields:
            draw.text((xx, strip_y+9), label, font=font(10, True), fill=DIM)
            draw.text((xx, strip_y+24), value, font=font(14, True),
                      fill=RED if label == "CONTACTS" and telemetry["collisions"] else TEXT)
            xx += 155
        draw.text((self.vw-240, strip_y+19), "JEV ACTION / SPEED GUARD", font=font(12, True), fill=ACCENT)
        return np.asarray(image.convert("RGB"))
