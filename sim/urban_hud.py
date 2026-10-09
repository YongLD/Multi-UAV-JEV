"""Full-frame flight HUD; preserves the original balance rings and firing FX.

The browser draws accessible live instruments over a clean scene stream. This
matching compositor bakes instruments into the retained replay video.
"""
from __future__ import annotations

import math
import numpy as np
from PIL import Image, ImageDraw

from duel_hud import DuelHud
from hud import font, TEXT, DIM, ACCENT, CYAN, GREEN, RED, _depth_rgb
from duel_tactics import ACTIONS


class UrbanHud(DuelHud):
    W_PANEL = 0

    def decorate(self, view, telemetry, enemy_xy):
        image = Image.fromarray(view).convert('RGBA')
        overlay = Image.new('RGBA', image.size)
        draw = ImageDraw.Draw(overlay)
        cx, cy = self.vw/2, self.vh/2
        for radius, alpha in ((65, 22), (57, 38), (49, 60), (42, 105)):
            draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline=ACCENT+(alpha,), width=3)
        for sx in (-1, 1):
            for sy in (-1, 1):
                x, y = cx+sx*42, cy+sy*42
                draw.line((x, y, x-sx*14, y), fill=ACCENT+(255,), width=2)
                draw.line((x, y, x, y-sy*14), fill=ACCENT+(255,), width=2)
        trajectory = telemetry.get('trajectory_overlay') or {}
        history = [p for p in trajectory.get('history_xy', []) if p]
        if len(history) >= 2:
            draw.line(history, fill=CYAN+(105,), width=2)
        previous = history[-1] if history else None
        for prediction in trajectory.get('predicted_xy', []):
            xy = prediction['xy']
            if not xy or prediction['confidence'] < .1:
                continue
            alpha = int(70+170*prediction['confidence'])
            if previous:
                for start in np.arange(0., 1., .14):
                    end = min(start+.07, 1.)
                    a = (previous[0]+(xy[0]-previous[0])*start, previous[1]+(xy[1]-previous[1])*start)
                    b = (previous[0]+(xy[0]-previous[0])*end, previous[1]+(xy[1]-previous[1])*end)
                    draw.line((*a, *b), fill=CYAN+(alpha,), width=2)
            draw.ellipse((xy[0]-5, xy[1]-5, xy[0]+5, xy[1]+5), outline=CYAN+(alpha,), width=2)
            draw.text((xy[0]+8, xy[1]-6), f"EST +{prediction['horizon_s']:.0f}s", font=font(10, True), fill=CYAN+(alpha,))
            previous = xy
        image.alpha_composite(overlay)
        self._enemy_lock(image, enemy_xy)
        shot = telemetry.get('shot_effect')
        self._shot_effect(image, {**shot, 'label_xy': (self.vw/2-155, 63)} if shot else None)
        return image

    def draw(self, view, depth, scene, decision, telemetry, enemy_xy=None, *, rgb=None, overview=None):
        image = self.decorate(view, telemetry, enemy_xy)
        overlay = Image.new('RGBA', image.size)
        d = ImageDraw.Draw(overlay)

        def panel(x, y, w, h):
            d.rounded_rectangle((x, y, x+w, y+h), radius=12, fill=(15, 25, 34, 225), outline=(125, 153, 171, 85))

        def text(x, y, value, size=11, color=TEXT, bold=False):
            d.text((x, y), str(value), font=font(size, bold), fill=color+(255,))

        def bar(x, y, w, h, value, color):
            d.rounded_rectangle((x, y, x+w, y+h), radius=2, fill=(60, 76, 86, 150))
            if value > 0:
                d.rounded_rectangle((x, y, x+max(2, w*min(value, 1)), y+h), radius=2, fill=color+(240,))

        panel(16, 16, 214, 239)
        text(30, 28, 'P-01 / FLIGHT INSTRUMENTS', 10, CYAN, True)
        text(28, 49, f"{telemetry['altitude_m']:.1f}", 38, TEXT, True)
        text(149, 76, 'm AGL', 11, DIM)
        text(30, 104, f"SPEED {telemetry['speed_mps']:.1f} m/s", 12)
        text(30, 125, f"V/S {telemetry.get('vertical_speed_mps', 0):+.1f} m/s", 11, DIM)
        text(30, 147, f"HDG {telemetry.get('yaw_deg', 0)%360:03.0f} deg", 12)
        text(30, 169, f"ROLL {telemetry.get('roll_deg', 0):+.0f}   PITCH {telemetry.get('pitch_deg', 0):+.0f}", 10, DIM)
        bar(30, 198, 185, 6, min(telemetry['speed_mps']/8, 1), CYAN)
        text(30, 215, f"RAIN x{telemetry['rain_factor']:.2f}   CONTACTS {telemetry['collisions']}", 10, DIM)
        panel(447, 16, 387, 33)
        text(461, 26, 'MULTI-UAV JEV / WATERFRONT PURSUIT', 13, TEXT, True)
        panel(990, 16, 274, 95)
        text(1006, 29, 'FLIGHT SESSION', 10, DIM, True)
        text(1006, 52, telemetry.get('model_name', 'NeoHorse-Jev-4B')+' / LIVE', 13, CYAN, True)
        text(1006, 81, 'RGB + DEPTH STATE' if telemetry.get('model_input_mode')=='rgb' else 'ONBOARD TEXT STATE', 10, DIM)
        panel(990, 124, 274, 237)
        text(1006, 138, 'ONBOARD / FRONT / RGB + DEPTH', 10, CYAN, True)
        image.alpha_composite(overlay)
        if rgb is not None:
            image.paste(Image.fromarray(rgb).resize((118, 89)), (1006, 162))
        image.paste(Image.fromarray(_depth_rgb(depth, 35)).resize((118, 89), Image.Resampling.NEAREST), (1130, 162))
        overlay = Image.new('RGBA', image.size)
        d = ImageDraw.Draw(overlay)
        target = scene['target']
        text(1006, 259, 'E-01 / '+str(target.get('camera') or 'front').upper()+' OBS' if target['visible'] else 'E-01 OCCLUDED', 11, GREEN if target['visible'] else ACCENT)
        text(1006, 281, f"RANGE {target['range_m']:.1f}m  DH {target['relative_height_m']:+.1f}m" if target['visible'] else 'Last sighting: camera only', 10, DIM)
        for i, distance in enumerate(scene['sector_range_m'].values()):
            bar(1006+i*49, 309, 40, 7, distance/70, RED if distance < 4 else ACCENT if distance < 20 else CYAN)
        text(1006, 329, 'L2    L1    FRONT    R1    R2', 10, DIM)
        panel(16, 377, 278, 326)
        text(31, 393, 'JEV / ALL ACTIONS', 11, CYAN, True)
        text(191, 395, 'P / COUNT', 9, DIM)
        counts = telemetry['counts']
        total = max(sum(counts.values()), 1)
        probabilities = decision.get('probabilities') or {}
        colors = (CYAN, CYAN, (161, 148, 238), (161, 148, 238), GREEN, RED)
        for i, action in enumerate(ACTIONS):
            y = 418+i*26
            selected = action == decision.get('action')
            text(31, y, action.replace('_', ' '), 11, TEXT if selected else DIM, selected)
            value = float(probabilities.get(action, 0))
            bar(120, y+5, 75, 6, value, colors[i])
            text(202, y, f'{value*100:2.0f}% / {counts[action]}', 10, TEXT if selected else DIM)
        text(31, 610, 'TASK SHARE: '+ ' / '.join(f'{counts[a]/total*100:.0f}' for a in ACTIONS), 9, DIM)
        text(31, 635, f"{telemetry['calls']} CALLS   {telemetry['latency_s']*1000:.0f}ms   {telemetry['errors']} ERR", 10)
        text(31, 659, f"HITS {telemetry['hits']}/10   SHOTS {telemetry['shots']}", 14, GREEN, True)
        panel(714, 583, 232, 120)
        text(730, 598, 'EPISODE METRICS', 10, DIM, True)
        text(730, 622, f"{telemetry['time_s']:.1f}s   {telemetry.get('distance_flown_m', 0):.0f}m flown", 12)
        text(730, 646, f"{telemetry['hits']} hits / {telemetry['shots']} shots", 12, GREEN)
        text(730, 670, f"{telemetry['collisions']} contacts / rain x{telemetry['rain_factor']:.2f}", 10, DIM)
        panel(962, 477, 302, 226)
        text(978, 491, 'DISTRICT OVERVIEW / REFEREE', 10, DIM, True)
        image.alpha_composite(overlay)
        if overview is not None:
            image.paste(Image.fromarray(overview).resize((274, 156)), (976, 516))
        overlay = Image.new('RGBA', image.size)
        d = ImageDraw.Draw(overlay)
        text(979, 684, 'P-01 ORANGE   E-01 RED   RAIN BLUE', 9, DIM)
        image.alpha_composite(overlay)
        return np.asarray(image.convert('RGB'))


def decorate_overview(view, camera, chaser_pos, enemy_pos, chaser_yaw, enemy_yaw, trails, rain_xy):
    """Spectator-only top camera markers. Never part of the perception path."""
    image = Image.fromarray(view).convert('RGBA')
    draw = ImageDraw.Draw(image)
    h, w = view.shape[:2]
    cam = camera
    forward, up = np.array(cam.forward), np.array(cam.up)
    right = np.cross(forward, up)
    focal = h/(2*math.tan(math.radians(24)))

    def project(pos):
        rel = np.array(pos)-np.array(cam.pos)
        depth = rel@forward
        if depth <= .1:
            return None
        return (w/2+(rel@right)/depth*focal, h/2-(rel@up)/depth*focal)

    for points, color in zip(trails, (ACCENT, RED)):
        line = [project(p) for p in points]
        line = [p for p in line if p]
        if len(line) > 1:
            draw.line(line, fill=color+(180,), width=2)
    for pos, yaw, color, label in ((chaser_pos, chaser_yaw, ACCENT, 'P'), (enemy_pos, enemy_yaw, RED, 'E')):
        xy = project(pos)
        nose = project(np.array(pos)+np.array([math.cos(yaw)*5, math.sin(yaw)*5, 0]))
        if xy and nose:
            draw.ellipse((xy[0]-5, xy[1]-5, xy[0]+5, xy[1]+5), fill=color+(255,), outline=(255, 255, 255, 230))
            draw.line((*xy, *nose), fill=color+(255,), width=3)
            draw.text((xy[0]+8, xy[1]-7), label, font=font(11, True), fill=(255, 255, 255, 255))
    return np.asarray(image.convert('RGB'))
