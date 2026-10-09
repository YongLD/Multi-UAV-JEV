"""A textured waterfront district assembled from CC0 Kenney building meshes.

Only the simulator and spectator cameras import this module. Jev still receives
the explicit onboard observation allowlist in duel_tactics.build_state.
"""
from __future__ import annotations

import json
import random
from pathlib import Path

import numpy as np
from PIL import Image

from city_layout import BUILDINGS as INNER_BUILDINGS, RAIN_RADIUS_M

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "urban_assets"
MODEL_NAMES = ("building-skyscraper-a", "building-f", "building-j", "building-n",
               "building-i", "building-l", "building-skyscraper-e", "building-a",
               "building-skyscraper-b", "building-c", "building-b", "building-skyscraper-d")


def district():
    result = [dict(b) for b in INNER_BUILDINGS]
    rng = random.Random(4401)
    # Background blocks make the district read as a city from an elevated view.
    for x, y in [(-112, y) for y in (-66, -22, 22, 66)] + [(112, y) for y in (-66, -22, 22, 66)] + [(x, y) for y in (-112, 112) for x in (-66, -22, 22, 66)]:
        result.append({"name": f"outer_{len(result):02}", "x": x, "y": y,
                       "half_x": rng.uniform(10, 13), "half_y": rng.uniform(9, 12),
                       "height": rng.uniform(18, 46)})
    for i, b in enumerate(result):
        b["model"] = MODEL_NAMES[i % len(MODEL_NAMES)]
    return result


BUILDINGS = district()


def write_city_xml(path):
    manifest = json.loads((ASSETS / "manifest.json").read_text())
    asphalt = ASSETS / "asphalt.png"
    if not asphalt.exists():
        noise = np.random.default_rng(32).normal(0, 2.0, (256, 256, 1))
        Image.fromarray(np.clip(np.array([52, 61, 66]) + noise, 0, 255).astype("uint8")).save(asphalt)
    lines = [
        '<mujoco model="jev_waterfront_district">',
        '<include file="mujoco_menagerie/skydio_x2/x2.xml"/>',
        '<statistic extent="300" center="0 0 18"/>',
        '<option timestep="0.002" density="1.2" viscosity="1.8e-5"/>',
        '<visual><global fovy="50" offwidth="1400" offheight="900"/>',
        '<quality shadowsize="4096" offsamples="0"/><headlight diffuse=".45 .45 .45" ambient=".32 .35 .38"/>',
        '<map znear="0.0002" zfar="4" haze=".8"/><rgba haze=".65 .76 .82 1"/></visual>',
        '<asset>',
        '<texture type="skybox" builtin="gradient" rgb1=".48 .68 .83" rgb2=".87 .91 .91" width="512" height="512"/>',
        f'<texture name="city_palette" type="2d" file="{ASSETS}/colormap.png"/>',
        '<material name="city_palette_mat" texture="city_palette" rgba=".73 .77 .79 1" specular=".10" shininess=".15"/>',
        f'<texture name="asphalt_tex" type="2d" file="{ASSETS}/asphalt.png"/>',
        '<material name="asphalt" texture="asphalt_tex" texrepeat="16 16" specular=".07"/>',
        '<material name="land" rgba=".34 .43 .35 1"/>',
        '<material name="water" rgba=".19 .41 .52 1" specular=".6" shininess=".5"/>',
        '<material name="sidewalk" rgba=".64 .67 .65 1"/>',
        '<material name="roof_green" rgba=".21 .35 .24 1"/>',
        '<material name="rain_blue" rgba=".25 .58 .92 .23"/>',
    ]
    for b in BUILDINGS:
        source = manifest[b["model"]]
        sx, sy, sz = 2*b["half_x"]/source["width"], 2*b["half_y"]/source["depth"], b["height"]/source["height"]
        lines.append(f'<mesh name="{b["name"]}_mesh" file="{ASSETS}/{b["model"]}.obj" scale="{sx:.5f} {sy:.5f} {sz:.5f}"/>')
    lines.extend(['</asset>', '<worldbody>',
        '<light pos="-100 -80 150" dir=".5 .4 -1" directional="true" diffuse=".65 .63 .59" castshadow="true"/>',
        '<geom name="floor" type="plane" size="800 800 .1" pos="0 0 -.5" material="water"/>',
        '<geom name="district_ground" type="box" size="146 146 .24" pos="0 0 -.24" material="land"/>',
        '<geom type="box" size="148 148 .2" pos="0 0 -.45" material="sidewalk"/>'])

    def visual(kind, pos, size, material=None, color=None):
        style = f'material="{material}"' if material else f'rgba="{color}"'
        lines.append(f'<geom type="{kind}" pos="{pos}" size="{size}" {style} contype="0" conaffinity="0" group="1"/>')

    for axis in (0, 1):
        for road in (-135, -90, -45, 0, 45, 90, 135):
            pos = f'{road} 0 .01' if axis == 0 else f'0 {road} .01'
            size = '5.4 146 .012' if axis == 0 else '146 5.4 .012'
            visual('box', pos, size, 'asphalt')
            for v in range(-141, 145, 9):
                p = f'{road} {v} .034' if axis == 0 else f'{v} {road} .034'
                ss = '.09 1.8 .007' if axis == 0 else '1.8 .09 .007'
                visual('box', p, ss, color='.91 .83 .49 1')
            for side in (-1, 1):
                p = f'{road+side*5.2} 0 .037' if axis == 0 else f'0 {road+side*5.2} .037'
                ss = '.08 146 .008' if axis == 0 else '146 .08 .008'
                visual('box', p, ss, color='.87 .9 .89 1')

    for b in BUILDINGS:
        x, y, hx, hy, h, name = (b[k] for k in ("x", "y", "half_x", "half_y", "height", "name"))
        # Collision uses the imported mesh's convex hull, not its whole bounding
        # box. The old box filled setback airspace that the camera saw as empty.
        # The hull remains conservative for small window recesses; onboard
        # clearance margins cover those instead of inventing invisible cube walls.
        lines.append(f'<geom name="{name}" type="mesh" mesh="{name}_mesh" pos="{x} {y} 0" rgba="0 0 0 0" group="3"/>')
        lines.append(f'<geom name="{name}_visual" type="mesh" mesh="{name}_mesh" pos="{x} {y} 0" material="city_palette_mat" contype="0" conaffinity="0" group="1"/>')
        visual('box', f'{x} {y} .03', f'{hx+1:.3f} {hy+1:.3f} .04', 'sidewalk')
        # Trees and planters remain below the normal flight layer.
        for side in (-1, 1):
            tx, ty = x+side*(hx+.5), y+hy+.4
            visual('box', f'{tx:.2f} {ty:.2f} .3', '.7 .7 .3', color='.40 .44 .43 1')
            visual('cylinder', f'{tx:.2f} {ty:.2f} 1.5', '.12 1.2', color='.32 .25 .19 1')
            visual('ellipsoid', f'{tx:.2f} {ty:.2f} 2.9', '1.2 1.1 1.65', color='.24 .40 .25 1')
        # A few stationary cars make the roads legible at flying altitude.
        if name.startswith('building'):
            visual('box', f'{x-hx-.4:.2f} {y:.2f} .62', '1.0 2.1 .5', color='.51 .61 .68 1')
            visual('box', f'{x-hx-.4:.2f} {y-.1:.2f} 1.18', '.85 1.12 .32', color='.19 .28 .34 1')
    # Pedestrian crossings in the central avenue.
    for road in (-45, 0, 45):
        for offset in (-7.3, 7.3):
            for stripe in np.arange(-4.2, 4.5, 1.4):
                visual('box', f'{road+stripe:.2f} {offset} .039', '.38 1.4 .006', color='.88 .90 .87 1')

    lines.extend([
        '<body name="enemy" mocap="true" pos="38 0 17">',
        '<geom name="enemy_x2_visual" material="phong3SG" mesh="X2_lowpoly" class="visual" quat="0 0 1 1"/>',
        '<geom name="enemy_camera_halo" type="sphere" size="1.3" rgba="1 .12 .08 .06" contype="0" conaffinity="0"/>',
        '</body>', '<body name="rain" mocap="true" pos="-14 32 0">',
        f'<geom name="rain_visual" type="cylinder" size="{RAIN_RADIUS_M} .025" pos="0 0 .07" material="rain_blue" contype="0" conaffinity="0" group="5"/>',
        '</body>', '</worldbody>', '</mujoco>'])
    Path(path).write_text('\n'.join(lines)+'\n')


if __name__ == '__main__':
    write_city_xml(ROOT/'city_urban.xml')
