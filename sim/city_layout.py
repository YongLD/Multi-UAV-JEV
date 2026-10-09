"""Deterministic MuJoCo city geometry; available to the simulator, never to Jev."""
from __future__ import annotations

import random
from pathlib import Path


def make_buildings() -> list[dict]:
    rng = random.Random(1907)
    result = []
    for x in (-60, -30, 30, 60):
        for y in (-60, -30, 30, 60):
            half_x = rng.uniform(8.0, 10.8)
            half_y = rng.uniform(8.0, 10.8)
            height = rng.uniform(13.0, 27.0)
            result.append({"name": f"building_{len(result)+1:02d}",
                           "x": float(x), "y": float(y),
                           "half_x": round(half_x, 2), "half_y": round(half_y, 2),
                           "height": round(height, 2),
                           "material": f"facade_{len(result) % 4}"})
    return result


BUILDINGS = make_buildings()
RAIN_RADIUS_M = 12.0


def write_city_xml(path: str | Path) -> None:
    lines = [
        '<mujoco model="jev_drone_city_duel">',
        '  <include file="mujoco_menagerie/skydio_x2/x2.xml"/>',
        '  <statistic extent="150" center="0 0 14"/>',
        '  <option timestep="0.002" density="1.2" viscosity="1.8e-5"/>',
        '  <visual><global fovy="55" offwidth="1400" offheight="900"/>',
        '    <headlight diffuse=".8 .8 .8" ambient=".45 .45 .48"/>',
        '    <map znear="0.0005" zfar="2.0"/></visual>',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1=".41 .62 .79" rgb2=".11 .21 .35" width="512" height="512"/>',
        '    <texture name="ground_tex" type="2d" builtin="checker" rgb1=".19 .24 .29" rgb2=".23 .28 .33" width="512" height="512"/>',
        '    <material name="ground" texture="ground_tex" texrepeat="32 32" reflectance="0"/>',
        '    <material name="facade_0" rgba=".48 .57 .64 1"/>',
        '    <material name="facade_1" rgba=".62 .54 .46 1"/>',
        '    <material name="facade_2" rgba=".40 .50 .54 1"/>',
        '    <material name="facade_3" rgba=".58 .61 .57 1"/>',
        '    <material name="rain_blue" rgba=".25 .58 .92 .32"/>',
        '  </asset>',
        '  <worldbody>',
        '    <light pos="0 0 90" dir="0 0 -1" directional="true" diffuse=".8 .8 .8"/>',
        '    <geom name="floor" type="plane" size="160 160 .1" material="ground"/>',
    ]
    for building in BUILDINGS:
        b = building
        lines.append(
            f'    <geom name="{b["name"]}" type="box" material="{b["material"]}" '
            f'size="{b["half_x"]} {b["half_y"]} {b["height"]/2:.2f}" '
            f'pos="{b["x"]} {b["y"]} {b["height"]/2:.2f}"/>')
    lines.extend([
        '    <body name="enemy" mocap="true" pos="38 0 17">',
        '      <geom name="enemy_x2_visual" material="phong3SG" mesh="X2_lowpoly" class="visual" quat="0 0 1 1"/>',
        '      <geom name="enemy_camera_halo" type="sphere" size="1.3" rgba="1 .12 .08 .06" contype="0" conaffinity="0"/>',
        '    </body>',
        '    <body name="rain" mocap="true" pos="-14 32 0">',
        f'      <geom name="rain_visual" type="cylinder" size="{RAIN_RADIUS_M} .025" pos="0 0 .025" material="rain_blue" contype="0" conaffinity="0" group="5"/>',
        '    </body>',
        '  </worldbody>',
        '</mujoco>',
    ])
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    write_city_xml(Path(__file__).with_name("city.xml"))
