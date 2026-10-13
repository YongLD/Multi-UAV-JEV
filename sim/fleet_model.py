"""Native MuJoCo model assembly for independent physical pursuers."""
from pathlib import Path
import copy
import xml.etree.ElementTree as ET
import mujoco
from urban_scene import write_city_xml
from fleet_tracking import validate_count, COLORS
ROOT = Path(__file__).resolve().parent


def create_model(count):
    """Duplicate the native body, free joint and four motors; assets remain shared."""
    validate_count(count)
    source = ROOT/'mujoco_menagerie/skydio_x2/x2.xml'
    drone_xml = ET.parse(source).getroot()
    drone_xml.find('compiler').set('assetdir', str(source.parent/'assets'))
    keyframes = drone_xml.find('keyframe')
    if keyframes is not None:
        drone_xml.remove(keyframes)
    body = drone_xml.find("worldbody/body[@name='x2']")
    motors = list(drone_xml.find('actuator'))
    sensors = list(drone_xml.find('sensor'))
    for index in range(1, count):
        prefix = f'x2_{index+1}__'
        duplicate = copy.deepcopy(body)
        mapping = {node.get('name'): prefix+node.get('name') for node in duplicate.iter() if node.get('name')}
        mapping['x2'] = f'x2_{index+1}'
        for node in duplicate.iter():
            if node.get('name'):
                node.set('name', mapping[node.get('name')])
            for attr in ('target', 'site', 'body', 'objname', 'joint'):
                if node.get(attr) in mapping:
                    node.set(attr, mapping[node.get(attr)])
        drone_xml.find('worldbody').append(duplicate)
        for original in motors+sensors:
            node = copy.deepcopy(original)
            if node.get('name'):
                node.set('name', prefix+node.get('name'))
            for attr in ('site', 'body', 'objname', 'joint'):
                if node.get(attr) in mapping:
                    node.set(attr, mapping[node.get(attr)])
            drone_xml.find('actuator' if original in motors else 'sensor').append(node)
    for index, node in enumerate(drone_xml.find('worldbody').findall('body')):
        color = ' '.join(str(c/255) for c in COLORS[index])
        ET.SubElement(node, 'geom', name=f'fleet_marker_{index}', type='sphere', size='.18',
                      pos='0 0 .2', rgba=color+' .8', contype='0', conaffinity='0', mass='0', group='2')
    include = ROOT/f'fleet_x2_{count}.xml'
    ET.ElementTree(drone_xml).write(include, encoding='unicode')
    scene_path = ROOT/f'city_fleet_{count}.xml'
    write_city_xml(scene_path)
    scene = ET.parse(scene_path)
    scene.getroot().find('include').set('file', include.name)
    scene.write(scene_path, encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(scene_path))
