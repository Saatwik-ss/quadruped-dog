#!/usr/bin/env python3
import yaml
import xml.etree.ElementTree as ET
import sys

def body_lookup(world, name):
    for b in world.findall('body'):
        if b.get('name') == name:
            return b
    raise KeyError(f"Body {name} not found")

def yaml_to_mjcf(yaml_path, xml_out):
    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    mujoco = ET.Element('mujoco', attrib={'model': cfg['robot']['name']})
    ET.SubElement(mujoco, 'option',
                  attrib={'gravity': ' '.join(map(str, cfg['robot']['gravity']))})
    world = ET.SubElement(mujoco, 'worldbody')

    # Links → bodies/geoms
    for link in cfg['links']:
        body = ET.SubElement(world, 'body', name=link['name'], pos="0 0 0")
        ET.SubElement(body, 'geom', type='box',
                      size=' '.join(map(str, [s/2 for s in link['size']])),
                      mass=str(link['mass']))

    # Joints
    for j in cfg['joints']:
        parent = body_lookup(world, j['parent'])
        ET.SubElement(parent, 'joint',
                      name=j['name'], type=j['type'],
                      axis=' '.join(map(str, j['axis'])),
                      range=' '.join(map(str, j['range'])))

    # Actuators
    actuators = ET.SubElement(mujoco, 'actuator')
    for a in cfg['actuators']:
        ET.SubElement(actuators, 'motor',
                      name=a['name'], joint=a['joint'],
                      gear=str(a['gear']),
                      ctrllimited=str(a['ctrllimited']).lower(),
                      ctrlrange=' '.join(map(str, a['ctrlrange'])))

    tree = ET.ElementTree(mujoco)
    tree.write(xml_out, encoding='utf-8', xml_declaration=True)
    print(f"Generated MJCF XML at: {xml_out}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: yaml2mjcf.py <input.yaml> <output.xml>")
        sys.exit(1)
    yaml_to_mjcf(sys.argv[1], sys.argv[2])
