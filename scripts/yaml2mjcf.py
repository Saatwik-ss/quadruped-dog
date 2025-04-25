## COMMENT EACH NEW STEP ##

import yaml
import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import argparse
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

#function to convert list/tuple to space-separated string for XML attributes
def format_list(data):
    if isinstance(data, bool): # Specific check for booleans
        return str(data).lower() # Convert True -> "true", False -> "false" (dont forget) 
    if isinstance(data, (list, tuple)):
        # Ensure all elements in list/tuple are also formatted correctly (handles nested lists/bools if needed)
        return " ".join(map(format_list, data))
    # For non-bool, non-list/tuple types, just convert to string
    return str(data)

#function for pretty printing XML
def prettify_xml(elem):
    """Return a pretty-printed XML string for the Element."""
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def create_mjcf_from_yaml(yaml_path, output_xml_path):
    """
    Reads a YAML configuration and generates a MuJoCo MJCF XML file.
    """
    logging.info(f"Loading YAML configuration from: {yaml_path}")
    try:
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        logging.error(f"Error: YAML file not found at {yaml_path}")
        return
    except yaml.YAMLError as e:
        logging.error(f"Error parsing YAML file: {e}")
        return

    logging.info("Generating MJCF XML structure...")

    # Root element
    mujoco_attribs = {"model": config.get("robot_name", "Unnamed Robot")}
    root = ET.Element("mujoco", attrib=mujoco_attribs)

    # Compiler
    compiler_config = config.get("compiler", {})
    compiler_attribs = {k: format_list(v) for k, v in compiler_config.items()}
    ET.SubElement(root, "compiler", attrib=compiler_attribs)

    # Options
    sim_config = config.get("simulation", {})
    option_attribs = {
        "timestep": format_list(sim_config.get("timestep", 0.002)),
        "integrator": sim_config.get("integrator", "RK4")
    }
    option_el = ET.SubElement(root, "option", attrib=option_attribs)
    # Handle flags within option
    if sim_config.get("gravity"): # Assuming gravity is just enabled/disabled here
         ET.SubElement(option_el, "flag", attrib={"gravity": "enable"})
         # Note: Actual gravity vector is usually set globally or not needed in <option>

    # Visual - Basic implementation
    visual_el = ET.SubElement(root, "visual")
    ET.SubElement(visual_el, "headlight", attrib={"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
    ET.SubElement(visual_el, "map", attrib={"znear": "0.01"})
    ET.SubElement(visual_el, "quality", attrib={"shadowsize": "4096"})
    ET.SubElement(visual_el, "global", attrib={"offwidth": "2560", "offheight": "1440"}) # Example values

    # Assets - Basic textures and materials from YAML
    asset_el = ET.SubElement(root, "asset")
    ET.SubElement(asset_el, "texture", attrib={"type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7", "rgb2": "0 0 0", "width": "512", "height": "3072"})
    ET.SubElement(asset_el, "texture", attrib={"type": "2d", "name": "groundplane", "builtin": "checker", "mark": "edge", "rgb1": "0.2 0.3 0.4", "rgb2": "0.1 0.2 0.3", "markrgb": "0.8 0.8 0.8", "width": "300", "height": "300"})
    ET.SubElement(asset_el, "material", attrib={"name": "groundplane", "texture": "groundplane", "texuniform": "true", "texrepeat": "5 5", "reflectance": "0.2"})

    if "visuals" in config and "materials" in config["visuals"]:
        for mat_name, mat_props in config["visuals"]["materials"].items():
             mat_attribs = {"name": mat_name}
             mat_attribs.update({k: format_list(v) for k, v in mat_props.items()})
             ET.SubElement(asset_el, "material", attrib=mat_attribs)

    # Defaults
    default_el = ET.SubElement(root, "default")
    defaults_config = config.get("defaults", {})
    if "joint" in defaults_config:
        joint_defaults = {k: format_list(v) for k, v in defaults_config["joint"].items()}
        ET.SubElement(default_el, "joint", attrib=joint_defaults)
    if "geom" in defaults_config:
        geom_defaults = {k: format_list(v) for k, v in defaults_config["geom"].items()}
        ET.SubElement(default_el, "geom", attrib=geom_defaults)
    if "motor" in defaults_config:
        motor_defaults = {k: format_list(v) for k, v in defaults_config["motor"].items()}
        ET.SubElement(default_el, "motor", attrib=motor_defaults)

    # Worldbody
    worldbody_el = ET.SubElement(root, "worldbody")
    # Ground
    ET.SubElement(worldbody_el, "geom", attrib={"name": "ground", "type": "plane", "size": "5 5 0.1", "material": "groundplane", "condim": "3", "friction": "1.0 0.1 0.1"})
    ET.SubElement(worldbody_el, "light", attrib={"pos": "0 0 1.5", "dir": "0 0 -1", "diffuse": "0.8 0.8 0.8", "specular": "0.1 0.1 0.1", "castshadow": "true"})

    # Robot Body
    robot_config = config.get("robot_body", {})
    torso_config = robot_config.get("torso", {})
    legs_config = robot_config.get("legs", {})
    common_leg_params = {k: v for k, v in legs_config.items() if not isinstance(v, dict)} # Extract common params

    # --- Torso ---
    torso_pos = format_list(torso_config.get("pos", [0, 0, 0.3]))
    torso_el = ET.SubElement(worldbody_el, "body", attrib={"name": torso_config.get("name", "torso"), "pos": torso_pos})
    ET.SubElement(torso_el, "freejoint", attrib={"name": "root"})
    torso_geom_attribs = {
        "type": torso_config.get("geom_type", "box"),
        "size": format_list(torso_config.get("size", [0.2, 0.07, 0.04])),
        "mass": format_list(torso_config.get("mass", 5.0))
        # Add material density and others in future etc.
    }
    ET.SubElement(torso_el, "geom", attrib=torso_geom_attribs)
    if "inertial" in torso_config: # Optional manual inertial
        inertial_attribs = {k: format_list(v) for k, v in torso_config["inertial"].items() if k != 'diag'}
        inertial_attribs["diaginertia"] = format_list(torso_config["inertial"].get("diag", [0.1, 0.1, 0.1]))
        ET.SubElement(torso_el, "inertial", attrib=inertial_attribs)


    # Legs
    leg_definitions = {k: v for k, v in legs_config.items() if isinstance(v, dict)} # Get specific leg configs
    all_joints = [] # To store joint info for actuators/sensors
    all_motors = [] # To store motor info

    for leg_name, leg_conf in leg_definitions.items():
        logging.debug(f"Generating leg: {leg_name}")
        hip_attach_pos = leg_conf.get("hip_attach_pos", [0,0,0])
        sign_y = leg_conf.get("sign_y", 1)
        abad_len = common_leg_params.get("abad_link_length", 0.05)
        thigh_len = common_leg_params.get("thigh_length", 0.2)
        calf_len = common_leg_params.get("calf_length", 0.2)
        link_radius = common_leg_params.get("link_radius", 0.025)
        foot_radius = common_leg_params.get("foot_radius", 0.03)
        hip_offset_y = common_leg_params.get("hip_offset_y", 0.05) * sign_y

        # 1. Hip (Abduction/Adduction)
        hip_joint_conf = leg_conf["joints"]["hip"]
        hip_body_pos = format_list(hip_attach_pos)
        hip_body_el = ET.SubElement(torso_el, "body", attrib={"name": f"{leg_name}_hip_link", "pos": hip_body_pos})
        hip_joint_attribs = {
            "name": hip_joint_conf["name"],
            "axis": format_list(hip_joint_conf["axis"]),
            "range": format_list(hip_joint_conf["range"]),
            # Add pos if needed, defaults taken from <default>
        }
        ET.SubElement(hip_body_el, "joint", attrib=hip_joint_attribs)
        ET.SubElement(hip_body_el, "geom", attrib={ # Small Ab/Ad link geom
            "fromto": f"0 0 0 0 {hip_offset_y} 0",
            "size": format_list(link_radius)
        })
        all_joints.append(hip_joint_conf)
        all_motors.append({ # Store motor info linked to joint
             "name": f"{leg_name}_hip_motor",
             "joint": hip_joint_conf["name"],
             "gear": format_list(hip_joint_conf["motor_gear"])
        })

        # 2. Thigh (Hip Flexion/Extension)
        thigh_joint_conf = leg_conf["joints"]["thigh"]
        thigh_body_pos = f"0 {hip_offset_y} 0" # Relative to hip link
        thigh_body_el = ET.SubElement(hip_body_el, "body", attrib={"name": f"{leg_name}_thigh_link", "pos": thigh_body_pos})
        thigh_joint_attribs = {
            "name": thigh_joint_conf["name"],
            "axis": format_list(thigh_joint_conf["axis"]),
            "range": format_list(thigh_joint_conf["range"]),
        }
        ET.SubElement(thigh_body_el, "joint", attrib=thigh_joint_attribs)
        ET.SubElement(thigh_body_el, "geom", attrib={ # Thigh geom
            "fromto": f"0 0 0 0 0 {-thigh_len}",
            "size": format_list(link_radius)
        })
        all_joints.append(thigh_joint_conf)
        all_motors.append({
             "name": f"{leg_name}_thigh_motor",
             "joint": thigh_joint_conf["name"],
             "gear": format_list(thigh_joint_conf["motor_gear"])
        })

        # 3. Calf (Knee Flexion/Extension)
        calf_joint_conf = leg_conf["joints"]["calf"]
        calf_body_pos = f"0 0 {-thigh_len}" # Relative to thigh link
        calf_body_el = ET.SubElement(thigh_body_el, "body", attrib={"name": f"{leg_name}_calf_link", "pos": calf_body_pos})
        calf_joint_attribs = {
            "name": calf_joint_conf["name"],
            "axis": format_list(calf_joint_conf["axis"]),
            "range": format_list(calf_joint_conf["range"]),
        }
        ET.SubElement(calf_body_el, "joint", attrib=calf_joint_attribs)
        ET.SubElement(calf_body_el, "geom", attrib={ # Calf geom
            "fromto": f"0 0 0 0 0 {-calf_len}",
            "size": format_list(link_radius)
        })
        all_joints.append(calf_joint_conf)
        all_motors.append({
             "name": f"{leg_name}_calf_motor",
             "joint": calf_joint_conf["name"],
             "gear": format_list(calf_joint_conf["motor_gear"])
        })

        # 4. Foot
        foot_body_pos = f"0 0 {-calf_len}" # Relative to calf link
        foot_body_el = ET.SubElement(calf_body_el, "body", attrib={"name": f"{leg_name}_foot", "pos": foot_body_pos})
        foot_geom_attribs = { # Foot contact geom
             "type": "sphere",
             "size": format_list(foot_radius),
             "material": config["visuals"]["materials"].get("foot_mat", {}).get("name", "foot_mat"), # Use name from yaml if exists
             "group": "4", # Using group 4 for feet as per example
             "contype": "1",
             "conaffinity": "1"
        }
        ET.SubElement(foot_body_el, "geom", attrib=foot_geom_attribs)
        # Optional: Add foot inertial definition here if needed


    # Actuators
    actuator_el = ET.SubElement(root, "actuator")
    logging.debug(f"Generating {len(all_motors)} actuators...")
    for motor_conf in all_motors:
        ET.SubElement(actuator_el, "motor", attrib=motor_conf)

    # Sensors
    sensor_el = ET.SubElement(root, "sensor")
    sensor_definitions = config.get("sensors", [])
    logging.debug(f"Processing {len(sensor_definitions)} sensor definitions from YAML...")

    for sensor_conf in sensor_definitions:
        sensor_type = sensor_conf.pop("type") # Get and remove type key
        tag_name = sensor_type # Most sensor types map directly to tags

        if tag_name in ["jointpos", "jointvel"]:
             # Generate one sensor for each joint
             for joint_info in all_joints:
                 joint_name = joint_info["name"]
                 sensor_name = f"{joint_name}_{'pos' if tag_name == 'jointpos' else 'vel'}"
                 ET.SubElement(sensor_el, tag_name, attrib={"name": sensor_name, "joint": joint_name})
        elif tag_name == "actuatorfrc":
             # Generate one sensor for each motor
             for motor_info in all_motors:
                  motor_name = motor_info["name"]
                  sensor_name = f"{motor_name}_frc" # Or just motor_name? Use convention
                  ET.SubElement(sensor_el, tag_name, attrib={"name": sensor_name, "actuator": motor_name})
        elif tag_name == "touch":
             # Assumes site name matches sensor name in YAML for now
             # Requires <site> elements to be defined in the body section (not currently done by this script)
             site_name = sensor_conf.get("site")
             if site_name:
                  sensor_name = f"{site_name}_touch"
                  ET.SubElement(sensor_el, tag_name, attrib={"name": sensor_name, "site": site_name})
             else:
                  logging.warning(f"Skipping touch sensor definition missing 'site': {sensor_conf}")

        else:
             # Handle simple sensors (IMU-like, frame*)
             sensor_attribs = {k: format_list(v) for k, v in sensor_conf.items()}
             # Generate a default name if none provided
             if "name" not in sensor_attribs:
                 objname = sensor_attribs.get("objname", sensor_attribs.get("site", "unnamed"))
                 sensor_attribs["name"] = f"{objname}_{tag_name}" # e.g., torso_accelerometer
             ET.SubElement(sensor_el, tag_name, attrib=sensor_attribs)


    # Write to file
    logging.info(f"Writing MJCF XML to: {output_xml_path}")
    try:
        # Use pretty printing for readability
        pretty_xml_string = prettify_xml(root)
        with open(output_xml_path, "w", encoding="utf-8") as f:
            f.write(pretty_xml_string)
        logging.info("Successfully generated MJCF file.")
    except Exception as e:
        logging.error(f"Error writing XML file: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate MuJoCo MJCF XML from YAML configuration.")
    parser.add_argument(
        "-y", "--yaml",
        default=os.path.join("..", "config", "quadruped_go2_like.yaml"), # Default relative path
        help="Path to the input YAML configuration file."
    )
    parser.add_argument(
        "-o", "--output",
        default=os.path.join("..", "mujoco", "quadruped_go2_like_generated.xml"), # Default relative path for output
        help="Path to save the generated MJCF XML file."
    )
    args = parser.parse_args()

    # Ensure paths are absolute or relative to the script location if needed
    script_dir = os.path.dirname(__file__)
    yaml_file = os.path.abspath(os.path.join(script_dir, args.yaml))
    output_file = os.path.abspath(os.path.join(script_dir, args.output))

    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(output_file)
    if not os.path.exists(output_dir):
        logging.info(f"Creating output directory: {output_dir}")
        os.makedirs(output_dir)

    create_mjcf_from_yaml(yaml_file, output_file)
