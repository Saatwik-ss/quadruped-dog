#!/usr/bin/env python3
import sys
from pathlib import Path
# Add the project root directory to Python path
sys.path.append(str(Path(__file__).parent.parent))

import mujoco
import mujoco.viewer as viewer
from gym_env.custom_env import calc_done

def main():
    xml_path = Path(__file__).parent.parent / "mujoco" / "go2_like_dog.xml"
    # load the model + data
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data  = mujoco.MjData(model)
    # launch the full GUI
    viewer.launch(model, data)

if __name__ == "__main__":
    main()
    print("Simulation has started.")