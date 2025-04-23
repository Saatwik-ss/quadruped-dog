#!/usr/bin/env python3
import mujoco
import mujoco.viewer as viewer
from pathlib import Path

def main():
    xml_path = Path(__file__).parent.parent / "mujoco" / "quadruped_go2_like_generated.xml"
    # load the model + data
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data  = mujoco.MjData(model)
    # launch the full GUI
    viewer.launch(model, data)

if __name__ == "__main__":
    main()
