import mujoco
import mujoco.viewer as viewer
from pathlib import Path
import os
import sys
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)


def main():
    xml_path = Path(__file__).parent.parent / "mujoco_xml" / "car.xml"
    # load the model + data
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data  = mujoco.MjData(model)
    # launch the full GUI
    viewer.launch(model, data)

if __name__ == "__main__":
    main()
    print("Simulation has started.")
    
# python -m scripts.run_simulation