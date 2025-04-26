#!/usr/bin/env python3
import mujoco
import mujoco.viewer as viewer
from pathlib import Path
from gym_env.custom_env import calc_done

def main():
    xml_path = Path(__file__).parent.parent / "mujoco" / "car.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data  = mujoco.MjData(model)
    viewer.launch(model, data)

if __name__ == "__main__":
    main()
    
"""
python -m scripts.run_simulation
"""
