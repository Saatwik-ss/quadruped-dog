import mujoco
import mujoco.viewer as viewer
from pathlib import Path

def main():
    xml_path = Path(__file__).parent.parent / "mujoco_xml" / "car.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data  = mujoco.MjData(model)
    viewer.launch(model, data)

if __name__ == "__main__":
    main()
    
"""
python -m scripts.run_simulation
"""
