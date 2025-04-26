import os
os.environ.setdefault('MUJOCO_GL', 'glfw')

import mujoco
import mujoco.viewer as viewer
from pathlib import Path

xml = Path("scripts\mujoco\car.xml") 
model = mujoco.MjModel.from_xml_path(str(xml))
data  = mujoco.MjData(model)
viewer.launch(model, data)
"""python scripts/yaml2mjcf.py -y ../config/example_robot.yaml -o ../mujoco/mybot.xml"""