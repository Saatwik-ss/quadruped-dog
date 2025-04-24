import os
# ensure we use GLFW
os.environ.setdefault('MUJOCO_GL', 'glfw')

import mujoco
import mujoco.viewer as viewer
from pathlib import Path

xml = Path("mujoco/quadruped_go2_like_generated.xml")   # adjust path if needed
model = mujoco.MjModel.from_xml_path(str(xml))
data  = mujoco.MjData(model)
viewer.launch(model, data)
