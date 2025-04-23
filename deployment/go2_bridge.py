import os
# ensure we use GLFW
os.environ.setdefault('MUJOCO_GL', 'glfw')

import mujoco
import mujoco.viewer as viewer
from pathlib import Path

xml = Path("mujoco/mybot.xml")   # adjust path if needed
model = mujoco.MjModel.from_xml_path(str(xml))
data  = mujoco.MjData(model)

# blocking full-GUI: should pop up a window and let you rotate/zoom
viewer.launch(model, data)
