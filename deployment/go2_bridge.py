import os
os.environ.setdefault('MUJOCO_GL', 'glfw')

import mujoco
import mujoco.viewer as viewer
from pathlib import Path

xml = Path("mujoco/go2_like_dog.xml") 
model = mujoco.MjModel.from_xml_path(str(xml))
data  = mujoco.MjData(model)
viewer.launch(model, data)
