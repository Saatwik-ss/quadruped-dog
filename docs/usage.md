# Usage

1. **Install dependencies**  
   `pip install -r requirements.txt`

2. **Generate XML**  
   `python scripts/yaml2mjcf.py config/example_robot.yaml mujoco/mybot.xml`

3. **Train**  
   `python training/train_td3.py`

4. **Export Policy**  
   `python policy/export_policy.py`

5. **Deploy**  
   `python deployment/go2_bridge.py`
