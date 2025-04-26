# MyBot YAML→MuJoCo Project

## Overview
End-to-end pipeline:  
1. Define your robot in YAML  
2. Convert YAML → MJCF (MuJoCo XML) or write the XML from scratch
3. Simulate the model in simulator(optional)
4. Train with RL (TD3/SAC/PPO)  
5. Deploy policy on real quadruped

## Quickstart
```bash
pip install -r requirements.txt
python scripts/yaml2mjcf.py -y ../config/example_robot.yaml -o ../mujoco/mybot.xml
python training/train_td3.py
python policy/export_policy.py
python deployment/go2_bridge.py
```

## Structure
See repo_map.txt or browse directories.
