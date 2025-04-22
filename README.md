# MyBot YAML→MuJoCo Project

## Overview
End-to-end pipeline:  
1. Define your robot in YAML  
2. Convert YAML → MJCF (MuJoCo XML)  
3. Wrap in a Gymnasium env  
4. Train with RL (TD3/SAC/PPO)  
5. Deploy policy on real Unitree Go2

## Quickstart
```bash
pip install -r requirements.txt
python scripts/yaml2mjcf.py config/example_robot.yaml mujoco/mybot.xml
python training/train_td3.py
python policy/export_policy.py
python deployment/go2_bridge.py
```

## Structure
See repo_map.txt or browse directories.
