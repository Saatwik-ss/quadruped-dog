# Architecture

```text
[config/*.yaml] 
      │
      ▼
scripts/yaml2mjcf.py → mujoco/*.xml
      │
      ▼
gym_env/custom_env.py → Gym interface
      │
      ▼
training/train_td3.py → policy/td3_mybot.zip
      │
      ▼
policy/export_policy.py → policy/policy.pth
      │
      ▼
deployment/go2_bridge.py → real Go2 robot
```
