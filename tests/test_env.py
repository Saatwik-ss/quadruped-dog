import pytest
from gym_env.custom_env import CustomMujocoEnv

def test_env_init():
    env = CustomMujocoEnv('mujoco/mybot.xml', render_mode='rgb_array')
    obs, _ = env.reset()
    assert env.observation_space.contains(obs)
