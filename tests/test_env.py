import pytest
from gym_env.custom_env import CustomMujocoEnv

def test_env_init():
    env = CustomMujocoEnv('mujoco/quadruped_go2_like_generated.xml', render_mode='rgb_array')
    obs, _ = env.reset()
    assert env.observation_space.contains(obs)
    assert env.action_space.contains(env.action_space.sample())
    env.close()