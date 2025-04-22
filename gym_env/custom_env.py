import gymnasium as gym
import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer

class CustomMujocoEnv(gym.Env):
    metadata = {'render_modes': ['human', 'rgb_array']}

    def __init__(self, xml_path, render_mode='human'):
        super().__init__()
        self.model = load_model_from_path(xml_path)
        self.sim = MjSim(self.model)
        self.viewer = MjViewer(self.sim) if render_mode=='human' else None

        # define your spaces
        self.action_space = gym.spaces.Box(-1, 1, (1,), dtype=float)
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(2,), dtype=float
        )

    def reset(self, *, seed=None, options=None):
        self.sim.reset()
        self.sim.forward()
        return self._get_obs(), {}

    def step(self, action):
        self.sim.data.ctrl[:] = action
        self.sim.step()
        obs = self._get_obs()
        reward = -abs(obs[0])
        done = False
        return obs, reward, done, False, {}

    def _get_obs(self):
        return np.concatenate([self.sim.data.qpos, self.sim.data.qvel])

    def render(self):
        if self.viewer:
            self.viewer.render()
