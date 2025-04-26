import gym
from gym import spaces
import numpy as np
import mujoco_py

class CarEnv(gym.Env):
    def __init__(self):
        super(CarEnv, self).__init__()
        self.model = mujoco_py.load_model_from_path("mujoco\car.xml")
        self.sim = mujoco_py.MjSim(self.model)
        self.viewer = mujoco_py.MjViewer(self.sim)
        self.action_space = spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(7,), dtype=np.float32)

        self.prev_xpos = None
        self.dt = self.model.opt.timestep

    def reset(self):
        self.sim.reset()
        self.prev_xpos = self.sim.data.qpos[0]
        
        return self._get_obs()

    def step(self, action):
        action = np.clip(action, -1.0, 1.0)
        self.sim.data.ctrl[:] = action
        self.sim.step()
        xpos_after = self.sim.data.qpos[0]
        reward = (xpos_after - self.prev_xpos) / self.dt

        self.prev_xpos = xpos_after
        done = False
        info = {}
        obs = self._get_obs()
        return obs, reward, done, info

    def _get_obs(self):
        return np.concatenate([
            self.sim.data.qpos.flat,
            self.sim.data.qvel.flat
        ])

    def render(self, mode='human'):
        self.viewer.render()

    def close(self):
        self.viewer = None
