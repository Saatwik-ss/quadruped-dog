#!/usr/bin/env python3
import numpy as np
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from gym_env.custom_env import CustomMujocoEnv

def main():
    env = CustomMujocoEnv('mujoco/mybot.xml')
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(mean=np.zeros(n_actions),
                                     sigma=0.1 * np.ones(n_actions))

    model = TD3(
        'MlpPolicy',
        env,
        action_noise=action_noise,
        verbose=1,
        tensorboard_log='./tensorboard/'
    )
    model.learn(total_timesteps=2_000_000)
    model.save('policy/td3_mybot')

if __name__ == '__main__':
    main()
