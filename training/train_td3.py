import gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from gym_env.custom_env import CarEnv

def train():
    env = DummyVecEnv([lambda: CarEnv()])
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=1000)
    model.save("ppo_car_model")
    
    
if __name__ == "__main__":
    train()
