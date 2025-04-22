#!/usr/bin/env python3
import torch
from stable_baselines3 import TD3

def main():
    model = TD3.load('policy/td3_mybot')
    policy = model.policy
    torch.save(policy.state_dict(), 'policy/policy.pth')
    print('Saved policy weights to policy/policy.pth')

if __name__ == '__main__':
    main()
