#!/usr/bin/env python3
import socket
import torch
from unitree_legged_sdk import LowCmd, LowState
from deployment.comms import decode_state, encode_cmd
from deployment.safety_checks import clamp_torques

GO2_IP = '192.168.123.10'
GO2_PORT = 8092

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', GO2_PORT))
    policy = torch.load('policy/policy.pth')
    policy.eval()

    while True:
        data, _ = sock.recvfrom(1024)
        state = decode_state(data)
        obs = state_to_obs(state)  # implement to mirror Gym obs
        with torch.no_grad():
            action = policy(torch.tensor(obs).float()).numpy()
        action = clamp_torques(action)
        cmd = LowCmd()
        cmd.motorCmd = action.tolist()
        sock.sendto(encode_cmd(cmd), (GO2_IP, GO2_PORT))

if __name__ == '__main__':
    main()
