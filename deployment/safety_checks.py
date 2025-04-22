#!/usr/bin/env python3

def clamp_torques(actions, min_torque=-1.0, max_torque=1.0):
    return [max(min(a, max_torque), min_torque) for a in actions]
