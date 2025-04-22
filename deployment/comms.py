#!/usr/bin/env python3
from unitree_legged_sdk import LowState, LowCmd

def decode_state(buf):
    return LowState.decode(buf)

def encode_cmd(cmd: LowCmd):
    return cmd.encode()
