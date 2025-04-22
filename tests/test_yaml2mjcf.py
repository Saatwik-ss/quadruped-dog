import pytest
from scripts.yaml2mjcf import yaml_to_mjcf
import os

def test_conversion(tmp_path):
    y = tmp_path / 'robot.yaml'
    x = tmp_path / 'robot.xml'
    y.write_text("""robot:\n  name: testbot\ngravity: [0,0,-9.81]\nlinks: []\n""")
    yaml_to_mjcf(str(y), str(x))
    assert x.exists()
