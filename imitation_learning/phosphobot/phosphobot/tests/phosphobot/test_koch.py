"""
Tests for the teleop module.

```
pytest tests/test_koch.py
```
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from phosphobot.hardware import KochHardware


# Create robot pytest fixture
@pytest.fixture
def robot() -> KochHardware:
    """
    Create a robot instance
    """
    robot = KochHardware()

    return robot


# TODO: implement Koch specific tests here
