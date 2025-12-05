"""
Tests for the teleop module.

```
pytest tests/test_so100.py
```
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from phosphobot.hardware import SO100Hardware


# Create robot pytest fixture
@pytest.fixture
def robot() -> SO100Hardware:
    """
    Create a robot instance
    """
    robot = SO100Hardware()

    return robot


# TODO: implement SO100 specific tests here
