# PhosphoBot: Keyboard Control Example

This example demonstrates how to control a robot using keyboard inputs.

## Prerequisites

- Python 3.6+
- A robot running the PhosphoBot server
- Required Python packages (install via `pip install -r requirements.txt`)

## Configuration

The script `crane.py` contains configurable parameters:

```python
BASE_URL: str = "http://127.0.0.1:80/"
STEP_SIZE: int = 2  # Movement step in centimeters
SLEEP_TIME: float = 0.05  # Loop sleep time (20 Hz)
```

## How to Run

1. Make sure your robot is powered on and the PhosphoBot server is running
2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```
3. Run the script:
   ```
   python crane.py
   ```
4. Choose your position relative to the robot ("Behind" or "Facing")

## Controls

Control the robot with these keys:

- Arrow Up/Down: Move forward/backward
- Arrow Right/Left: Move right/left
- A/D: Move up/down
- Space: Toggle gripper open/close

## Customization

You can modify:

- The step size for movement
- The control loop speed
- The key mappings for different directions
