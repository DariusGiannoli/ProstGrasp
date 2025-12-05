# PhosphoBot: Wave Back Example

This example demonstrates a robot that waves back when it detects a hand in the camera view.

## Prerequisites

- Python 3.6+
- A robot running the PhosphoBot server
- Webcam or camera connected to your computer
- Required Python packages (install via `pip install -r requirements.txt`)

## How It Works

The application uses MediaPipe for hand detection through your camera:

1. When a hand is detected in the camera view
2. The robot performs a waving motion
3. A cooldown period prevents the robot from waving too frequently

## Configuration

The script `wave_hand.py` contains configurable parameters:

```python
PI_IP: str = "127.0.0.1"  # IP address of the robot
PI_PORT: int = 80         # Port of the robot's API server
WAVE_COOLDOWN = 3         # Seconds between wave motions
```

## Setup

1. Make sure your robot is powered on and the PhosphoBot server is running
2. Ensure your webcam is connected and functioning
3. Install dependencies:
   ```
   pip install -r requirements.txt
   ```
4. Grant camera access permissions to the application

## Running the Application

1. Run the script:
   ```
   python wave_hand.py
   ```
2. A window will open showing your camera feed
3. Show your hand to the camera to trigger the robot to wave
4. The robot will wave and then wait for the cooldown period before it can wave again
5. Press Ctrl+C in the terminal to exit the program

## Troubleshooting

- If the robot doesn't wave, check that your hand is clearly visible in the camera frame
- Ensure good lighting for better hand detection
- If you see connection errors, verify the robot server is running at the configured IP and port

## Customization

You can modify the script to change:

- The wave motion pattern by adjusting the coordinates in the `wave_motion()` function
- The cooldown time between waves
- The hand detection sensitivity by changing MediaPipe parameters
