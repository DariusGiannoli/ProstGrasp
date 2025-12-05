# Rock Paper Scissors Game with Robot

This example demonstrates how to play Rock-Paper-Scissors with a robot. The program uses computer vision to detect hand gestures from a webcam and controls the robot to respond with its own gesture.

## Prerequisites

- Python 3.7+
- Webcam
- Compatible robot with REST API interface
- Robot running on the local network

## Installation

1. Clone this repository
2. Install the required dependencies:

```bash
pip install -r requirements.txt
```

## Configuration

Edit the `rock_paper_scissors.py` file to set your robot's IP address and port:

```python
# Robot API Configuration
PI_IP = "127.0.0.1"  # Change to your robot's IP
PI_PORT = 80         # Change to your robot's port
```

## How to Play

1. Connect to your robot and ensure it's powered on
2. Run the game:

```bash
python rock_paper_scissors.py
```

3. When the robot performs the countdown (moving up and down three times), show your hand gesture (rock, paper, or scissors) to the webcam
4. The program will:
   - Detect your gesture using MediaPipe hand tracking
   - Choose a random gesture for the robot
   - Display both choices and the winner
   - Command the robot to perform its chosen gesture

## Supported Gestures

- **Rock**: Closed fist
- **Paper**: Open hand with all fingers extended
- **Scissors**: Only index and middle fingers extended

## Troubleshooting

- If the webcam isn't detected, check your camera connections and permissions
- If hand gestures aren't recognized, ensure proper lighting and hold your hand clearly in the camera's view
- If the robot doesn't respond, verify the connection parameters and check that the robot's API server is running

## How It Works

The application uses:

- OpenCV for webcam capture
- MediaPipe for hand landmark detection
- REST API calls to control the robot's movements
- Simple game logic to determine the winner
