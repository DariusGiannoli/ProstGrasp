# PhosphoBot: Hand Tracking Example

Control your robot using hand movements captured by your camera.

## Prerequisites

- Python 3.6+
- A robot running the PhosphoBot server
- Webcam or camera connected to your computer
- Required Python packages (install via `pip install -r requirements.txt`)

## How It Works

This example uses MediaPipe to track your hand movements through a webcam and converts them to robot movements:

- Your right hand controls the robot's movement in the Y and Z directions
- Your left hand controls the X direction
- Pinching your thumb and index finger together controls the gripper (open/close)

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
   python hand_tracking.py
   ```
2. A window will open showing your camera feed with hand tracking overlay
3. Move your hands to control the robot
4. The application displays the X, Y, Z coordinates and gripper state
5. Press 'q' to exit the application

## Troubleshooting

- If you see "Failed to connect to server", check that your robot server is running
- Ensure good lighting for better hand tracking
- Keep your hands within the camera frame for consistent tracking
- If tracking is unstable, try adjusting the `min_detection_confidence` and `min_tracking_confidence` parameters in the code

## Customization

You can modify the script to change:

- The mapping between hand position and robot movement
- The sensitivity of the gripper control
- The tracking parameters for different environments
