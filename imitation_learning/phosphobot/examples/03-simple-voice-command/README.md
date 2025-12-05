# PhosphoBot: Voice Command Example

Control your robot with simple voice commands.

## Prerequisites

- Python 3.6+
- A robot running the PhosphoBot server
- Required Python packages (install via `pip install -r requirements.txt`)
- Microphone connected to your computer

## How to Run

1. Ensure your robot is powered on and the PhosphoBot server is running
2. Install dependencies:
   ```
   pip install -r requirements.txt
   ```
3. Make sure your system has granted microphone access permissions to the application
4. Run the script:
   ```
   python voice_command.py
   ```
5. Press and hold SPACEBAR to record your voice command
6. Release SPACEBAR to process the command
7. Press ESC to exit the program

## Available Commands

The robot responds to these voice commands:

- "left" or "that": Moves a box to the left
- "right", "write", or "riots": Moves a box to the right
- "wave", "hello", "say", "what", "wait", or "ways": Makes the robot wave

## Implementation Details

- Uses CMUSphinx for offline speech recognition
- Records audio while the SPACEBAR is held down
- Processes voice commands through simple keyword matching
- Executes pre-recorded movement patterns stored as JSON files
