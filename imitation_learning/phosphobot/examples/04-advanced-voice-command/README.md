# Phosphobot AI Voice Demo

This demo showcases the voice-enabled AI assistant capabilities of Phosphobot, allowing users to interact with an AI through voice commands and receive spoken responses. It can also control a robot arm when connected to the Phosphobot hardware.

## Overview

The demo consists of:

- A backend Python service that handles:
  - Voice recording
  - Speech-to-text transcription (Whisper)
  - LLM processing (for understanding and generating responses)
  - Text-to-speech synthesis (ElevenLabs)
  - Robot arm control integration
- A frontend web interface built with Next.js

## Prerequisites

- Python 3.9+
- Node.js 18+
- npm or yarn
- A microphone and speakers
- OpenAI API key
- ElevenLabs API key
- Faster-Whisper installed (for speech recognition)
- For robot arm control: Phosphobot hardware setup

## Installation

### Backend Setup

1. Navigate to the backend directory:

   ```bash
   cd backend
   ```

2. Install the required Python packages:

   ```bash
   pip install -r requirements.txt
   ```

3. Set up your environment variables (create a `.env` file in the backend directory):

   ```
   # Required API keys
   OPENAI_API_KEY=your_openai_api_key
   ELEVENLABS_API_KEY=your_elevenlabs_api_key

   # ElevenLabs voice configuration
   ELEVENLABS_VOICE_ID=your_preferred_voice_id

   # Optional: For robot control
   PHOSPHO_API_URL=http://localhost:80
   ```

4. Install Faster-Whisper for speech recognition:
   ```bash
   pip install faster-whisper
   ```

### Frontend Setup

1. Navigate to the frontend directory:

   ```bash
   cd frontend-demo-AI
   ```

2. Install the required npm packages:
   ```bash
   npm install
   ```

## Usage

### Running the Demo

You can run the demo in several modes:

#### Gr00t Demo (Recommended)

```bash
cd frontend-demo-AI
npm run demo-Gr00t
```

In another terminal:

```bash
cd frontend-demo-AI
npm run dev
```

#### ACT Demo

```bash
cd frontend-demo-AI
npm run demo-ACT
```

In another terminal:

```bash
cd frontend-demo-AI
npm run dev
```

#### Replay Demo

```bash
cd frontend-demo-AI
npm run demo-replay
```

In another terminal:

```bash
cd frontend-demo-AI
npm run dev
```

Then open your browser and navigate to `http://localhost:3000`

### Testing Individual Components

You can also test the individual components:

#### Test LLM Response

```bash
python llm_test.py
```

#### Test Voice Interaction

```bash
python llm_voice_test.py
```

#### Test Command Execution

```bash
python executor_test.py
```

## Robot Arm Integration

For full robot arm functionality:

1. Ensure the Phosphobot hardware is correctly set up and connected
2. The Phosphobot API server should be running on the default port (http://localhost:80)
3. The robot's cameras should be properly calibrated and working
4. The Gr00t model will be automatically initialized when running the demo

## Project Structure

- `backend/`: Contains the Python backend services
  - `modules/`: Core functionality modules (mic, speech, LLM, etc.)
  - `prompts/`: LLM prompts for different use cases
  - `ws_server_*.py`: WebSocket server implementations for different modes
- `frontend-demo-AI/`: Next.js frontend application

## License

This project is part of Phosphobot.

## Troubleshooting

- Make sure your microphone is properly connected and permissions are granted
- Check that your OpenAI API key is valid
- Verify your ElevenLabs API key is correct and has sufficient credits
- If text-to-speech isn't working, check the ElevenLabs voice ID in the configuration
- For robot control issues, verify the Phosphobot API server is running and accessible
- If speech recognition is slow, ensure Faster-Whisper is properly installed
- Ensure all dependencies are correctly installed
