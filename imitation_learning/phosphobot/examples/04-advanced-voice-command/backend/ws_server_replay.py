import asyncio
import json
import os
import websockets
import uvicorn
import subprocess
import requests
import time
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from threading import Thread

from backend.modules.mic import record_audio
from backend.modules.llm import get_llm_response
from backend.modules.whisper_transcriber import transcribe_audio
from backend.modules.tts import speak_streaming

# ===== CONFIG =====
COLOR_TO_EPISODE = {
    "green": {
        "episode_id": 7,
        "episode_path": "/Users/pierre-alexandreboulay/phosphobot/recordings/lerobot_v2/AI-voice-control-demo/data/chunk-000/episode_000007.parquet"
    },
    "blue": {
        "episode_id": 5,
        "episode_path": "/Users/pierre-alexandreboulay/phosphobot/recordings/lerobot_v2/AI-voice-control-demo/data/chunk-000/episode_000005.parquet"
    },
    "red": {
        "episode_id": 6,
        "episode_path": "/Users/pierre-alexandreboulay/phosphobot/recordings/lerobot_v2/AI-voice-control-demo/data/chunk-000/episode_000006.parquet"
    }
}

PHOSPHOBOT_API_URL = "http://0.0.0.0:80/recording/play"
connected_clients = set()

# ====== FASTAPI ======
app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

@app.post("/shutdown")
def shutdown_robot():
    print("🛑 Shutdown received!")
    return {"status": "ok"}

# ====== HELPER ======
def execute_command(command: dict):
    action = command.get("action", "")
    obj = command.get("object", "").lower()

    if action == "pick_and_place":
        for color, episode_info in COLOR_TO_EPISODE.items():
            if color in obj:
                payload = {
                    "dataset_name": "AI-voice-control-demo",
                    "episode_id": episode_info["episode_id"],
                    "episode_path": episode_info["episode_path"],
                    "robot_id": 0,
                    "replicate": True,
                    "playback_speed": 1,
                    "interpolation_factor": 4
                }

                print(f"🚀 Sending replay command for {color} brick to Phosphobot API...")
                response = requests.post(PHOSPHOBOT_API_URL, json=payload)

                if response.ok:
                    print("✅ Replay launched successfully.")
                else:
                    print(f"❌ Replay failed. Status: {response.status_code} - {response.text}")
                return

        print("❓ No episode found for color in object.")
    else:
        print(f"⚠️ Unsupported action: {action}")

# ====== WEBSOCKET HANDLER ======
async def handler(websocket):
    print("🔌 New client connected")
    connected_clients.add(websocket)
    loop = asyncio.get_event_loop()

    try:
        while True:
            await websocket.send(json.dumps({"listening": True}))
            audio_path = record_audio()
            await websocket.send(json.dumps({"listening": False}))

            if not audio_path:
                continue

            transcript = transcribe_audio(audio_path)
            print(f"📝 User said: {transcript}")

            result_raw = get_llm_response(transcript)
            try:
                result = json.loads(result_raw) if isinstance(result_raw, str) else result_raw
            except json.JSONDecodeError:
                result = {"reply": "Sorry, I didn’t get that.", "command": None}

            reply = result.get("reply", "Sorry, I didn’t get that.")
            print(f"🤖 Phosphobot: {reply}")

            await websocket.send(json.dumps({"benderTranscriptReset": True}))

            def on_word(word):
                asyncio.run_coroutine_threadsafe(
                    websocket.send(json.dumps({"benderTranscriptAppend": word})),
                    loop
                )

            t = Thread(target=speak_streaming, args=(reply, on_word))
            t.start()
            t.join()

            await websocket.send(json.dumps({"doneSpeaking": True}))

            command = result.get("command")

            if command:
                print(f"🚀 Executing replay for command: {command}")
                Thread(target=execute_command, args=(command,)).start()

            await asyncio.sleep(0.5)

    except websockets.ConnectionClosed:
        print("❌ Client disconnected")
    finally:
        connected_clients.remove(websocket)

# ====== MAIN ======
async def main():
    print("🌐 WebSocket server running on ws://localhost:5051/ws")
    config = uvicorn.Config(app, host="0.0.0.0", port=5051, log_level="error")
    server = uvicorn.Server(config)

    await asyncio.gather(
        server.serve(),
        websockets.serve(handler, "0.0.0.0", 5050, ping_interval=None),
    )

if __name__ == "__main__":
    asyncio.run(main())
