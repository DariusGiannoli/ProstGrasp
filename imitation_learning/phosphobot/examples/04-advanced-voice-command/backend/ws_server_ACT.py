import asyncio
import json
import os
import cv2
import time
import numpy as np
import websockets
import uvicorn
import subprocess
import requests
from pathlib import Path

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from threading import Thread

from backend.modules.mic import record_audio
from backend.modules.llm import get_llm_response
from backend.modules.whisper_transcriber import transcribe_audio
from backend.modules.tts import speak_streaming
from phosphobot.camera import AllCameras
from phosphobot.api.client import PhosphoApi
from phosphobot.am import ACT

# ===== CONFIG =====
SHARED_STATE_PATH = "shared_state.json"
connected_clients = set()
model = None
allcameras = AllCameras()
client = PhosphoApi(base_url="http://localhost:80")

MODEL_ID = "phospho-app/PAphospho-AI-voice-lego-red-2-6lp91kv18x"
ACT_SERVER_PORT = 8080  # Port sur lequel ton server.py tourne


# ===== UTILS =====
def write_shared_state(prompt: str = "", running: bool = False):
    with open(SHARED_STATE_PATH, "w") as f:
        json.dump({"prompt": prompt, "running": running}, f)


def load_state():
    try:
        with open(SHARED_STATE_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"⚠️ Could not load shared state: {e}")
        return {"running": False}


def wait_for_act_server(host="localhost", port=ACT_SERVER_PORT, timeout=60):
    url = f"http://{host}:{port}/health"
    print(f"⏳ Waiting for ACT server to be ready on {url}...")
    start_time = time.time()
    while True:
        try:
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                print("✅ ACT server is ready!")
                break
        except Exception:
            pass

        if time.time() - start_time > timeout:
            raise TimeoutError("❌ ACT server did not start in time.")

        time.sleep(0.5)


# ====== FASTAPI ======
app = FastAPI()
app.add_middleware(
    CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"]
)


@app.post("/shutdown")
def shutdown_robot():
    write_shared_state(prompt="", running=False)
    print("🛑 Shutdown received!")
    return {"status": "ok"}


# ====== INFERENCE THREAD ======
def run_model_loop(prompt: str):
    global model
    if model is None:
        print("❌ ACT not ready yet. Skipping inference.")
        return

    print(f"🤖 Starting inference loop for prompt: {prompt}")
    while load_state().get("running"):
        images = [
            allcameras.get_rgb_frame(0, resize=(240, 320)),
            allcameras.get_rgb_frame(1, resize=(240, 320)),
        ]

        for i, image in enumerate(images):
            images[i] = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        robot_state = client.control.read_joints()

        inputs = {
            "state": np.array(robot_state.angles_rad),
            "images": np.array(images),
        }

        for key, value in inputs.items():
            if isinstance(value, np.ndarray):
                print(f"{key}: {value.shape}")

        actions = model(inputs)

        for action in actions:
            print(f"Writing joints for action {action}")
            client.control.write_joints(angles=action.tolist())
            time.sleep(1 / 30)
    print("🛑 Inference stopped.")


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
                result = (
                    json.loads(result_raw)
                    if isinstance(result_raw, str)
                    else result_raw
                )
            except json.JSONDecodeError:
                result = {"reply": "Sorry, I didn’t get that.", "command": None}

            reply = result.get("reply", "Sorry, I didn’t get that.")
            print(f"🤖 Phosphobot: {reply}")

            await websocket.send(json.dumps({"benderTranscriptReset": True}))

            def on_word(word):
                asyncio.run_coroutine_threadsafe(
                    websocket.send(json.dumps({"benderTranscriptAppend": word})), loop
                )

            t = Thread(target=speak_streaming, args=(reply, on_word))
            t.start()
            t.join()

            await websocket.send(json.dumps({"doneSpeaking": True}))

            command = result.get("command")

            if command:
                print(f"🚀 Triggering robot model with prompt: {transcript}")
                write_shared_state(prompt=transcript, running=True)
                Thread(target=run_model_loop, args=(transcript,)).start()

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
    if not os.path.exists(SHARED_STATE_PATH):
        write_shared_state(prompt="", running=False)

    # 🚀 1. Start ACT server with uv run
    print(f"⚡ Launching ACT server with model_id={MODEL_ID}...")
    act_server_process = subprocess.Popen(
        [
            "uv",
            "run",
            "../../phosphobot/inference/ACT/server.py",
            "--model_id",
            MODEL_ID,
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # 🛜 2. Wait until ACT server is ready
    wait_for_act_server()

    # 🚀 3. Connect ACT client
    model = ACT()

    # 🛜 4. Now start FastAPI and WebSocket servers
    try:
        asyncio.run(main())
    finally:
        print("🛑 Shutting down ACT server...")
        act_server_process.terminate()
        act_server_process.wait()
