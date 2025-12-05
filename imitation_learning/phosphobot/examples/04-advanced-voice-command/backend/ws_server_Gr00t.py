import asyncio
import json
import os
import cv2
import requests
import time
import numpy as np
import websockets
import uvicorn

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from threading import Thread

from backend.modules.mic import record_audio
from backend.modules.llm import get_llm_response
from backend.modules.whisper_transcriber import transcribe_audio
from backend.modules.tts import speak_streaming
from phosphobot.camera import AllCameras
from phosphobot.api.client import PhosphoApi
from phosphobot.am import Gr00tN1

# ===== SHARED STATE =====
SHARED_STATE_PATH = "shared_state.json"
connected_clients = set()
model = None
allcameras = AllCameras()
client = PhosphoApi(base_url="http://localhost:80")


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


# ====== ASYNC MODEL INIT (THREAD) ======
def init_modal_model():
    global model
    try:
        print("⚡ Starting Modal server (/ai-control/spawn)...")
        payload = {"model_id": "phospho-app/PAphospho-AI-voice-lego-red-2"}
        r = requests.post(
            "http://localhost:80/ai-control/spawn", json=payload, timeout=120
        )
        print(f"✅ Modal spawn response: {r.status_code} {r.text}")

        if r.status_code == 200:
            server_info = r.json()["server_info"]
            server_url = server_info["tcp_socket"][0]
            server_port = server_info["tcp_socket"][1]
            print(f"🌐 Connecting Gr00tN1 to {server_url}:{server_port}")

            model = Gr00tN1(
                action_keys=["action.single_arm", "action.gripper"],
                server_url=server_url,
                server_port=server_port,
            )
        else:
            print(f"❌ Modal server returned {r.status_code}: {r.text}")

        print("👍 Connected to Gr00tn1")

    except Exception as e:
        print(f"❌ Could not initialize model: {e}")


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
        print("❌ Gr00tN1 not ready yet. Skipping inference.")
        return

    print(f"🤖 Starting inference loop for prompt: {prompt}")
    while load_state().get("running"):
        images = [
            allcameras.get_rgb_frame(camera_id=0, resize=(320, 240)),
            allcameras.get_rgb_frame(camera_id=1, resize=(320, 240)),
        ]

        for i, image in enumerate(images):
            images[i] = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            # Add a batch dimension (from (240, 320, 3) to (1, 240, 320, 3))
            images[i] = np.expand_dims(images[i], axis=0)
            # Ensure dtype is uint8 (if it isn’t already)
            images[i] = images[i].astype(np.uint8)

        robot_state = client.control.read_joints()
        inputs = {
            "state.single_arm": np.array(robot_state.angles_rad)[:5].reshape(1, 5),
            "state.gripper": np.array(robot_state.angles_rad[-1]).reshape(1, 1),
            "video.cam_context": images[0],
            "video.cam_wrist": images[1],
            "annotation.human.action.task_description": prompt,
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

    # 🚀 Start Modal server in background thread
    Thread(target=init_modal_model).start()

    asyncio.run(main())
