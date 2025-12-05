import os
import tempfile
import requests
import time
from threading import Thread
from dotenv import load_dotenv

load_dotenv()
API_KEY = os.getenv("ELEVENLABS_API_KEY")
VOICE_ID = os.getenv("ELEVENLABS_VOICE_ID")

def speak_streaming(text: str, on_word):
    print(f"🔊 [ElevenLabs] Streaming: {text}")

    # Découpe les mots pour les sous-titres
    words = text.strip().split()

    # ⚡ Lance les sous-titres en parallèle
    def stream_words():
        for word in words:
            time.sleep(0.25)  # rythme approximatif pour synchronisation
            on_word(word + " ")

    t = Thread(target=stream_words)
    t.start()

    # Appel API ElevenLabs (bloquant le temps de parole)
    url = f"https://api.elevenlabs.io/v1/text-to-speech/{VOICE_ID}/stream"
    headers = {
        "xi-api-key": API_KEY,
        "Content-Type": "application/json"
    }
    data = {
        "text": text,
        "model_id": "eleven_monolingual_v1",
        "voice_settings": {
            "stability": 0.7,
            "similarity_boost": 0.8
        }
    }

    with requests.post(url, headers=headers, json=data, stream=True) as r:
        if r.status_code != 200:
            print(f"❌ TTS error: {r.status_code} - {r.text}")
            return

        tmp_path = tempfile.mktemp(suffix=".mp3")
        with open(tmp_path, "wb") as f:
            for chunk in r.iter_content(chunk_size=1024):
                if chunk:
                    f.write(chunk)

        os.system(f"afplay '{tmp_path}'")

    t.join()
