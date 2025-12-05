from faster_whisper import WhisperModel

# Utilise le modèle anglais ultra léger pour la vitesse
model = WhisperModel("small.en", compute_type="int8")

def transcribe_audio(audio_path: str) -> str:
    print(f"📝 Transcribing (fast) from: {audio_path}")
    segments, _ = model.transcribe(audio_path, beam_size=1)

    return " ".join([segment.text for segment in segments]).strip()
