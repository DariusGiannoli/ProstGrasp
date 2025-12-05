from faster_whisper import WhisperModel

# Utilise ton modèle converti localement
model = WhisperModel("/Users/pierre-alexandreboulay/whisper-small-fr-ctranslate2", compute_type="int8")

def transcribe_audio_fr(audio_path: str) -> str:
    print(f"📝 Transcription rapide depuis : {audio_path}")
    segments, _ = model.transcribe(audio_path, beam_size=1, language="fr")
    return " ".join([segment.text for segment in segments]).strip()
