import speech_recognition as sr
import tempfile

def record_audio():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        print("🎤 Listening... (speak when you're ready)")
        recognizer.energy_threshold = 250
        recognizer.pause_threshold = 1.2
        recognizer.dynamic_energy_threshold = False
        recognizer.adjust_for_ambient_noise(source, duration=0.3)

        try:
            audio = recognizer.listen(source, timeout=None, phrase_time_limit=15)
            print("🛑 Done recording.")
        except sr.WaitTimeoutError:
            print("⏱️ No speech detected.")
            return None

    # Sauvegarder le fichier audio temporairement
    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as f:
        f.write(audio.get_wav_data())
        return f.name
