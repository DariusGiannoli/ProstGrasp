import sounddevice as sd  # type: ignore
import scipy.io.wavfile as wavfile  # type: ignore
import numpy as np
import keyboard  # type: ignore
import os
import requests  # type: ignore
import speech_recognition as sr  # type: ignore


class AudioRecorder:
    def __init__(self, sample_rate=44100, channels=1):
        self.sample_rate = sample_rate
        self.channels = channels
        self.recording = False
        self.recorded_audio = []
        self.recognizer = sr.Recognizer()

    def start_recording(self):
        self.recording = True
        self.recorded_audio = []
        print("Recording started...")
        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=self.channels,
            callback=self.audio_callback,
        )
        self.stream.start()

    def stop_recording(self):
        self.recording = False
        self.stream.stop()
        print("Recording stopped.")
        return self.convert_to_text()

    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status)
        if self.recording:
            self.recorded_audio.append(indata.copy())

    def save_audio(self):
        if not self.recorded_audio:
            return None

        # Combine recorded audio chunks
        audio_data = np.concatenate(self.recorded_audio, axis=0)

        # Normalize audio to prevent clipping
        audio_data = audio_data / np.max(np.abs(audio_data))

        # Scale to 16-bit range
        scaled_audio = (audio_data * 32767).astype(np.int16)

        # Save to a WAV file
        filename = "recorded_audio.wav"
        wavfile.write(filename, self.sample_rate, scaled_audio)

        return filename

    def convert_to_text(self):
        filename = self.save_audio()
        if not filename:
            return None

        try:
            # Use CMUSphinx for offline recognition
            with sr.AudioFile(filename) as source:
                audio = self.recognizer.record(source)
                text = self.recognizer.recognize_sphinx(audio)

                if text:
                    print("Recognized Text:", text)
                    # Call action function
                    decide_action(text)
                    return text

                print("No text recognized")
                return None

        except sr.UnknownValueError:
            print("Could not understand audio")
            return None
        except sr.RequestError as e:
            print(f"Recognition error: {e}")
            return None
        finally:
            # Remove temporary audio file
            if filename and os.path.exists(filename):
                os.remove(filename)


def api_call(endpoint: str, data: dict | None = None):
    try:
        response = requests.post(
            f"http://localhost:80/{endpoint}",
            json=data,
        )
        return response
    except requests.RequestException as e:
        print(f"Failed to send data: {e}")
        return None


def move_box_left():
    api_call(
        "recording/play",
        {"episode_path": "push_left.json"},
    )


def move_box_right():
    api_call(
        "recording/play",
        {"episode_path": "push_right.json"},
    )


def say_hello():
    api_call(
        "recording/play",
        {"episode_path": "wave.json"},
    )


def decide_action(prompt: str):
    if "left" in prompt or "that" in prompt:
        move_box_left()
        print("Moving box left")
    elif "right" in prompt or "write" in prompt or "riots" in prompt:
        move_box_right()
        print("Moving box right")
    elif (
        "wave" in prompt
        or "hello" in prompt
        or "say" in prompt
        or "what" in prompt
        or "wait" in prompt
        or "ways" in prompt
    ):
        say_hello()
        print("Waving")
    else:
        print("No action taken")


def main():
    api_call("move/init")
    recorder = AudioRecorder()

    print("Press and hold SPACEBAR to record. Release to stop and transcribe.")

    # Use a flag to prevent multiple event handlers
    is_space_pressed = False

    def on_press(event):
        nonlocal is_space_pressed
        if event.name == "space" and not is_space_pressed:
            is_space_pressed = True
            recorder.start_recording()

    def on_release(event):
        nonlocal is_space_pressed
        if event.name == "space" and is_space_pressed:
            is_space_pressed = False
            recorder.stop_recording()

    # Register keyboard event listeners
    keyboard.on_press_key("space", on_press)
    keyboard.on_release_key("space", on_release)

    # Keep the script running
    keyboard.wait("esc")  # Press ESC to exit the program


if __name__ == "__main__":
    main()
