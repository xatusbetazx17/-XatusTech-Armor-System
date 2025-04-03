## 🗣 Voice Command Interface - `voice_command.py`
```python
import speech_recognition as sr

def listen_for_command():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        print("[Voice] Listening for command...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        command = recognizer.recognize_google(audio).lower()
        print(f"[Voice] You said: {command}")
        return command
    except sr.UnknownValueError:
        print("[Voice] Could not understand the audio.")
        return ""
    except sr.RequestError:
        print("[Voice] Could not request results from Google Speech Recognition service.")
        return ""

def activate_buster():
    print("[Voice] Command received: Activating Buster Mode 💥")
    # Trigger related hardware or software logic here

if __name__ == '__main__':
    command = listen_for_command()
    if "buster" in command:
        activate_buster()
    else:
        print("[Voice] No recognized activation keyword.")
```
