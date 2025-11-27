import threading
import pyttsx3
from unitree_control.core.control_modules import DogModule


class AudioModule(DogModule):
    """Handles text-to-speech audio"""

    def __init__(self):
        super().__init__("Audio")
        self.initialize()
    

    def initialize(self) -> None:
        if self._initialized:
            return
        
        self._engine = pyttsx3.init()
        self._initialized = True


    def play_audio(self, text: str, blocking=False):
        self._engine.say(text)
        
        if blocking:
            self._engine.runAndWait()
        else:
            threading.Thread(target=self._engine.runAndWait, daemon=True).start()


    def shutdown(self) -> None:
        self._initialized = False