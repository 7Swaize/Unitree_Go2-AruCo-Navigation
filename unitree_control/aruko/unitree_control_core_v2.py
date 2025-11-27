from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable, List, Optional, Protocol, Dict, Type, TypeVar, Generic
from enum import Enum, auto
import threading
import time
import sys

import cv2
import numpy as np
import pytesseract
import pyttsx3

from unitree_control.controller_input.callback_manager import _InputSignalCallbackManager, _UnitreeRemoteControllerInputParser
from unitree_control.controller_input.controller_state import ControllerState
from unitree_control.controller_input.input_signal import InputSignal


# TTS: https://medium.com/@vndee.huynh/build-your-own-voice-assistant-and-run-it-locally-whisper-ollama-bark-c80e6f815cba
# Video Streaming: https://medium.com/@prasadparit006/python-project-building-a-simple-video-streaming-server-using-opencv-c8a9d9ba86d2

# Digging into Dog: https://www.darknavy.org/darknavy_insight/the_jailbroken_unitree_robot_dog

# Github Repo Searcher: https://github.com/search?type=Code


class HardwareInterface(ABC):
    """Abstract interface for hardware control (SDK or simulation)"""
    
    @abstractmethod
    def initialize(self) -> None:
        """Initialize hardware connection"""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """Clean shutdown of hardware"""
        pass
    
    @abstractmethod
    def move(self, vx: float, vy: float) -> None:
        """Move the dog"""
        pass

    @abstractmethod
    def rotate(self, vrot: float):
        """Rotate the dog"""
        pass

    
    @abstractmethod
    def stand_up(self) -> None:
        """Stand up"""
        pass
    
    @abstractmethod
    def stand_down(self) -> None:
        """Lay down"""
        pass
    
    @abstractmethod
    def stop_move(self) -> None:
        """Stop all movement"""
        pass


class UnitreeSDKHardware(HardwareInterface):
    """Real Unitree SDK implementation"""
    
    def __init__(self):
        self._sport_client = None
        self._initialized = False
    
    def initialize(self) -> None:
        if self._initialized:
            return
            
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        
        print("[SDK] Initializing ChannelFactory")
        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)
        
        print("[SDK] Creating SportClient")
        self._sport_client = SportClient()
        self._sport_client.Init()
        self._sport_client.SetTimeout(3.0)
        
        print("[SDK] Standing up")
        self._sport_client.StandUp()
        time.sleep(1)
        self._sport_client.StopMove()
        time.sleep(0.5)
        
        self._initialized = True
    
    def shutdown(self) -> None:
        if self._sport_client:
            self._sport_client.StopMove()
    
    def move(self, vx: float, vy: float) -> None:
        if self._sport_client:
            self._sport_client.Move(vx, vy, 0)

    def rotate(self, vrot: float):
        if self._sport_client:
            self._sport_client.Move(0, 0, vrot)
    
    def stand_up(self) -> None:
        if self._sport_client:
            self._sport_client.StandUp()
    
    def stand_down(self) -> None:
        if self._sport_client:
            self._sport_client.StandDown()
    
    def stop_move(self) -> None:
        if self._sport_client:
            self._sport_client.StopMove()


class SimulatedHardware(HardwareInterface):
    """Simulated hardware for testing without SDK"""
    
    def __init__(self):
        self._initialized = False
        self._position = [0.0, 0.0]
        self._rotation = 0
        self._is_standing = False
    
    def initialize(self) -> None:
        print("[SIM] Initializing simulated hardware")
        self._initialized = True
        self._is_standing = True
    
    def shutdown(self) -> None:
        print("[SIM] Shutting down simulation")
        self._initialized = False
    
    def move(self, vx: float, vy: float) -> None:
        self._position[0] += vx * 0.1
        self._position[1] += vy * 0.1
        print(f"[SIM] Move: vx={vx:.2f}, vy={vy:.2f} -> pos={self._position}")

    def rotate(self, vrot: float):
        self._rotation += vrot * 0.1
        print(f"[SIM] Rotate: vrot={vrot:.2f} -> rotation={self._rotation}")
    
    def stand_up(self) -> None:
        print("[SIM] Standing up")
        self._is_standing = True
    
    def stand_down(self) -> None:
        print("[SIM] Standing down")
        self._is_standing = False
    
    def stop_move(self) -> None:
        print("[SIM] Stopping movement")


class DogModule(ABC):
    """Base class for all dog functionality modules"""
    
    def __init__(self, name: str):
        self.name = name
        self._initialized = False
    
    @abstractmethod
    def initialize(self) -> None:
        """Initialize the module"""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """Clean shutdown of the module"""
        pass
    
    def is_initialized(self) -> bool:
        return self._initialized
    

T = TypeVar('T', bound=DogModule)

class ModuleType(Enum):
    """Enum for all available module types"""
    VIDEO = auto()
    MOVEMENT = auto()
    INPUT = auto()
    OCR = auto()
    AUDIO = auto()
    LIDAR = auto()
    GPS = auto()
    IMU = auto()

@dataclass
class ModuleDescriptor(Generic[T]):
    """Descriptor that links ModuleType to its implementation class"""
    module_type: ModuleType
    module_class: Type[T]
    display_name: str
    requires_sdk: bool = False
    
    def create_instance(self, *args, **kwargs) -> T:
        return self.module_class(*args, **kwargs)
    

class ModuleRegistry:
    """
    Central registry for all module types. Maps ModuleType enums to their implementation classes.
    """

    _descriptors: Dict[ModuleType, ModuleDescriptor] = {}

    @classmethod
    def register(cls, descriptor: ModuleDescriptor) -> None:
        """Register a module descriptor"""
        cls._descriptors[descriptor.module_type] = descriptor

    @classmethod
    def get_descriptor(cls, module_type: ModuleType) -> Optional[ModuleDescriptor]:
        """Get descriptor for a module type"""
        return cls._descriptors.get(module_type)

    @classmethod
    def get_class(cls, module_type: ModuleType) -> Optional[Type[DogModule]]:
        """Get implementation class for a module type"""
        descriptor = cls._descriptors.get(module_type)
        return descriptor.module_class if descriptor else None

    @classmethod
    def is_registered(cls, module_type: ModuleType) -> bool:
        """Check if a module type is registered"""
        return module_type in cls._descriptors

    @classmethod
    def get_list_available(cls, sdk_enabled: bool = False) -> list[ModuleType]:
        """List all available module types for the current mode"""
        return [
            mt for mt, desc in cls._descriptors.items()
            if not desc.requires_sdk or sdk_enabled
        ]
    


# =============================================================================
# CONCRETE MODULES
# =============================================================================

class VideoModule(DogModule):
    """Handles video capture from dog camera or webcam"""

    def __init__(self, use_sdk: bool = False):
        super().__init__("Video")
        self.use_sdk = use_sdk

        self.initialize()


    def initialize(self) -> None:
        if self._initialized:
            return
        
        if self.use_sdk:
            from unitree_sdk2py.go2.video.video_client import VideoClient

            print("[Video] Initializing VideoClient")
            self._video_client = VideoClient()
            self._video_client.SetTimeout(3.0)
            self._video_client.Init()
        else:
            print("[Video] Initializing webcam")
            self._webcam = cv2.VideoCapture(0)

            if not self._webcam.isOpened():
                raise RuntimeError("Failed to open webcam")
            
        self._initialized = True

    def get_image(self) -> tuple[int, Optional[np.ndarray]]:
        """Get image from video source. A code of 0 means success. A code of -1 means there was an internal failure."""
        if self.use_sdk and self._video_client:
            code, data = self._video_client.GetImageSample()
            if code != 0 or data is None:
                return -1, None
            
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return code, image
        
        if self._webcam:
            ret, image = self._webcam.read()
            if not ret:
                return -1, None
        
            return 0, image
        
        return -1, None
    
    def shutdown(self) -> None:
        if self._webcam:
            self._webcam.release()

        self._initialized = False


class InputModule(DogModule):
    """Handles controller input"""
    
    def __init__(self, use_sdk: bool = False):
        super().__init__("Input")
        self.use_sdk = use_sdk

        self.initialize()


    def initialize(self) -> None:
        if self._initialized or not self.use_sdk:
            return
        
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        from unitree_sdk2py.core.channel import ChannelSubscriber

        self._input_parser = _UnitreeRemoteControllerInputParser()
        self._callback_manager = _InputSignalCallbackManager()
        self._ControllerState = ControllerState
        self._InputSignal = InputSignal

        self._lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self._lowstate_subscriber.Init(self._process_input, 10)

    def register_callback(
        self,
        signal: InputSignal,
        callback: Callable[[ControllerState], None],
        name: Optional[str] = None,
        threshold: float = 0.1
    ):
        """Register a callback for a specific input signal"""        
        return self._callback_manager.register(signal, callback, name, threshold)
    
    def unregister_callback(
        self,
        signal: InputSignal,
        callback: Callable[[ControllerState], None]
    ):
        """Unregister a previously registered callback"""       
        self._callback_manager.unregister(signal, callback)
    
    def shutdown(self) -> None:
        """Clean up resources"""
        if self._lowstate_subscriber:
            self._lowstate_subscriber.Close()
        if self._callback_manager:
            self._callback_manager.shutdown()

    def _process_input(self, msg) -> ControllerState:
        """Process incoming controller input messages"""
        controller_state = self._input_parser.parse(msg.wireless_remote)
        self._callback_manager.handle(controller_state)

        return controller_state
    

class OCRModule(DogModule):
    """Handles optical character recognition"""

    def __init__(self):
        super().__init__("OCR")

        self.initialize()

    def initialize(self) -> None:
        if self._initialized:
            return

        self._initialized = True

    # followed this exactly: https://medium.com/@EnginDenizTangut/from-image-to-voice-building-an-ocr-tts-app-with-python-opencv-tesseract-5f5db8ea3b7b
    def extract_text_from_image(self, image: np.ndarray) -> tuple[str, np.ndarray]:
        """Extract text from an image"""
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.medianBlur(image, 3) # kernal to remove noise

        thresh = cv2.adaptiveThreshold(
            image, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 10
        )

        kernel = np.ones((2,2), np.uint8)
        image = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

        image = cv2.GaussianBlur(image, (5, 5), 0)
        _, image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        custom_config = r'--oem 3 --psm 6'
        text = pytesseract.image_to_string(image, lang='eng', config=custom_config)
        image = self._highlight_detected_result(image)

        return (text, image)
    

    def _highlight_detected_result(self, image: np.ndarray):
        ocr_data = pytesseract.image_to_data(image, output_type=pytesseract.Output.DICT)

        for i in range(len(ocr_data['text'])):
            if int(ocr_data['conf'][i]) > 60:
                x, y, w, h = (ocr_data["left"][i], ocr_data['top'][i], ocr_data['width'][i], ocr_data['height'][i])
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        return image 


    def shutdown(self) -> None:
        self._initialized = False


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


class MovementModule(DogModule):
    """High-level movement controls"""

    def __init__(self, hardware: HardwareInterface):
        super().__init__("Movement")
        self.hardware = hardware
        self.max_rotation = 5.0
        self.max_translation = 5.0

    def initialize(self) -> None:
        self._initialized = True

    def rotate(self, amount: float) -> None:
        """Rotate the dog (yaw)"""
        amount = max(-self.max_rotation, min(amount, self.max_rotation))
        self.hardware.rotate(amount)

    def move(self, amount_x: float = 0.0, amount_y: float = 0.0) -> None:
        """Shift the dog (forward/backward and lateral)"""
        amount_x = max(-self.max_translation, min(amount_x, self.max_translation))
        amount_y = max(-self.max_translation, min(amount_y, self.max_translation))
        self.hardware.move(amount_x, amount_y)

    def stand_up(self) -> None:
        """Stand up the dog"""
        self.hardware.stand_up()

    def stand_down(self) -> None:
        """Lay down the dog"""
        self.hardware.stand_down()

    def stop(self) -> None:
        """Stop all movement"""
        self.hardware.stop_move()

    def shutdown(self) -> None:
        self.stop()
        self._initialized = False


def register_all_default_modules():
    ModuleRegistry.register(ModuleDescriptor(
        ModuleType.VIDEO,
        VideoModule,
        "Video Capture",
        requires_sdk=False
    ))
    
    ModuleRegistry.register(ModuleDescriptor(
        ModuleType.MOVEMENT,
        MovementModule,
        "Movement Control",
        requires_sdk=False
    ))
    
    ModuleRegistry.register(ModuleDescriptor(
        ModuleType.OCR,
        OCRModule,
        "Optical Character Recognition",
        requires_sdk=False
    ))
    
    ModuleRegistry.register(ModuleDescriptor(
        ModuleType.AUDIO,
        AudioModule,
        "Text-to-Speech",
        requires_sdk=False
    ))
    
    ModuleRegistry.register(ModuleDescriptor(
        ModuleType.INPUT,
        InputModule,
        "Controller Input",
        requires_sdk=True  
    ))

register_all_default_modules()


class UnitreeGo2Controller:
    """    
    This is the main entry point that students/users interact with.
    Modules are accessed via enum types for compile-time safety.
    """

    def __init__(self, use_sdk: Optional[bool]):
        if use_sdk is None:
            use_sdk = self._detect_sdk()

        self.use_sdk = use_sdk

        self._shutdown_event = threading.Event()
        self._shutdown_lock = threading.Lock()
        self._cleanup_callbacks: List[Callable[[], None]] = []

        self.hardware: HardwareInterface = (
            UnitreeSDKHardware() if use_sdk else SimulatedHardware()
        )

        self._modules: Dict[ModuleType, DogModule] = {}
        self._register_all_modules()
        self._initialize_all()

        print(f"[Controller] Initialized in {'SDK' if use_sdk else 'SIMULATION'} mode\n")


    def _detect_sdk(self) -> bool:
        """Auto-detect if Unitree SDK is available"""
        try:
            import unitree_sdk2py
            return True
        
        except ImportError:
            return False
        

    def _initialize_all(self):
        self.hardware.initialize()

        self.input.register_callback(
            InputSignal.BUTTON_A,
            lambda _: self._shutdown_event.set(),
            "emergency_stop"
        )


    def _register_all_modules(self):
        self._add_module(ModuleType.VIDEO, use_sdk=self.use_sdk)
        self._add_module(ModuleType.MOVEMENT, hardware=self.hardware)
        self._add_module(ModuleType.OCR)
        self._add_module(ModuleType.AUDIO)

        if self.use_sdk:
            self._add_module(ModuleType.INPUT, use_sdk=self.use_sdk)


    def _add_module(self, module_type: ModuleType, **kwargs) -> None:
        """
        Add a new module to the controller using enum type.
        
        Args:
            module_type: The type of module to add (from ModuleType enum)
            **kwargs: Constructor arguments for the module
        
        Example:
            controller.add_module(ModuleType.VIDEO, use_sdk=True)
        """
        descriptor = ModuleRegistry.get_descriptor(module_type)
        if descriptor is None:
            raise ValueError(f"Module type {module_type} is not registered")
        
        if descriptor.requires_sdk and not self.use_sdk:
            print(f"[Controller] Warning: {module_type.name} requires SDK mode")
            return
        
        module = descriptor.create_instance(**kwargs)
        self._modules[module_type] = module
    

    def has_module(self, module_type: ModuleType) -> bool:
        """Check if a module is loaded"""
        return module_type in self._modules
    
    
    def list_available_modules(self) -> list[ModuleType]:
        """List all modules available for the current mode (SDK/simulation)"""
        return ModuleRegistry.get_list_available(self.use_sdk)
    

    @property
    def video(self) -> VideoModule:
        module = self._modules.get(ModuleType.VIDEO)
        if not isinstance(module, VideoModule):
            raise RuntimeError("Video module not loaded")
        
        return module
    
    @property
    def movement(self) -> MovementModule:
        module = self._modules.get(ModuleType.MOVEMENT)
        if not isinstance(module, MovementModule):
            raise RuntimeError("Movement module not loaded")
        
        return module
    
    @property
    def ocr(self) -> OCRModule:
        module = self._modules.get(ModuleType.OCR)
        if not isinstance(module, OCRModule):
            raise RuntimeError("OCR module not loaded")
        
        return module
    
    @property
    def audio(self) -> AudioModule:
        module = self._modules.get(ModuleType.AUDIO)
        if not isinstance(module, AudioModule):
            raise RuntimeError("Audio module not loaded")
        
        return module
    
    @property
    def input(self) -> InputModule:
        module = self._modules.get(ModuleType.INPUT)
        if not isinstance(module, InputModule):
            raise RuntimeError("Input module not loaded")
        
        return module
    

    def register_cleanup_callback(self, callback: Callable[[], None]):
        """Register a callback to run during shutdown"""
        self._cleanup_callbacks.append(callback)


    def safe_shutdown(self):
        """Perform safe shutdown of all systems"""
        print("\n[Controller] Starting safe shutdown...")
        self.movement.stop()
        
        with self._shutdown_lock:
            if not self._shutdown_event.is_set():
                self._shutdown_event.set()
            
            for callback in self._cleanup_callbacks:
                try:
                    callback()
                except Exception as e:
                    print(f"[Controller] Cleanup callback failed: {e}")
            
            for module_type, module in self._modules.items():
                try:
                    print(f"[Controller] Shutting down {module_type.name} module")
                    module.shutdown()
                except Exception as e:
                    print(f"[Controller] Failed to shutdown {module_type.name}: {e}")
            
            try:
                self.hardware.shutdown()
            except Exception as e:
                print(f"[Controller] Hardware shutdown failed: {e}")
            
            print("[Controller] Shutdown complete")

    def is_shutdown_requested(self) -> bool:
        """Check if shutdown has been requested"""
        return self._shutdown_event.is_set()

    