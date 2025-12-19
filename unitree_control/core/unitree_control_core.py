from typing import Callable, List, Optional, Dict
import threading

from unitree_control.controller_input_control.input_signal import InputSignal
from unitree_control.core.base_module import DogModule
from unitree_control.core.module_registry import AudioModule, InputModule, ModuleRegistry, ModuleType, MovementModule, OCRModule, VideoModule
from unitree_control.core.hardware_control import HardwareInterface, SimulatedHardware, UnitreeSDKHardware
from unitree_control.lidar_control.decoder import LIDARModule


# TTS: https://medium.com/@vndee.huynh/build-your-own-voice-assistant-and-run-it-locally-whisper-ollama-bark-c80e6f815cba
# Digging into Dog: https://www.darknavy.org/darknavy_insight/the_jailbroken_unitree_robot_dog

# Github Repo Searcher: https://github.com/search?type=Code

# Some very interesting turtorial with the G02: https://hackmd.io/@c12hQ00ySVi6JYIERU7bCg/ByAOr12qJg


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

        self._hardware: HardwareInterface = (
            UnitreeSDKHardware() if use_sdk else SimulatedHardware()
        )
        self._hardware.initialize()

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
        self._hardware.initialize()

        if self.use_sdk:
            self.input.register_callback(
                InputSignal.BUTTON_A,
                lambda _: self._shutdown_event.set(),
                "emergency_stop"
            )


    def _register_all_modules(self):
        self._add_module(ModuleType.VIDEO, use_sdk=self.use_sdk)
        self._add_module(ModuleType.MOVEMENT, hardware=self._hardware)
        self._add_module(ModuleType.OCR)
        self._add_module(ModuleType.AUDIO)

        if self.use_sdk:
            self._add_module(ModuleType.INPUT, use_sdk=self.use_sdk)
            # self._add_module(ModuleType.LIDAR, use_sdk=self.use_sdk, visualize_lidar=False)


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
    
    
    def get_available_modules(self) -> list[ModuleType]:
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
    
    @property
    def lidar(self) -> LIDARModule:
        module = self._modules.get(ModuleType.LIDAR)
        if not isinstance(module, LIDARModule):
            raise RuntimeError("LIDAR module not loaded")
        
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

            for module_type, module in self._modules.items():
                try:
                    module.shutdown()
                except Exception as e:
                    print(f"[Controller] Failed to shutdown {module_type.name}: {e}")
            
            for callback in self._cleanup_callbacks:
                try:
                    callback()
                except Exception as e:
                    print(f"[Controller] Cleanup callback failed: {e}")
            
            try:
                self._hardware.shutdown()
            except Exception as e:
                print(f"[Controller] Hardware shutdown failed: {e}")
            
            print("[Controller] Shutdown complete")


    def is_shutdown_requested(self) -> bool:
        """Check if shutdown has been requested"""
        return self._shutdown_event.is_set()

    