from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, Generic, Optional, Type, TypeVar

from unitree_control.audio_control.audio_module import AudioModule
from unitree_control.controller_input_control.input_module import InputModule
from unitree_control.movement.movement_module import MovementModule
from unitree_control.ocr_control.ocr_module import OCRModule
from unitree_control.video_control.video_module import VideoModule


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