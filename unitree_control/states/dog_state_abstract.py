from abc import ABC, abstractmethod
from typing import Any, final

from unitree_control.core.unitree_control_core_v2 import UnitreeGo2Controller
from unitree_control.states.validation import _CancellableMeta


class DogStateAbstract(ABC, metaclass=_CancellableMeta):
    """Abstract base class for dog behavior states."""
    
    def __init__(self, functionality_wrapper: UnitreeGo2Controller):
        super().__init__()
        self.unitree_controller = functionality_wrapper
        self.is_running = False
        self.should_cancel = False

    @abstractmethod
    def execute(self, *args, **kwargs) -> Any: # might have to preserve var args in source gen?
        pass

    def on_enter(self):
        self.is_running = True
        self.should_cancel = False

    def on_exit(self):
        self.is_running = False

    @final
    def check_shutdown(self):
        if self.unitree_controller is None:
            return
        
        if self.unitree_controller._shutdown_event.is_set():
            raise KeyboardInterrupt()

    def cancel(self):
        self.should_cancel = True

    