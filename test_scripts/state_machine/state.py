from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from typing import Optional, Set
from enum import Enum

from test_scripts.controller_interface import ControllerState

class State(ABC):
    @abstractmethod
    def on_enter(self) -> None:
        pass

    def update(self) -> None:
        pass

    def on_exit(self) -> None:
        pass


