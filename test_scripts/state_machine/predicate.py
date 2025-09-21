from abc import ABC, abstractmethod
from collections.abc import Callable


class Predicate(ABC):
    @abstractmethod
    def evaluate(self) -> bool:
        pass

class FuncPredicate(Predicate):
    def __init__(self, func: Callable[[], bool]) -> None:
        super().__init__()
        
        self.func = func
    
    def evaluate(self) -> bool:
        return self.func()
