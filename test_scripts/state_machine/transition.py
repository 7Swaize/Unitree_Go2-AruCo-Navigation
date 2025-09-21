from abc import ABC
from state import State
from predicate import Predicate


class Transition:
    def __init__(self, to: State, condition: Predicate) -> None:
        self.to = to
        self.condition = condition

