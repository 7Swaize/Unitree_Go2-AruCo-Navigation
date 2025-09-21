from state import State
from predicate import Predicate
from transition import Transition
from typing import Set, Dict, Type, Optional

class StateMachine:
    class StateNode:
        def __init__(self, state: State) -> None:
            self.state = state
            self.transitions: Set[Transition] = set()

        def add_transition(self, to: State, condition: Predicate) -> None:
            self.transitions.add(Transition(to, condition))


    def __init__(self) -> None:
        self.currentStateNode: StateMachine.StateNode
        self.nodes: Dict[Type[State], StateMachine.StateNode] = {}
        self.any_transitions: Set[Transition] = set()

    def update(self) -> None:
        transition = self.get_transition()

        if transition is not None:
            self.change_state(transition.to)

        if self.currentStateNode is not None:
            self.currentStateNode.state.update()


    def set_state(self, state: State) -> None:
        self.change_state(state)

    def add_transition(self, fromState: State, toState: State, condition: Predicate) -> None:
        self.get_or_add_node(fromState).add_transition(toState, condition)

    def add_any_transition(self, toState: State, condition: Predicate) -> None:
        self.any_transitions.add(Transition(toState, condition))

    def get_or_add_node(self, state: State):
        node = self.nodes.get(type(state), None)

        if node is None:
            node = StateMachine.StateNode(state)
            self.nodes[type(state)] = node

        return node

    def get_transition(self) -> Optional[Transition]:
        for transition in self.any_transitions:
            if transition.condition.evaluate():
                return transition
            
        for transition in self.currentStateNode.transitions:
            if transition.condition.evaluate():
                return transition
            
        return None
    
    def change_state(self, state: State) -> None:
        if (self.currentStateNode.state == state):
            return
        
        previousState = self.currentStateNode.state
        nextState = self.nodes[type(state)].state

        if previousState is not None:
            previousState.on_exit()
        
        if nextState is not None:
            nextState.on_enter()
            
        self.currentStateNode = self.nodes[type(state)]
        