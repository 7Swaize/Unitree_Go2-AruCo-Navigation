from dataclasses import dataclass


@dataclass
class ControllerState:
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    
    l1: float = 0.0
    l2: float = 0.0
    r1: float = 0.0
    r2: float = 0.0
    
    a: float = 0.0
    b: float = 0.0
    x: float = 0.0
    y: float = 0.0
    
    up: float = 0.0
    down: float = 0.0
    left: float = 0.0
    right: float = 0.0
    
    select: float = 0.0
    start: float = 0.0
    f1: float = 0.0
    f3: float = 0.0

    changed: bool = False
