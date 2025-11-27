from enum import Enum


class InputSignal(Enum):
    LEFT_STICK = "left_stick"
    RIGHT_STICK = "right_stick"
    LEFT_STICK_X = "lx"
    LEFT_STICK_Y = "ly"
    RIGHT_STICK_X = "rx"
    RIGHT_STICK_Y = "ry"
    
    LEFT_TRIGGER = "l2"
    RIGHT_TRIGGER = "r2"
    LEFT_BUMPER = "l1"
    RIGHT_BUMPER = "r1"
    
    BUTTON_A = "a"
    BUTTON_B = "b"
    BUTTON_X = "x"
    BUTTON_Y = "y"
    
    DPAD_UP = "up"
    DPAD_DOWN = "down"
    DPAD_LEFT = "left"
    DPAD_RIGHT = "right"
    
    SELECT = "select"
    START = "start"
    F1 = "f1"
    F3 = "f3"