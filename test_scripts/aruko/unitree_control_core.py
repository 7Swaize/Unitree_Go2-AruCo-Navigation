from abc import ABC, abstractmethod
import struct
import sys
import time

from typing import Any, Callable, Dict, List, Optional
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np


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



class DogFunctionalityWrapper:
    """Wrapper for Unitree dog functionality with fallback to webcam simulation."""
    
    def __init__(self, shutdown_callback: Callable[[ControllerState], None]):
        self._use_unitree_sdk_methods = False
        self._max_rotation_amount = 5
        self._max_movement_amount = 5

        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
            from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.go2.video.video_client import VideoClient

            print("[Init] Unitree SDK detected. Using real dog control.")
            self._use_unitree_sdk_methods = True

            print("[Init] Initializing ChannelFactory")
            if len(sys.argv) > 1:
                ChannelFactoryInitialize(0, sys.argv[1])
            else:
                ChannelFactoryInitialize(0)

            print("[Init] Creating SportClient")
            self._sport_client = SportClient()
            self._sport_client.Init()
            self._sport_client.SetTimeout(3.0)

            print("[Init] Connecting stop key override")
            self.input_handler = _InputHandler(ChannelSubscriber, LowState_)
            self.input_handler.register_callback(InputSignal.BUTTON_A, shutdown_callback, "on_emergency_stop")
            self._root_cleanup_callback = shutdown_callback

            print("[Init] Standing up and stopping movement")
            self._sport_client.StandUp()
            time.sleep(1)
            self._sport_client.StopMove()
            time.sleep(0.5)

            print("[Init] Initializing VideoClient\n")
            self._video_client = VideoClient()
            self._video_client.SetTimeout(3.0)
            self._video_client.Init()

        except ImportError:
            print("[Init] Unitree SDK not found. Running in simulated (webcam) mode.")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise RuntimeError("Failed to open webcam")


    def get_image(self):
        """Get image from video source (dog camera or webcam)."""
        if self._use_unitree_sdk_methods:
            code, data = self._video_client.GetImageSample()
            if code != 0 or data is None:
                return -1, None

            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return code, image

        ret, image = self.cap.read()
        if not ret:
            return -1, None
        
        return 0, image


    def rotate_dog(self, amount):
        """Rotate the dog (yaw rotation)."""
        if self._use_unitree_sdk_methods:
            amount = min(amount, self._max_rotation_amount)

            vx, vy, vz = 0.0, 0.0, amount
            self._sport_client.Move(vx, vy, vz)
            

    def shift_dog(self, amount_x=0, amount_y=0):
        """Shift the dog (forward/backward and lateral movement)."""
        if self._use_unitree_sdk_methods:
            amount_x = min(amount_x, self._max_movement_amount)
            amount_y = min(amount_y, self._max_movement_amount)
            
            vx, vy, vz = amount_x, amount_y, 0.0
            self._sport_client.Move(vx, vy, vz)


    def stop_dog(self):
        """Stop all dog movement."""
        print("[Dog] Stopping")

        if self._use_unitree_sdk_methods:
            self._sport_client.StopMove()


    def safe_shutdown(self):
        print("\n[Shutdown] Starting Safe Shutdown...")
        self._root_cleanup_callback(ControllerState()) # default controller state
        self.cleanup()
        

    def cleanup(self):
        """Clean up resources."""
        self.input_handler.shutdown()
        self.stop_dog()
        
        if not self._use_unitree_sdk_methods:
            self.cap.release()



_ANALOG_SIGNALS = {
    InputSignal.LEFT_STICK_X, InputSignal.LEFT_STICK_Y,
    InputSignal.RIGHT_STICK_X, InputSignal.RIGHT_STICK_Y,
    InputSignal.LEFT_TRIGGER, InputSignal.RIGHT_TRIGGER
}

@dataclass
class _Callback:
    callback: Callable[[ControllerState], None]
    signal: InputSignal
    name: Optional[str] = None
    threshold: float = 0.1


class _InputSignalCallbackManager:
    def __init__(self):
        self.callbacks: Dict[InputSignal, List[_Callback]] = {}
        self.previous_state = ControllerState()

    def register(
            self,
            signal: InputSignal,
            callback: Callable[[ControllerState], None],
            name: Optional[str] = None,
            threshold: float = 0.1    
        ) -> _Callback:
        cb = _Callback(
            callback=callback,
            signal=signal,
            name=name or getattr(callback, "__name__", f"<lambda:{id(callback)}>"),
            threshold=threshold
        )

        self.callbacks.setdefault(signal, []).append(cb)
        return cb


    def unregister(self, signal: InputSignal, callback: Callable[[ControllerState], None]):
        if signal in self.callbacks:
            self.callbacks[signal] = [cb for cb in self.callbacks[signal] if cb.callback != callback]
            
            if not self.callbacks[signal]:
                del self.callbacks[signal]


    def handle(self, state: ControllerState):
        if not state.changed:
            return

        for signal, cb_list in self.callbacks.items():
            for cb in cb_list:
                if self._should_trigger(signal, cb, state):
                    self._execute(cb, state)

        self.previous_state = ControllerState(**state.__dict__)


    def shutdown(self):
        self.callbacks.clear()


    def _execute(self, cb: _Callback, state: ControllerState):
        try:
            cb.callback(state)
        except Exception as e:
            print(f"[CallbackManager] Callback {cb.name} failed: {e}")


    def _should_trigger(self, signal: InputSignal, cb: _Callback, current_state: ControllerState) -> bool:
        if signal == InputSignal.LEFT_STICK:
            return self._stick_changed('l', current_state, cb.threshold)
            
        if signal == InputSignal.RIGHT_STICK:
            return self._stick_changed('r', current_state, cb.threshold)
        
        signal_attr = signal.value
        if not hasattr(current_state, signal_attr):
            return False
        
        current_value = getattr(current_state, signal_attr, 0.0)
        previous_value = getattr(self.previous_state, signal_attr, 0.0)

        if signal in _ANALOG_SIGNALS:
            return self._check_analog_trigger(current_value, previous_value, cb)
        
        return self._check_digital_trigger(current_value, previous_value, cb)
        

    def _stick_changed(self, stick: str, current_state: ControllerState, threshold: float) -> bool:
        x_attr = f"{stick}x"
        y_attr = f"{stick}y"
        
        current_x = getattr(current_state, x_attr, 0.0)
        current_y = getattr(current_state, y_attr, 0.0)
        previous_x = getattr(self.previous_state, x_attr, 0.0)
        previous_y = getattr(self.previous_state, y_attr, 0.0)
        
        # vector magnitude of x and y components
        distance = ((current_x - previous_x) ** 2 + (current_y - previous_y) ** 2) ** 0.5
        return distance > threshold


    def _check_analog_trigger(self, current: float, previous: float, cb: _Callback) -> bool:
        return abs(current - previous) > cb.threshold
    

    def _check_digital_trigger(self, current: float, previous: float, cb: _Callback) -> bool:
        return previous == 0.0 and current == 1.0


class _InputHandler:
    def __init__(self, ChannelSubscriber, LowState_):
        self.input_parser = _UnitreeRemoteControllerInputParser()
        self.callback_manager = _InputSignalCallbackManager()
            
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._process_input, 10)


    def register_callback(
            self,
            signal: InputSignal,
            callback: Callable[[ControllerState], None],
            name: Optional[str] = None,
            threshold: float = 0.1
        ):
        return self.callback_manager.register(signal, callback, name, threshold)
    
    
    def unregister_callback(self, signal: InputSignal, callback: Callable[[ControllerState], None]):
        self.callback_manager.unregister(signal, callback)

    
    def shutdown(self) -> None:
        self.lowstate_subscriber.Close() # cleanup resources and unsubscribe from Lowstate_ topic
        self.callback_manager.shutdown()

    
    def _process_input(self, msg) -> ControllerState:
        controller_state = self.input_parser.parse(msg.wireless_remote)
        self.callback_manager.handle(controller_state)
        return controller_state


class _UnitreeRemoteControllerInputParser:
    def __init__(self):
        self._state = ControllerState()

    def _parse_buttons(self, data1: int, data2: int):
        mapping1 = {
            0: "r1", 1: "l1", 2: "start", 3: "select",
            4: "r2", 5: "l2", 6: "f1", 7: "f3"
        }
        mapping2 = {
            0: "a", 1: "b", 2: "x", 3: "y",
            4: "up", 5: "right", 6: "down", 7: "left"
        }

        for i, attr in mapping1.items():
            setattr(self._state, attr, (data1 >> i) & 1)

        for i, attr in mapping2.items():
            setattr(self._state, attr, (data2 >> i) & 1)


    def _parse_analog(self, data: bytes):
        self._state.lx = struct.unpack('<f', data[4:8])[0]
        self._state.ly = struct.unpack('<f', data[20:24])[0]
        self._state.rx = struct.unpack('<f', data[8:12])[0]
        self._state.ry = struct.unpack('<f', data[12:16])[0]
        self._state.l2 = struct.unpack('<f', data[16:20])[0]


    # remote_data is some type that is an array of 8-bit-seqs
    def parse(self, remote_data) -> ControllerState:
        self._previous_state = ControllerState(**self._state.__dict__)
        
        self._parse_analog(remote_data)
        self._parse_buttons(remote_data[2], remote_data[3])

        # just compares if there is a change
        self._state.changed = any(
            getattr(self._state, attr) != getattr(self._previous_state, attr)
            for attr in vars(self._state) if attr != "changed"
        )
    
        return self._state
