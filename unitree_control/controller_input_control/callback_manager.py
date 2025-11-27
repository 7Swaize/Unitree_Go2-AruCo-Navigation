from dataclasses import dataclass
import struct
from typing import Callable, Dict, List, Optional
from unitree_control.controller_input_control.controller_state import ControllerState
from unitree_control.controller_input_control.input_signal import InputSignal


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
        self._callbacks: Dict[InputSignal, List[_Callback]] = {}
        self._previous_state = ControllerState()

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

        self._callbacks.setdefault(signal, []).append(cb)
        return cb


    def unregister(self, signal: InputSignal, callback: Callable[[ControllerState], None]):
        if signal in self._callbacks:
            self._callbacks[signal] = [cb for cb in self._callbacks[signal] if cb.callback != callback]
            
            if not self._callbacks[signal]:
                del self._callbacks[signal]


    def handle(self, state: ControllerState):
        if not state.changed:
            return

        for signal, cb_list in self._callbacks.items():
            for cb in cb_list:
                if self._should_trigger(signal, cb, state):
                    self._execute(cb, state)

        self._previous_state = ControllerState(**state.__dict__)


    def shutdown(self):
        self._callbacks.clear()


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
        previous_value = getattr(self._previous_state, signal_attr, 0.0)

        if signal in _ANALOG_SIGNALS:
            return self._check_analog_trigger(current_value, previous_value, cb)
        
        return self._check_digital_trigger(current_value, previous_value, cb)
        

    def _stick_changed(self, stick: str, current_state: ControllerState, threshold: float) -> bool:
        x_attr = f"{stick}x"
        y_attr = f"{stick}y"
        
        current_x = getattr(current_state, x_attr, 0.0)
        current_y = getattr(current_state, y_attr, 0.0)
        previous_x = getattr(self._previous_state, x_attr, 0.0)
        previous_y = getattr(self._previous_state, y_attr, 0.0)
        
        # vector magnitude of x and y components
        distance = ((current_x - previous_x) ** 2 + (current_y - previous_y) ** 2) ** 0.5
        return distance > threshold


    def _check_analog_trigger(self, current: float, previous: float, cb: _Callback) -> bool:
        return abs(current - previous) > cb.threshold
    

    def _check_digital_trigger(self, current: float, previous: float, cb: _Callback) -> bool:
        return previous == 0.0 and current == 1.0
    

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
