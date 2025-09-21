from test_scripts.controller_interface import ControllerState

import threading
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional
from dataclasses import dataclass


@dataclass
class CallbackInfo:
    callback: Callable[[ControllerState], None]
    name: Optional[str] = None
    enabled: bool = True

class CallbackInfoFactory:
    def create_callback_info(
            self,
            callback: Callable[[ControllerState], None],
            name: Optional[str] = None
    ) -> CallbackInfo:
        callback_info = CallbackInfo(
            callback=callback,
            name=name or getattr(callback, '__name__', f"<lambda:{id(callback)}>"),
            enabled=True
        )

        return callback_info

class CallbackManager:
    def __init__(self) -> None:
        self.callbacks: Dict[int, CallbackInfo] = {}
        self.callback_info_factory = CallbackInfoFactory()
        self._execution_lock = threading.Lock()
        self._execution_pool = ThreadPoolExecutor(max_workers=4)

    def register_callback(self, callback_info: CallbackInfo) -> None:
        self.callbacks[id(callback_info.callback)] = callback_info

    def unregister_callback(self, callback: Callable[[ControllerState], None]):
        self.callbacks.pop(id(callback), None)

    def enable_callback(self, callback: Callable[[ControllerState], None], enabled: bool = True):
        cb_info = self.callbacks.get(id(callback))
        if cb_info:
            cb_info.enabled = enabled

    def clear_callbacks(self):
        self.callbacks.clear()

    def handle_callbacks(self, controller_state: ControllerState):
        if not controller_state.changed:
            return

        to_call = self.determine_callbacks_to_call(controller_state)

        with self._execution_lock:
            for callback_info in to_call:
                self._execution_pool.submit(
                    self._execute_callback_sync,
                    callback_info,
                    controller_state
                )

    def _execute_callback_sync(self, callback_info: CallbackInfo, controller_state: ControllerState) -> None:
        try:
            callback_info.callback(controller_state)
        except Exception as e:
            print(f"Callback {callback_info.name} failed: {e}")


    def determine_callbacks_to_call(self, controller_state: ControllerState) -> List[CallbackInfo]:
        to_call = []

        for callback in self.callbacks.values():
            if callback.enabled:
                to_call.append(callback)

        return to_call