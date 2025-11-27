from typing import Callable, Optional
from unitree_control.controller_input_control.callback_manager import _InputSignalCallbackManager, _UnitreeRemoteControllerInputParser
from unitree_control.controller_input_control.controller_state import ControllerState
from unitree_control.controller_input_control.input_signal import InputSignal
from unitree_control.core.control_modules import DogModule


class InputModule(DogModule):
    """Handles controller input"""
    
    def __init__(self, use_sdk: bool = False):
        super().__init__("Input")
        self.use_sdk = use_sdk

        self.initialize()


    def initialize(self) -> None:
        if self._initialized or not self.use_sdk:
            return
        
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
        from unitree_sdk2py.core.channel import ChannelSubscriber

        self._input_parser = _UnitreeRemoteControllerInputParser()
        self._callback_manager = _InputSignalCallbackManager()
        self._ControllerState = ControllerState
        self._InputSignal = InputSignal

        self._lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self._lowstate_subscriber.Init(self._process_input, 10)

    def register_callback(
        self,
        signal: InputSignal,
        callback: Callable[[ControllerState], None],
        name: Optional[str] = None,
        threshold: float = 0.1
    ):
        """Register a callback for a specific input signal"""        
        return self._callback_manager.register(signal, callback, name, threshold)
    
    def unregister_callback(
        self,
        signal: InputSignal,
        callback: Callable[[ControllerState], None]
    ):
        """Unregister a previously registered callback"""       
        self._callback_manager.unregister(signal, callback)
    
    def shutdown(self) -> None:
        """Clean up resources"""
        if self._lowstate_subscriber:
            self._lowstate_subscriber.Close()
        if self._callback_manager:
            self._callback_manager.shutdown()

    def _process_input(self, msg) -> ControllerState:
        """Process incoming controller input messages"""
        controller_state = self._input_parser.parse(msg.wireless_remote)
        self._callback_manager.handle(controller_state)

        return controller_state