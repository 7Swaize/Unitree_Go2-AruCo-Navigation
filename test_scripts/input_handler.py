from test_scripts.controller_interface import ControllerState


from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


import struct
from collections.abc import Callable


class CustomHandler:
    class UnitreeRemoteControllerInputParser:
        def __init__(self):
            self.state = ControllerState()

        # Parses 2 bytes of data (data1 and data2) to detect which button inputs are pressed. 
        def _parse_buttons(self, data1, data2):
            mapping1 = ["R1", "L1", "Start", "Select", "R2", "L2", "F1", "F3"]
            mapping2 = ["A", "B", "X", "Y", "Up", "Right", "Down", "Left"]

            for i, name in enumerate(mapping1):
                setattr(self, name, (data1 >> i) & 1)

            for i, name in enumerate(mapping2):
                setattr(self, name, (data2 >> i) & 1)

        # Parses all 24 bytes of data. Bytes 2 and 3 represent button inputs.
        def _parse_key(self,data):
            lx_offset = 4
            self.lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
            rx_offset = 8
            self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
            ry_offset = 12
            self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
            L2_offset = 16
            L2 = struct.unpack('<f', data[L2_offset:L2_offset + 4])[0] # Placeholder，unused
            ly_offset = 20
            self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

        def parse(self, remoteData):
            self._parse_key(remoteData)
            self._parse_buttons(remoteData[2], remoteData[3])

    def __init__(self, callbacks: list[Callable[[ControllerState], None]]):
        self.input_parser = CustomHandler.UnitreeRemoteControllerInputParser()
        self.callbacks = callbacks

    def init(self):
        self.lowstate_subscriber = ChannelSubscriber("rt/lf/lowstate", LowState_)
        self.lowstate_subscriber.Init(self._lowstate_callback, 10)

    def add_callbacks(self, callbacks: list[Callable[[ControllerState], None]]):
        self.callbacks.extend(callbacks)

    def remove_callbacks(self, callbacks: list[Callable[[ControllerState], None]]):
        for callback in callbacks:
            if callback in self.callbacks:
                self.callbacks.remove(callback)

    def _handle_callbacks(self, controller_state: ControllerState):
        for callback in self.callbacks:
            callback(controller_state)

    def _lowstate_callback(self, msg: LowState_):
        self.input_parser.parse(msg.wireless_remote)
        self._handle_callbacks(self.input_parser.state)