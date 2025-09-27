import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

from test_scripts.callback_manager import InputHandler, InputSignal
from test_scripts.input_handle import ControllerState


class Main:
    def __init__(self) -> None:
        self.a_key_pressed: bool = False
        self.sport_client = SportClient()
        self.input_handler = InputHandler()

    def init(self):
        if len(sys.argv) > 1:
            ChannelFactoryInitialize(0, sys.argv[1])
        else:
            ChannelFactoryInitialize(0)

        self.sport_client.Init()

    def main(self) -> None:
        self.input_handler.register_callback(
            InputSignal.BUTTON_A,
            self.on_button_A,
            name="on_button_",
        )

        vx, vy, vz = 0.3, 0.0, 0.0
        start_time = time.time()

        while time.time() - start_time < 3.0:
            if self.a_key_pressed:
                break
                
            self.sport_client.Move(vx, vy, vz)
            time.sleep(0.05)

        self.input_handler.shutdown()

    def on_button_A(self, state: ControllerState):
        global a_pressed
        a_pressed = True
    

if __name__ == "__main__":
    main = Main()
    main.main()