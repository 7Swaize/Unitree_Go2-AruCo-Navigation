import time
import sys
import threading

from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ # specific to using Go2
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_ # specific to using Go2
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient
from test_scripts.input_handler import CustomHandler
from test_scripts.controller_interface import ControllerState

def controller_callback(controller_state: ControllerState):
    if controller_state.a == 1:
        pass



def main():
    if len(sys.argv)>1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    sport_client = SportClient()
    sport_client.Init()
    '''
    stop_event = threading.Event()
    handler = CustomHandler])
    handler.init()
    
    vx, vy, vz = 0.3, 0.0, 0.0
    start_time = time.time()

    while time.time() - start_time < 3.0:
        if stop_event.is_set():
            break

        sport_client.Move(vx, vy, vz)
        time.sleep(0.05) 

    stop_event.clear()
    '''    
if __name__ == "__main__":
    main()