import struct
import sys
import threading
import time

from typing import Optional, TypeVar
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum

import cv2
import numpy as np


class DogFunctionalityWrapper:
    """Wrapper for Unitree dog functionality with fallback to webcam simulation."""
    
    def __init__(self):
        self.stop_event = threading.Event()
        self.use_unitree_sdk_methods = False
        self.max_rotation_amount = 5
        self.max_movement_amount = 5

        try:
            from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
            from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.go2.video.video_client import VideoClient

            print("[Init] Unitree SDK detected. Using real dog control.")
            self.use_unitree_sdk_methods = True

            print("[Init] Initializing ChannelFactory")
            if len(sys.argv) > 1:
                ChannelFactoryInitialize(0, sys.argv[1])
            else:
                ChannelFactoryInitialize(0)

            print("[Init] Creating SportClient")
            self.sport_client = SportClient()
            self.sport_client.Init()
            self.sport_client.SetTimeout(3.0)

            print("[Init] Connecting stop key override")
            self.handler = UnitreeRemoteController.CustomHandler(self.stop_event, LowState_, ChannelSubscriber)
            self.handler.init()

            print("[Init] Standing up and stopping movement")
            self.sport_client.StandUp()
            time.sleep(1)
            self.sport_client.StopMove()
            time.sleep(0.5)

            print("[Init] Initializing VideoClient\n")
            self.video_client = VideoClient()
            self.video_client.SetTimeout(3.0)
            self.video_client.Init()

        except ImportError:
            print("[Init] Unitree SDK not found. Running in simulated (webcam) mode.")
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise RuntimeError("Failed to open webcam")


    def get_image(self):
        """Get image from video source (dog camera or webcam)."""
        if self.use_unitree_sdk_methods:
            code, data = self.video_client.GetImageSample()
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
        if self.use_unitree_sdk_methods:
            amount = min(amount, self.max_rotation_amount)

            vx, vy, vz = 0.0, 0.0, amount
            self.sport_client.Move(vx, vy, vz)
            

    def shift_dog(self, amount_x=0, amount_y=0):
        """Shift the dog (forward/backward and lateral movement)."""
        if self.use_unitree_sdk_methods:
            amount_x = min(amount_x, self.max_movement_amount)
            amount_y = min(amount_y, self.max_movement_amount)
            
            vx, vy, vz = amount_x, amount_y, 0.0
            self.sport_client.Move(vx, vy, vz)


    def stop_dog(self):
        """Stop all dog movement."""
        print("[Dog] Stopping")

        if self.use_unitree_sdk_methods:
            self.sport_client.StopMove()


    def cleanup(self):
        """Clean up resources."""

        self.stop_event.clear()
        if not self.use_unitree_sdk_methods:
            self.cap.release()


    class StopMonitor:
        def __init__(self, stop_event: threading.Event, cleanup_callback=None):
            self.stop_event = stop_event
            self.cleanup_callback = cleanup_callback
            self.monitor_thread = None
            self.main_thread = threading.main_thread()

        def start(self):
            self.monitor_thread = threading.Thread(
                target=self._monitor_loop,
                daemon=True,
                name="StopMonitor"
            )

            self.monitor_thread.start()
            
        def _monitor_loop(self):
            while True:
                if self.stop_event.is_set():
                    print("\n[StopMonitor] Stop event detected! Initiating shutdown...")

                    if self.cleanup_callback:
                        try:
                            self.cleanup_callback()
                        except Exception as e:
                            print(f"[StopMonitor] Error during cleanup: {e}")

                    sys.exit(0)

                time.sleep(0.1)

        def cleanup(self):
            if (self.monitor_thread):
                self.monitor_thread.join()


class UnitreeRemoteController:
    """Handler for Unitree remote controller input."""
    
    def __init__(self):
        # Joystick axes
        self.Lx = 0           
        self.Rx = 0            
        self.Ry = 0            
        self.Ly = 0

        # Buttons
        self.L1 = 0
        self.L2 = 0
        self.R1 = 0
        self.R2 = 0
        self.A = 0
        self.B = 0
        self.X = 0
        self.Y = 0
        self.Up = 0
        self.Down = 0
        self.Left = 0
        self.Right = 0
        self.Select = 0
        self.F1 = 0
        self.F3 = 0
        self.Start = 0
        self.Start = 0

    def parse_button(self, data1, data2):
        """Parse button states from uint8 bit sequences."""
        self.R1 = (data1 >> 0) & 1
        self.L1 = (data1 >> 1) & 1
        self.Start = (data1 >> 2) & 1
        self.Select = (data1 >> 3) & 1
        self.R2 = (data1 >> 4) & 1
        self.L2 = (data1 >> 5) & 1
        self.F1 = (data1 >> 6) & 1
        self.F3 = (data1 >> 7) & 1

        self.A = (data2 >> 0) & 1
        self.B = (data2 >> 1) & 1
        self.X = (data2 >> 2) & 1
        self.Y = (data2 >> 3) & 1
        self.Up = (data2 >> 4) & 1
        self.Right = (data2 >> 5) & 1
        self.Down = (data2 >> 6) & 1
        self.Left = (data2 >> 7) & 1

    def parse_key(self, data):
        """Parse joystick axes from data."""
        lx_offset = 4
        self.Lx = struct.unpack('<f', data[lx_offset:lx_offset + 4])[0]
        rx_offset = 8
        self.Rx = struct.unpack('<f', data[rx_offset:rx_offset + 4])[0]
        ry_offset = 12
        self.Ry = struct.unpack('<f', data[ry_offset:ry_offset + 4])[0]
        ly_offset = 20
        self.Ly = struct.unpack('<f', data[ly_offset:ly_offset + 4])[0]

    def parse(self, remoteData):
        """Parse complete remote controller data."""
        self.parse_key(remoteData)
        self.parse_button(remoteData[2], remoteData[3])

    class CustomHandler:
        """Custom handler for remote controller callbacks."""
        
        def __init__(self, stop_event: threading.Event, LowState_, ChannelSubscriber):
            self.remote_controller = UnitreeRemoteController()
            self.stop_event = stop_event
            self.LowState_ = LowState_
            self.ChannelSubscriber = ChannelSubscriber

        def init(self):
            """Initialize the subscriber for low-level state."""
            self.lowstate_subscriber = self.ChannelSubscriber("rt/lf/lowstate", self.LowState_)
            self.lowstate_subscriber.Init(self.lowstate_callback, 10)
        
        def lowstate_callback(self, msg):
            """Callback for low-level state updates."""
            self.remote_controller.parse(msg.wireless_remote)
            if self.remote_controller.A == 1:
                self.stop_event.set()
