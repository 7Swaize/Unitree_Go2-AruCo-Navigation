from typing import Optional

import cv2
import numpy as np
from unitree_control.core.control_modules import DogModule


class VideoModule(DogModule):
    """Handles video capture from dog camera or webcam"""

    def __init__(self, use_sdk: bool = False):
        super().__init__("Video")
        self.use_sdk = use_sdk

        self.initialize()


    def initialize(self) -> None:
        if self._initialized:
            return
        
        if self.use_sdk:
            from unitree_sdk2py.go2.video.video_client import VideoClient

            print("[Video] Initializing VideoClient")
            self._video_client = VideoClient()
            self._video_client.SetTimeout(3.0)
            self._video_client.Init()
        else:
            print("[Video] Initializing webcam")
            self._webcam = cv2.VideoCapture(0)

            if not self._webcam.isOpened():
                raise RuntimeError("Failed to open webcam")
            
        self._initialized = True

    def get_image(self) -> tuple[int, Optional[np.ndarray]]:
        """Get image from video source. A code of 0 means success. A code of -1 means there was an internal failure."""
        if self.use_sdk and self._video_client:
            code, data = self._video_client.GetImageSample()
            if code != 0 or data is None:
                return -1, None
            
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            return code, image
        
        if self._webcam:
            ret, image = self._webcam.read()
            if not ret:
                return -1, None
        
            return 0, image
        
        return -1, None
    
    def shutdown(self) -> None:
        if self._webcam:
            self._webcam.release()

        self._initialized = False