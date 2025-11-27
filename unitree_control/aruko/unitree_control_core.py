import ast
import inspect
import struct
import sys
import textwrap
import threading
import time
from abc import ABC, ABCMeta, abstractmethod
from enum import Enum

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, final

import cv2
import numpy as np
import pytesseract
import pyttsx3

# TTS: https://medium.com/@vndee.huynh/build-your-own-voice-assistant-and-run-it-locally-whisper-ollama-bark-c80e6f815cba
# Video Streaming: https://medium.com/@prasadparit006/python-project-building-a-simple-video-streaming-server-using-opencv-c8a9d9ba86d2

# Digging into Dog: https://www.darknavy.org/darknavy_insight/the_jailbroken_unitree_robot_dog

# Github Repo Searcher: https://github.com/search?type=Code


# =============================================================================
# RULES:
#
# - Classes and class attributes starting with "_" are internal and should NOT 
#   be accessed or modified.
#
# - Classes and attributes without a leading "_" are EXPOSED: they can be 
#   accessed or inherited from, but should not be tampered with directly.
# =============================================================================
