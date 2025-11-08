import time
import sys
import struct
import math
import threading
import cv2
import numpy as np

from enum import Enum
from typing import Any
from abc import ABC, abstractmethod


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)


def extract_corners_points(corners):
    """Extract the four corner points from ArUco marker corners."""
    x0, y0 = corners[0]
    x1, y1 = corners[1]
    x2, y2 = corners[2]
    x3, y3 = corners[3]
    return (x0, y0), (x1, y1), (x2, y2), (x3, y3)


def determine_horizontal_offset(image, fiducial_center, screen_width):
    """Calculate horizontal offset from center of image."""
    midX = screen_width // 2
    cx = int(fiducial_center[0])
    return midX - cx


def compute_fiducial_center_point(p0, p1):
    """Compute center point between two diagonal corners."""
    mx = int((p0[0] + p1[0]) / 2)
    my = int((p0[1] + p1[1]) / 2)
    return (mx, my)


def dist_between_points(a, b):
    """Calculate distance between two points."""
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)


def get_fiducial_area_from_corners(c0, c1, c2, c3):
    """Calculate approximate area of the marker."""
    width = dist_between_points(c0, c1)
    height = dist_between_points(c1, c2)
    return width * height


def get_aruko_marker(image):
    """Detect ArUco markers in an image."""
    if len(image.shape) == 2: 
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    return corners, ids, rejected


def extract_data_from_marker(image, corners, ids):
    """Extract all relevant data from detected marker."""
    if corners is None or len(corners) == 0 or ids is None or len(ids) == 0:
        return -1, [(0, 0)], (0, 0), 0, 0

    bounds = [(int(x), int(y)) for x, y in corners[0][0]]
    marker_id = int(ids[0][0])

    c0, c1, c2, c3 = extract_corners_points(corners[0][0])
    fiducial_center = compute_fiducial_center_point(c0, c2)
    h_offset = determine_horizontal_offset(image, fiducial_center, image.shape[1])
    fiducial_area = get_fiducial_area_from_corners(c0, c1, c2, c3)

    return marker_id, bounds, fiducial_center, h_offset, fiducial_area


class MarkerMappings(Enum):
    """Mapping of marker IDs to commands."""
    STOP_MARKER = 0
    RIGHT_MARKER = 1
    LEFT_MARKER = 2

# add a gaurd clause in all off the dog methods to make sure no significantly high value is passed in
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

            print("[Init] Initializing VideoClient")
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
        print("[Cleanup] Cleaning up resources.")

        self.stop_event.clear()
        if not self.use_unitree_sdk_methods:
            self.cap.release()


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
                print("[Remote] Emergency stop triggered (A button)")
                self.stop_event.set()


class DogStateAbstract(ABC):
    """Abstract base class for dog behavior states."""
    
    def __init__(self, functionality_wrapper: DogFunctionalityWrapper):
        super().__init__()
        self.functionality_wrapper = functionality_wrapper
        self.is_running = False
        self.should_cancel = False

    @abstractmethod
    def execute(self) -> Any:
        pass

    def on_enter(self):
        self.is_running = True
        self.should_cancel = False

    def on_exit(self):
        self.is_running = False

    def cancel(self):
        self.should_cancel = True


class ScanForMarkerState(DogStateAbstract):
    def __init__(self, functionality_wrapper, window_title, search_range, search_delta, max_sweeps=3):
        super().__init__(functionality_wrapper)

        self.window_title = window_title
        self.search_range = search_range
        self.search_delta = search_delta
        self.max_sweeps = max_sweeps

    def execute(self):
        """Execute scanning behavior."""
        fiducial_found = False
        marker_id = -1
        current_angle = 0  
        sweeps_done = 0
        direction = 1

        half_range = self.search_range / 2
        sweep_limit = half_range

        print(f"[Scan] Starting scan (range={self.search_range}, sweeps={self.max_sweeps})")

        while not fiducial_found and sweeps_done < self.max_sweeps:
            if self.functionality_wrapper.stop_event.is_set():
                break

            code, image = self.functionality_wrapper.get_image()
            if code != 0 or image is None:
                continue

            corners, ids, _ = get_aruko_marker(image)
            marker_id, bounds, _, _, _ = extract_data_from_marker(image, corners, ids)
            fiducial_found = (marker_id != -1)

            self.update_visuals(image, fiducial_found, marker_id, bounds, current_angle, sweeps_done)
            if cv2.waitKey(1) & 0xFF == 27:
                break

            self.functionality_wrapper.rotate_dog(direction * self.search_delta)
            current_angle += direction * self.search_delta
            next_angle = current_angle + direction * self.search_delta

            # Check if next step would exceed the sweep limit
            if abs(next_angle) >= sweep_limit:
                next_angle = direction * sweep_limit
                sweeps_done += 1
                direction *= -1  # Reverse sweep direction
                sweep_limit = self.search_range

        self.functionality_wrapper.stop_dog()
        
        if fiducial_found:
            print(f"[Scan] Marker {marker_id} found!")
        else:
            print("[Scan] No marker detected")

        return fiducial_found, marker_id
    

    def update_visuals(self, image, fiducial_found, marker_id, bounds, current_angle, sweeps_done):
        if fiducial_found:
            cv2.putText(image, f"ID: {marker_id}", bounds[0],
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(image, "FOUND!", (bounds[0][0], bounds[0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Display sweep info
        cv2.putText(image, f"Sweep: {sweeps_done + 1}/{self.max_sweeps}", (5, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(image, f"Angle: {current_angle:.1f}", (5, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.imshow(self.window_title, image) 


class WalkToMarkerState(DogStateAbstract):
    """State for walking toward a detected marker."""
    
    def __init__(self, functionality_wrapper, window_title):
        super().__init__(functionality_wrapper)
        self.window_title = window_title

    def execute(self):
        """Execute walking behavior."""
        arrived = False
        marker_id = -1

        forward_step = 1
        rotate_step = 1
        h_offset_threshold = 30 # i just played around with this number; in the future we would need a system to calculate this
        fiducial_area_threshold = 80000 # i just played around with this number; in the future we would need a system to calculate this

        print("[Walk] Starting approach to marker")

        while not arrived:
            if self.functionality_wrapper.stop_event.is_set():
                break

            code, image = self.functionality_wrapper.get_image()
            if code != 0 or image is None:
                continue

            corners, ids, _ = get_aruko_marker(image)
            marker_id, bounds, fiducial_center, h_offset, fiducial_area = extract_data_from_marker(image, corners, ids)
            
            is_centered = self.correct_alignment_to_marker(
                marker_id, h_offset, h_offset_threshold, rotate_step
            )

            if is_centered:
                arrived = self.approach_marker(
                    fiducial_area, fiducial_area_threshold, forward_step
                )

            self.update_visuals(image, marker_id, h_offset, fiducial_area, is_centered)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        if arrived:
            print(f"[Walk] Arrived at marker {marker_id}")

        self.functionality_wrapper.stop_dog()
        return arrived, marker_id
    

    def correct_alignment_to_marker(self, marker_id, h_offset, h_offset_threshold, rotate_step):
        """Align the dog to center on the marker."""
        if marker_id == -1:
            return False
        
        if abs(h_offset) <= h_offset_threshold:
            return True
        elif h_offset < -h_offset_threshold:
            self.functionality_wrapper.rotate_dog(-rotate_step)
        elif h_offset > h_offset_threshold:
            self.functionality_wrapper.rotate_dog(rotate_step)

        return False

    def approach_marker(self, fiducial_area, fiducial_area_threshold, forward_step):
        """Move forward toward the marker."""
        if fiducial_area < fiducial_area_threshold:
            self.functionality_wrapper.shift_dog(forward_step)
            return False
        else:
            self.functionality_wrapper.stop_dog()
            return True
        
    def update_visuals(self, image, marker_id, h_offset, fiducial_area, is_centered):
        if marker_id != -1:
            cv2.putText(image, f"ID: {marker_id}", (5, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(image, f"Offset: {h_offset}", (5, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(image, f"Area: {int(fiducial_area)}", (5, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(image, "Centered" if is_centered else "Aligning...", (5, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if is_centered else (0, 165, 255), 2)
        else:
            cv2.putText(image, "MARKER LOST!", (5, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow(self.window_title, image)
      

class RespondToMarkerState(DogStateAbstract):
    """State for responding to marker commands."""
    
    def __init__(self, functionality_wrapper, window_title, marker_id=-1):
        super().__init__(functionality_wrapper)
        self.window_title = window_title
        self.marker_id = marker_id

    def set_marker_id(self, marker_id):
        """Update the marker ID to respond to."""
        self.marker_id = marker_id

    def execute(self):
        """Execute response behavior based on marker ID."""
        print(f"[Respond] Responding to marker {self.marker_id}")
    
        if self.marker_id == MarkerMappings.STOP_MARKER.value:
            print("[Respond] STOP command received")
            self.functionality_wrapper.stop_dog() 
            return True
        
        elif self.marker_id in (MarkerMappings.LEFT_MARKER.value, MarkerMappings.RIGHT_MARKER.value):
            rotation_amount = 90
            step_amount = 1

            if self.marker_id == MarkerMappings.RIGHT_MARKER.value:
                step_amount = step_amount * 1
            else:
                step_amount = step_amount * -1

            for _ in range(rotation_amount):
                if self.functionality_wrapper.stop_event.is_set():
                    break

                self.functionality_wrapper.rotate_dog(step_amount)

        
            self.functionality_wrapper.stop_dog()
            time.sleep(1)
            
        return False
        
    def update_visuals(self):
        code, image = self.functionality_wrapper.get_image()
        if code != 0 or image is None:
            return
        
        cv2.imshow(self.window_title, image)

## MOVE
def mainMove():
    input("Press Enter to continue...")

    functionality_wrapper = DogFunctionalityWrapper()
    window_title = "Aruko Detection"
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)

    # Configuration
    search_range = 70
    search_delta = 0.5
    max_sweeps = 3
    last_marker_id = -1
    command_running = False

    scan_state = ScanForMarkerState(functionality_wrapper, window_title, search_range, search_delta, max_sweeps)
    walk_state = WalkToMarkerState(functionality_wrapper, window_title)
    respond_state = RespondToMarkerState(functionality_wrapper, window_title)

    print()
    print("Controls:")
    print("  [s] - Scan for marker")
    print("  [w] - Walk to marker")
    print("  [r] - Respond to marker")
    print("  [q] - Quit")
    print()
    
    try:
        while True:
            if not command_running:
                code, image = functionality_wrapper.get_image()
                if code == 0 and image is not None:
                    cv2.putText(image, "Waiting for command...", (5, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.imshow(window_title, image)
                    cv2.waitKey(1)
        
            key = input("Enter command (s/w/r/q): ").strip().lower()

            if key == 's':
                command_running = True
                fiducial_found, marker_id = scan_state.execute()
                
                if fiducial_found:
                    last_marker_id = marker_id
                
                command_running = False

            elif key == 'w':
                # Commented out for now. In an actual implementation (without test components) we would include this.
                # if last_marker_id == -1:
                    # continue
                    
                command_running = True
                has_arrived, marker_id = walk_state.execute()
                
                if has_arrived:
                    last_marker_id = marker_id
                
                command_running = False

            elif key == 'r':
                if last_marker_id == -1:
                    continue

                command_running = True
                respond_state.set_marker_id(last_marker_id)
                should_exit = respond_state.execute()
                command_running = False
                
                if should_exit:
                    break

            elif key == "q":
                break
            
    finally:
        cv2.destroyAllWindows()
        functionality_wrapper.stop_dog()
        time.sleep(0.5)
        functionality_wrapper.cleanup()



if __name__ == "__main__":
    mainMove()
