import time
import cv2
import os
import signal

from aruko_helpers import *
from unitree_control_core import *

from enum import Enum


class MarkerMappings(Enum):
    """Mapping of marker IDs to commands."""
    STOP_MARKER = 0
    RIGHT_MARKER = 1
    LEFT_MARKER = 2


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
        h_offset_threshold = 50 # i just played around with this number; in the future we would need a system to calculate this
        fiducial_area_threshold = 80000 # i just played around with this number; in the future we would need a system to calculate this

        print("[Walk] Starting approach to marker")

        while not arrived:
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
            time.sleep(0.5)
            self.functionality_wrapper._sport_client.StandDown()
            return True
        
        elif self.marker_id in (MarkerMappings.LEFT_MARKER.value, MarkerMappings.RIGHT_MARKER.value):
            rotation_amount = 150
            step_amount = 1

            if self.marker_id == MarkerMappings.RIGHT_MARKER.value:
                step_amount = step_amount * 1
            else:
                step_amount = step_amount * -1

            for _ in range(rotation_amount):
                self.functionality_wrapper.rotate_dog(step_amount)
                time.sleep(0.02)  # <-- Allow movement to persist (tune as needed)

        
            self.functionality_wrapper.stop_dog()
            time.sleep(1)
            
        return False
        
    def update_visuals(self):
        code, image = self.functionality_wrapper.get_image()
        if code != 0 or image is None:
            return
        
        cv2.imshow(self.window_title, image)



class Main:
    def __init__(self):
        self.functionality_wrapper = DogFunctionalityWrapper(self.shutdown_callback)
        

    def main_move(self):
        input("Press Enter to start autonomous movement...")

        window_title = "Aruko Detection"
        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)

        search_range = 70
        search_delta = 0.5
        max_sweeps = 3

        scan_state = ScanForMarkerState(self.functionality_wrapper, window_title, search_range, search_delta, max_sweeps)
        walk_state = WalkToMarkerState(self.functionality_wrapper, window_title)
        respond_state = RespondToMarkerState(self.functionality_wrapper, window_title)

        print("\n[Autonomous Mode] Starting continuous operation.")
        print("  - Press Ctrl+C or trigger remote A button to stop.\n")

        try:
            while True:
                fiducial_found, marker_id = scan_state.execute()

                if not fiducial_found:
                    continue

                has_arrived, arrived_marker_id = walk_state.execute()

                if not has_arrived:
                    continue

                time.sleep(1)
                respond_state.set_marker_id(arrived_marker_id)
                should_exit = respond_state.execute()

                if should_exit:
                    self.functionality_wrapper.safe_shutdown() 
                    time.sleep(1) 
                    break

        except KeyboardInterrupt:
            self.functionality_wrapper.safe_shutdown() 

        except Exception as e:
            print(f"Unhandled error: {e}")
            self.functionality_wrapper.safe_shutdown() 


    def shutdown_callback(self, _):
        cv2.destroyAllWindows()
        print("[Shutdown] Killing Program...")
        time.sleep(1)

        os.kill(os.getpid(), signal.SIGINT)




if __name__ == "__main__":
    main = Main()
    main.main_move()
