import time
import cv2

from aruko_helpers import * #

from enum import Enum

from unitree_control.aruko.unitree_control_core_v2 import UnitreeGo2Controller
from unitree_control.states.dog_state_abstract import DogStateAbstract


class MarkerMappings(Enum):
    """Mapping of marker IDs to commands."""
    STOP = 0
    RIGHT_90_DEGREES = 1
    LEFT_90_DEGREES = 2
    ROTATE_180_DEGREES = 3


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
            code, image = self.unitree_controller.video.get_image()
            if code != 0 or image is None:
                continue

            corners, ids, _ = get_aruko_marker(image)
            marker_id, bounds, _, _, _ = extract_data_from_marker(image, corners, ids)
            fiducial_found = (marker_id != -1)

            self.update_visuals(image, fiducial_found, marker_id, bounds, current_angle, sweeps_done)
            if cv2.waitKey(1) & 0xFF == 27:
                break

            self.unitree_controller.movement.rotate(direction * self.search_delta)
            current_angle += direction * self.search_delta
            next_angle = current_angle + direction * self.search_delta

            # Check if next step would exceed the sweep limit
            if abs(next_angle) >= sweep_limit:
                next_angle = direction * sweep_limit
                sweeps_done += 1
                direction *= -1  # Reverse sweep direction
                sweep_limit = self.search_range
                
        self.unitree_controller.movement.stop()
        
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
    
    def __init__(self, functionality_wrapper, window_title, forward_step_amount=2, rotate_step_amount=1):
        super().__init__(functionality_wrapper)

        self.window_title = window_title
        self.foward_step_amount = forward_step_amount
        self.rotate_step_amount = rotate_step_amount

    def execute(self):
        """Execute walking behavior."""
        arrived = False
        marker_id = -1

        h_offset_threshold = 50 # i just played around with this number; in the future we would need a system to calculate this
        fiducial_area_threshold = 90000 # i just played around with this number; in the future we would need a system to calculate this

        print("[Walk] Starting approach to marker")

        while not arrived:
            code, image = self.unitree_controller.video.get_image()
            if code != 0 or image is None:
                continue

            corners, ids, _ = get_aruko_marker(image)
            marker_id, bounds, fiducial_center, h_offset, fiducial_area = extract_data_from_marker(image, corners, ids)
            
            is_centered = self.correct_alignment_to_marker(
                marker_id, h_offset, h_offset_threshold, self.rotate_step_amount
            )

            if is_centered:
                arrived = self.approach_marker(
                    fiducial_area, fiducial_area_threshold, self.foward_step_amount
                )

            self.update_visuals(image, marker_id, h_offset, fiducial_area, is_centered)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        if arrived:
            print(f"[Walk] Arrived at marker {marker_id}")

        self.unitree_controller.movement.stop()
        return arrived, marker_id
    

    def correct_alignment_to_marker(self, marker_id, h_offset, h_offset_threshold, rotate_step):
        """Align the dog to center on the marker."""
        if marker_id == -1:
            return False
        
        if abs(h_offset) <= h_offset_threshold:
            return True
        elif h_offset < -h_offset_threshold:
            self.unitree_controller.movement.rotate(-rotate_step)
        elif h_offset > h_offset_threshold:
            self.unitree_controller.movement.rotate(rotate_step)

        return False


    def approach_marker(self, fiducial_area, fiducial_area_threshold, base_forward_step):
        """Move forward toward the marker."""
        if fiducial_area >= fiducial_area_threshold:
            self.unitree_controller.movement.stop()
            return True
        

        slowdown_start = fiducial_area_threshold * 60 # starts to slow down at 60% of threshold reached

        if fiducial_area >= slowdown_start:
            # logarithmic slowdown scale I just took from someone online
            progress = min(1.0, max(0.0,
                (fiducial_area - slowdown_start) / (fiducial_area_threshold - slowdown_start)
            ))

            scale = max(0.1, math.log1p((1 - progress) * 4) / math.log1p(4))
            step = base_forward_step * scale
        else:
            step = base_forward_step

        self.unitree_controller.movement.move(step)
        return False
        

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
    
        if self.marker_id == MarkerMappings.STOP.value:
            print("[Respond] STOP command received")
            self.unitree_controller.movement.stop()
            time.sleep(0.5)
            self.unitree_controller.movement.stand_down()
            return True
        
        elif self.marker_id in (MarkerMappings.LEFT_90_DEGREES.value, MarkerMappings.RIGHT_90_DEGREES.value):
            rotation_amount = 160 # this is 90 degrees somehow
            step_amount = 1

            if self.marker_id == MarkerMappings.RIGHT_90_DEGREES.value:
                step_amount = step_amount * 1
            else:
                step_amount = step_amount * -1

            for _ in range(rotation_amount):
                self.unitree_controller.movement.rotate(step_amount)
                time.sleep(0.02)  # <-- Allow movement to persist (tune as needed)

        
            self.unitree_controller.movement.stop()
            time.sleep(1)

        elif self.marker_id == MarkerMappings.ROTATE_180_DEGREES.value:
            rotation_amount = 320
            step_amount = 1


            for _ in range(rotation_amount):
                self.unitree_controller.movement.rotate(step_amount)
                time.sleep(0.02) # <-- Allow movement to persist (tune as needed)
            
        return False
        
    def update_visuals(self):
        code, image = self.unitree_controller.video.get_image()
        if code != 0 or image is None:
            return
        
        cv2.imshow(self.window_title, image)



class Main:
    def __init__(self):
        self.unitree_controller = UnitreeGo2Controller(use_sdk=True)
        self.unitree_controller.register_cleanup_callback(self.shutdown_callback)


    def main_move(self):
        input("Press Enter to start autonomous movement...")

        window_title = "Aruko Detection"
        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)

        search_range = 70
        search_delta = 1.3
        max_sweeps = 3

        scan_state = ScanForMarkerState(self.unitree_controller, window_title, search_range, search_delta, max_sweeps)
        walk_state = WalkToMarkerState(self.unitree_controller, window_title)
        respond_state = RespondToMarkerState(self.unitree_controller, window_title)

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
                    break # maybe time.sleep(1) before this?

        except KeyboardInterrupt:
            print(f"Keyboard Interrupt detected")
        except Exception as e:
            print(f"Unhandled error: {e}")
        finally:
            self.unitree_controller.safe_shutdown()


    def shutdown_callback(self):
        cv2.destroyAllWindows()
        time.sleep(1)


if __name__ == "__main__":
    main = Main()
    main.main_move()
