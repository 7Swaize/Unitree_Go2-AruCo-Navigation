from unitree_control.core.control_modules import DogModule
from unitree_control.core.hardware_control import HardwareInterface


class MovementModule(DogModule):
    """High-level movement controls"""

    def __init__(self, hardware: HardwareInterface):
        super().__init__("Movement")
        self.hardware = hardware
        self.max_rotation = 5.0
        self.max_translation = 5.0

    def initialize(self) -> None:
        self._initialized = True

    def rotate(self, amount: float) -> None:
        """Rotate the dog (yaw)"""
        amount = max(-self.max_rotation, min(amount, self.max_rotation))
        self.hardware.rotate(amount)

    def move(self, amount_x: float = 0.0, amount_y: float = 0.0) -> None:
        """Shift the dog (forward/backward and lateral)"""
        amount_x = max(-self.max_translation, min(amount_x, self.max_translation))
        amount_y = max(-self.max_translation, min(amount_y, self.max_translation))
        self.hardware.move(amount_x, amount_y)

    def stand_up(self) -> None:
        """Stand up the dog"""
        self.hardware.stand_up()

    def stand_down(self) -> None:
        """Lay down the dog"""
        self.hardware.stand_down()

    def stop(self) -> None:
        """Stop all movement"""
        self.hardware.stop_move()

    def shutdown(self) -> None:
        self.stop()
        self._initialized = False