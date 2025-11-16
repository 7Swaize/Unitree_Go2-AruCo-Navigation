# Unitree Go2 Control Framework

A lightweight framework for controlling the Unitree Go2 quadruped. It simplifies movement, camera input, and remote controller integration using a state-based architecture.

Includes a reference ArUco marker detection and navigation system to demonstrate building autonomous behaviors. If the Unitree SDK isn’t available, it falls back to webcam simulation so you can develop and test without hardware.

Custom behaviors can be added by extending DogStateAbstract, while safety features like emergency stops and clean shutdowns are handled automatically.

Designed for students at my school.
