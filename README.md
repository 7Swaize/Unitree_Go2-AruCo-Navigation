# Unitree Go2 Control Framework

This repository provides an abstraction layer for controlling the Unitree Go2 quadruped robot, designed to simplify autonomous navigation and interaction tasks for students and researchers. 

The framework wraps the Unitree SDK's complexity behind a clean, state-based architecture that handles camera input, movement control, and remote controller integration. It includes a complete ArUco marker detection and navigation system as a reference implementation, demonstrating how to build autonomous behaviors using the state machine pattern. 

The code gracefully falls back to webcam simulation when the Unitree SDK is unavailable, making it easy to develop and test logic without physical hardware. Students can extend the `DogStateAbstract` base class to create custom behaviors while the framework automatically handles safety features like emergency stops and clean shutdowns.
