# Unitree Go2 ArUco Navigation

Autonomous navigation system for Unitree Go2 quadruped robot using ArUco marker detection. Robot scans for markers, walks toward them, and executes commands based on marker IDs.

## What It Does

- Scans environment for ArUco markers (sweeping head motion)
- Walks toward detected markers with real-time alignment
- Executes commands: STOP (marker 0), TURN_RIGHT (marker 1), TURN_LEFT (marker 2)
- Falls back to webcam simulation if Unitree SDK unavailable

## Core Components

**aruko_navigation_and_response.py** - State machine implementation with three states: ScanForMarkerState (sweeps to find markers), WalkToMarkerState (approaches and centers on marker), RespondToMarkerState (executes marker commands). Runs continuous loop until STOP marker encountered.

**aruko_helpers.py** - ArUco detection utilities. Extracts corner points, computes center, calculates horizontal offset from image center, estimates marker area (for distance approximation).

**unitree_control_core.py** - Hardware abstraction layer. Wraps Unitree SDK for robot control (movement, rotation, camera) with fallback to webcam. Includes callback system for emergency stop via controller A button.

## How It Works

1. **Scan**: Robot rotates in sweeping pattern (configurable range and number of sweeps) until marker detected
2. **Approach**: Aligns horizontally using offset threshold, moves forward until marker area exceeds threshold (distance heuristic)
3. **Respond**: Executes command based on marker ID. STOP sits down and exits. LEFT/RIGHT rotates ~90° then continues scanning
4. Loop repeats until STOP marker

## State Pattern

All states inherit from `DogStateAbstract` with metaclass that injects `check_shutdown()` calls into loops (allows emergency stop mid-execution). Each state implements `execute()` and handles its own visual feedback via OpenCV overlays.

## Controller Integration

`_InputHandler` parses Unitree wireless remote data (analog sticks, buttons). `_InputSignalCallbackManager` handles threshold-based triggering for analog inputs and edge detection for digital buttons. A button mapped to emergency shutdown.

## Tunable Parameters

- **Scan**: `search_range` (degrees), `search_delta` (rotation step), `max_sweeps`
- **Walk**: `h_offset_threshold` (centering tolerance in pixels), `fiducial_area_threshold` (stop distance), `forward_step`, `rotate_step`
- **Respond**: `rotation_amount` for 90° turns (empirically set to 150 steps)

## Safety

Emergency stop via remote controller A button or Ctrl+C. Calls `safe_shutdown()` which stops movement, stands down robot, and releases resources. Metaclass-injected shutdown checks ensure loops terminate cleanly.

## Fallback Mode

If Unitree SDK not detected, uses webcam (cv2.VideoCapture). Movement commands become no-ops but visualization/logic still runs for testing marker detection algorithms.
