# ros2_diff_drive

- `arduino/`: motor + encoder firmware (UNO).
- `ros2_ws/`: ROS 2 workspace (to be added from Pi; exclude build/install/log).

## Arduino
Open `arduino/*.ino` in Arduino IDE and upload.

## ROS 2 (Pi5) — planned
Clone repo on Pi, copy only `src/` and metadata into `ros2_ws/`, then build with `colcon`.
