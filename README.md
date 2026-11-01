# Robotics course projects (TH Köln, SoSe 2026)

Code and reports from the Robotics module at TH Köln (Faculty 10, Prof. Dr. Algorri), written by Fabian Alexander Feith and Inalbek Akiev. The module builds up from wheel odometry to an autonomous robot that localises itself in a maze.

## Hardware and stack

- Raspberry Pi 4 with ROS 2 Iron
- Teensy 4.0 running micro-ROS, two Servo360 drives with wheel odometry from the feedback pins
- RPLidar A1, BNO055 IMU, USB camera (OpenCV)
- Python, C++ and Arduino C++

## Projects

| Folder | Content |
|---|---|
| `01_Uebungen_Odometrie_und_Grundlagen` | Weekly Python exercises, odometry variance evaluation, Teensy sketch with odometry and PID, first micro-ROS version |
| `02_Rennstrecke_Spurverfolgung_und_Ueberholen` | Race track: lane detection from the camera (OpenCV), lidar filter and obstacle detector (C++), Teensy sketch with obstacle avoidance, project log, report |
| `03_MCL_Waypoint_Navigation` | Monte Carlo Localization package `mcl_nav`: particle filter, navigator state machine, map configuration, offline simulation, Teensy sketch with a command protocol, report. The robot drives ten waypoints through a maze |
| `04_Place_Recognition` | Kidnapped-robot problem: place signatures from a full lidar sweep, recognition by histogram and cross-correlation, offline calibration, trained signatures for four places, report |

Reports are in the `Dokumentation` folders (PDF, German). Projects 02 to 04 were done as a team of two.

## Versions

- In `03`, `mcl_nav/` and `Teensy/` are the submitted state. `spaeterer_Stand_nach_Abgabe/navigator.py` is a later version that additionally divides the 90-degree turns at WP6 and WP7 by 1.03 (Teensy angle scaling).
- In `02`, `fruehere_Staende/` holds earlier versions of `lidar_logger` and `camera_publisher`.
- `04/signatures/` holds signatures recorded on the robot, `signatures_synthetic/` ones computed offline. Both are Python pickle files, so only load files you trust.

## Known gaps

- `mcl_nav` has no `setup.py`, `package.xml` or `setup.cfg` any more. The logic is complete, but the package does not build as it is.
- `pr_nav.launch.py`, mentioned in `pr_navigator.py`, was not found.
- Run data from the Pi (CSV logs, particle snapshots, reference lidar scans) is not included.
- Not included: the lecturer's templates and third-party packages (`rplidar_ros`, `micro_ros_agent`, `micro_ros_setup`).
- Paths like `/home/pi/ros2_ws/...` point to the original Raspberry Pi setup.
