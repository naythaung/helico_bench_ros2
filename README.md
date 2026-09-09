# Helico Bench ROS 2

ROS 2 software for the Helico automated balloon characterisation bench.

## System overview

The system coordinates:

- Meca500 robotic arm
- MoveIt 2 motion planning
- Laser and force sensing via ESP32
- Helico actuator measurements via ESP32
- Experiment sequencing and data logging
- Operator GUI

## Current development state

The current system uses:

- ROS 2 Lyrical
- Ubuntu 26.04
- MoveIt 2
- RViz
- Mock robot execution

Existing Meca500 functionality is being migrated from the original
`ros2_meca_ws` workspace.

## Planned architecture

GUI
|
Experiment Manager
|
+-- Meca500 / MoveIt
+-- Sensor ESP32
+-- Helico ESP32
+-- Data Logger

## Development strategy

1. Reproduce current Meca500 simulation
2. Add operator GUI
3. Add mock sensor interfaces
4. Add mock Helico actuator interface
5. Integrate physical ESP32 devices
6. Integrate physical Meca500
