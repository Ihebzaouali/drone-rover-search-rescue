# Drone–Rover Search and Rescue

A cooperative drone and ground robot team for search and rescue in a forest. The drone flies over the area and detects a lost person, then shares her position. The ground robot sets a Nav2 goal near that position and drives there while avoiding obstacles.

Everything runs in simulation: PX4 SITL and Gazebo for the drone, ROS 2 and Nav2 for the ground robot.

<img width="1786" height="1003" alt="image" src="https://github.com/user-attachments/assets/2ea39910-a454-4f4a-a337-9da3c583acb4" />

## Scenario

1. The drone and the ground robot are placed in a forest.
2. The drone searches the area and detects the lost person with its camera.
3. The drone publishes its position.
4. The ground robot sets a Nav2 goal near that position and navigates to it, avoiding obstacles.

**Status:** the direction drone → rover is implemented. The opposite direction (the rover finds her and the drone flies to her) is not implemented yet.

## Features

- **Drone:** PX4 SITL with a depth camera, controlled through MAVSDK, with vision-based person detection (YOLOv8)
- **Ground robot:** custom URDF model, SLAM mapping (SLAM Toolbox), localization and Nav2 navigation
- **Cooperation:** the drone position is shared with the ground robot, which turns it into a Nav2 goal automatically
- **Visualization:** RViz views of the navigation and the drone camera feed

## Architecture

```
Drone (PX4 + Gazebo)                       Ground robot (ROS 2 + Nav2)
  camera + YOLOv8  →  person found
          ↓
  position published  ──────────────→  mission node
                                              ↓
                                  Nav2 goal near the position
                                              ↓
                              navigation with obstacle avoidance
```

## Stack

ROS 2 Humble, Nav2, SLAM Toolbox, PX4 SITL, Gazebo Harmonic, Micro XRCE-DDS, MAVSDK, YOLOv8 (Ultralytics), RViz

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- A Python virtual environment for PX4, MAVSDK and YOLO (created below)

