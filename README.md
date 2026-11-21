# Drone–Rover Search and Rescue

A cooperative drone and ground robot team for search and rescue in a cluttered outdoor area, simulated here by an obstacle course (walls, blocks, a tower and an arch). The drone flies over the area and detects a lost person, then shares her position. The ground robot sends a Nav2 goal at that position and drives there while avoiding obstacles.

Everything runs in simulation: PX4 SITL and Gazebo for the drone, ROS 2 and Nav2 for the ground robot.

<img width="1786" height="1003" alt="image" src="https://github.com/user-attachments/assets/2ea39910-a454-4f4a-a337-9da3c583acb4" />

## Scenario

1. The drone and the ground robot are placed in the test area.
2. The drone flies over the area (flown with the keyboard) and detects the lost person with its camera.
3. The drone publishes its position.
4. The ground robot sends a Nav2 goal at that position and navigates to it, avoiding obstacles.

**Status:** the direction drone → rover is implemented. The opposite direction (the rover finds her and the drone flies to her) is not implemented yet.

## Features

- **Drone:** PX4 SITL with a depth camera, flown with keyboard controls through MAVSDK, with vision-based person detection (YOLOv8)
- **Ground robot:** custom URDF model, SLAM mapping (SLAM Toolbox), localization (AMCL) and Nav2 navigation
- **Cooperation:** the drone position at the moment of detection is shared with the ground robot, which turns it into a Nav2 goal automatically
- **Visualization:** RViz views of the navigation and the drone camera feed with detection boxes

## Architecture

```
Drone (PX4 + Gazebo)                          Ground robot (ROS 2 + Nav2)

  camera ──► drone_vision (YOLOv8)
                 │  /drone/person_detected (Bool)
                 ▼
  PX4 position ► person_locator ──────────►  robot_mission
  (/fmu/out/vehicle_local_position_v1)        /detected_person_pose        │
                                              (PoseStamped, frame "map")   ▼
                                                                  NavigateToPose goal (Nav2)
                                                                           │
                                                                           ▼
                                                            navigation with obstacle avoidance
```

### How the drone position becomes a Nav2 goal

1. `drone_vision` runs YOLOv8 on the drone camera and publishes `True` on `/drone/person_detected` for every frame in which a `person` is detected (`False` otherwise).
2. `person_locator` reads the drone position from PX4 (local frame, NED). On a detection it converts the position to ENU (`x_enu = y_ned`, `y_enu = x_ned`) and publishes it as a `PoseStamped` in the `map` frame on `/detected_person_pose`, at most once every 3 seconds.
3. The person is assumed to be on the ground directly below the drone, so the published position is the drone's horizontal position when the detection fires. The drone should therefore be hovering above the person with the camera pointing down.
4. `robot_mission` sends this pose to Nav2 as a `NavigateToPose` goal. While a goal is active, new detections are ignored.

> **Frame offset:** PX4's local frame starts at the drone's spawn point, while the Nav2 `map` frame starts at the rover's start pose. In the provided world the drone spawns at `(0, 0)` and the rover at `(2, 0)`, so `person_locator` subtracts this 2 m offset (`DRONE_SPAWN_*` and `UGV_SPAWN_*` constants in `person_locator_node.py`). Update these constants if you move the spawn points. Both robots are assumed to start with yaw 0.

## Stack

ROS 2 Humble, Nav2, SLAM Toolbox, PX4 SITL, Gazebo Harmonic, Micro XRCE-DDS, MAVSDK, YOLOv8 (Ultralytics), RViz

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- A Python virtual environment for PX4, MAVSDK and YOLO (created below)

## Installation

### 1. Python virtual environment

```bash
sudo apt install python3.10-venv
python3 -m venv ~/px4-venv
source ~/px4-venv/bin/activate
```

### 2. ROS 2 Humble

Follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), then:

```bash
sudo apt install ros-humble-desktop ros-dev-tools
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. PX4-Autopilot

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
```

### 4. Micro XRCE-DDS Agent

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

### 5. ROS 2 packages (Gazebo bridge, Nav2, SLAM Toolbox, EKF, image tools)

```bash
sudo apt install ros-humble-ros-gzharmonic-bridge \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-cv-bridge \
  ros-humble-rqt-image-view
```

### 6. Python dependencies

**Inside the virtual environment** (PX4 build and drone keyboard control):

```bash
source ~/px4-venv/bin/activate
pip install -U pyros-genmsg setuptools
pip install empy==3.3.4 catkin_pkg lark
pip install mavsdk aioconsole pygame numpy opencv-python ultralytics
```

`empy` must stay on version 3.3.4, otherwise the ROS 2 build fails.

**For the system Python** (the mission launch runs outside the virtual environment because it needs ROS's `cv_bridge`):

```bash
deactivate
pip3 install ultralytics "numpy<2"
```

ROS 2 Humble's `cv_bridge` is built against NumPy 1.x, so keep NumPy below 2. The `yolov8n.pt` weights are downloaded automatically by Ultralytics on the first run.

### 7. Build the ROS 2 workspace

```bash
git clone https://github.com/Ihebzaouali/drone-rover-search-rescue.git ~/drone-rover-search-rescue
cp -r ~/drone-rover-search-rescue/ws_sensor_combined ~/ws_sensor_combined
cd ~/ws_sensor_combined/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
cd ..
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Use the `px4_msgs` branch that matches your PX4-Autopilot version. `person_locator` subscribes to `/fmu/out/vehicle_local_position_v1`: if your PX4 version publishes the topic under another name (for example without the `_v1` suffix), update it in `person_locator_node.py`.

### 8. Gazebo models and drone with depth camera

This project uses the simulation setup from [PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8) (the drone model with a depth camera and the Gazebo models).

```bash
cd ~
git clone https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8.git
mkdir -p ~/.gz/models
cp -r PX4-ROS2-Gazebo-YOLOv8/models/* ~/.gz/models/
echo "export GZ_SIM_RESOURCE_PATH=~/.gz/models" >> ~/.bashrc
source ~/.bashrc
```

Then replace the content of `~/PX4-Autopilot/Tools/simulation/gz/models/x500_depth/model.sdf` with:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<sdf version='1.9'>
  <model name='x500_depth'>
    <include merge='true'>
      <uri>x500</uri>
    </include>
    <include merge='true'>
      <uri>model://OakD-Lite</uri>
      <pose>.2 0 0.1 0 0 0</pose>
    </include>
    <joint name="CameraJoint" type="fixed">
      <parent>base_link</parent>
      <child>OakD-Lite/base_link</child>
      <pose relative_to="base_link">0 0 0 0 0 0</pose>
    </joint>
  </model>
</sdf>
```

### 9. Test world

Put `worlds/default.sdf` from this repository under `~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf`:

```bash
cp ~/drone-rover-search-rescue/worlds/default.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
```

### Optional: QGroundControl

QGroundControl is not needed to run this project. Install it only if you want a ground-station view of the simulated drone: download `QGroundControl.AppImage` from the official QGroundControl website, run `chmod +x QGroundControl.AppImage`, then start it with `./QGroundControl.AppImage` after the PX4 simulation is running.

## Running the simulation

Open one terminal per step. In terminals 1 to 6 and 9, run first:

```bash
source ~/px4-venv/bin/activate
source ~/ws_sensor_combined/install/setup.bash
```

After changing anything inside the workspace, rebuild it:

```bash
cd ~/ws_sensor_combined
colcon build
source install/setup.bash
```

**Terminal 1: Micro XRCE-DDS Agent**
```bash
cd ~/Micro-XRCE-DDS-Agent
MicroXRCEAgent udp4 -p 8888
```

**Terminal 2: PX4 SITL with Gazebo and the drone**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4002 PX4_GZ_MODEL=x500_depth make px4_sitl gz_x500_depth
```

**Terminal 3: ground robot bringup**
```bash
ros2 launch ugv_description ugv_bringup.launch.py
```

This starts the Gazebo ↔ ROS 2 bridges (clock, `cmd_vel`, lidar, camera, IMU, odometry, joint states **and the drone camera**), the EKF (wheel odometry + IMU → `odom` → `base_link` TF), `robot_state_publisher` and a static transform for the lidar frame.

To view the robot model in RViz (optional):
```bash
ros2 launch ugv_description display.launch.py
```

### Mapping (only once, skip it if you use the provided map)

**Terminal 4: SLAM**
```bash
ros2 launch URDFF_navigation slam.launch.py use_sim_time:=true
```

**Terminal 5: drive the robot around to build the map**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

When the map is complete, save it:
```bash
ros2 run nav2_map_server map_saver_cli -f $HOME/ws_sensor_combined/src/URDFF_navigation/maps/map
```

### Navigation (with a saved map)

**Terminal 4: Nav2**
```bash
ros2 launch URDFF_navigation navigation.launch.py use_sim_time:=true
```

**Terminal 5: localization**
```bash
ros2 launch URDFF_navigation localization.launch.py use_sim_time:=true \
  map:=$HOME/ws_sensor_combined/src/URDFF_navigation/maps/map.yaml
```

**Terminal 6: navigation visualization in RViz**
```bash
ros2 launch URDFF_viz view_navigation.launch.py use_sim_time:=true
```

In RViz, set the robot's initial pose with **2D Pose Estimate**. AMCL has no initial pose configured, so Nav2 will not accept goals until it is localized.

### Mission

**Terminal 7: drone vision, person locator and robot mission** (no virtual environment)
```bash
cd ~/ws_sensor_combined
source install/setup.bash
ros2 launch drone_ugv_mission mission.launch.py
```

This starts three nodes: `drone_vision` (YOLOv8 on the drone camera), `person_locator` (drone position → goal pose) and `robot_mission` (goal pose → Nav2).

**Terminal 8: drone camera view with detections** (optional, no virtual environment)
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Select `/drone/vision_avancee_debug` to see the camera feed with the YOLO boxes (and colour-based OpenCV boxes for the blue block, yellow cylinder, rover and tall obstacle).

**Terminal 9: drone control (keyboard)**
```bash
python3 ~/drone-rover-search-rescue/scripts/control_drone.py
```

`control_drone.py` opens a small pygame dashboard that flies the simulated drone with the keyboard through MAVSDK. It connects to the PX4 simulation on UDP port 14540, arms the drone and switches to offboard mode, then sends smoothed velocity commands. The drone does not take off by itself: press `T`, then `W` to climb.

| Key | Action |
|---|---|
| `T` | Arm the drone and start offboard mode |
| `W` / `X` | Climb / descend |
| `Z` / `S` | Forward / backward |
| `Q` / `D` | Left / right |
| `A` / `E` | Rotate left / right |
| `L` | Land |

The keys follow an AZERTY keyboard layout. Speed limits are 2 m/s horizontally, 1.2 m/s vertically and 45°/s in yaw. The dashboard shows the drone status, the current velocities and whether offboard mode is active. The pygame window must have focus to receive the key presses.

Fly over the person and hover above her with the camera pointing down. When `drone_vision` detects her, `person_locator` publishes the goal and the rover starts navigating; the `robot_mission` terminal logs the goal, the remaining distance and the result.

### Main topics

| Topic | Type | Published by → used by |
|---|---|---|
| `/drone/person_detected` | `std_msgs/Bool` | `drone_vision` → `person_locator` |
| `/drone/vision_avancee_debug` | `sensor_msgs/Image` | `drone_vision` → `rqt_image_view` |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | PX4 → `person_locator` |
| `/detected_person_pose` | `geometry_msgs/PoseStamped` (frame `map`, latched) | `person_locator` → `robot_mission` |
| `navigate_to_pose` | Nav2 action | `robot_mission` → Nav2 |
| `/model/ugv/lidar`, `/model/ugv/odometry`, `/model/ugv/imu` | LaserScan, Odometry, Imu | Gazebo → SLAM / AMCL / Nav2 / EKF |

## Project structure

- `ugv_description`: ground robot URDF (4 wheels, camera, lidar), EKF configuration, `ugv_bringup.launch.py` (bridges, EKF, robot state publisher) and `display.launch.py` (RViz view of the model)
- `URDFF_navigation`: SLAM Toolbox, AMCL localization and Nav2 configuration (DWB local planner, NavFn global planner), launch files and the saved map
- `URDFF_viz`: RViz configuration and launcher for the navigation view
- `drone_ugv_mission`: `drone_advanced_vision.py` (YOLOv8 person detection), `person_locator_node.py` (drone position → pose in the map frame) and `robot_mission_node.py` (Nav2 goal client), started by `mission.launch.py`
- `worlds/default.sdf`: Gazebo world (obstacle course, `ugv` model and lost person) to copy into PX4
- `scripts/control_drone.py`: keyboard dashboard to fly the simulated drone

## Troubleshooting

- **`colcon build` fails with `empy` / `em` errors:** reinstall the pinned version in the virtual environment, then clean and rebuild:
  ```bash
  pip uninstall em empy
  pip install empy==3.3.4
  cd ~/ws_sensor_combined && rm -rf build/ install/ log/ && colcon build
  ```
- **The drone does not react to the keyboard:** click the pygame dashboard so it has focus, press `T` first, and check that the Micro XRCE-DDS Agent and PX4 SITL are running.
- **The rover does not move after a detection:** check that Nav2 and AMCL are running, that the initial pose was set in RViz, and read the `robot_mission` log: a goal inside an obstacle on the costmap is rejected.

## Limitations

- Runs in simulation only, with no tests on real robots.
- Only the direction drone → rover is implemented. The rover cannot send a position to the drone yet.
- The person's position is the drone's position at the time of detection (no 3D back-projection from the image), and the frame offset is a fixed constant (see above).
- The spawn-point offset between the drone and the rover is a constant in `person_locator_node.py`, not read from Gazebo. In RViz, set the initial pose at the rover's spawn pose, which is the map origin `(0, 0)` with yaw 0.
- The goal is sent exactly at that position. Nav2 rejects it if it falls inside an obstacle on the costmap, and the rover ignores new detections while it is navigating.
- Detection is class-based (`person`, YOLOv8n, confidence 0.25), so any person in view triggers the mission.
- `drone_ugv_mission/config/mission_config.yaml` is not loaded by `mission.launch.py`: the topic names and the 3 s cooldown are constants in the node files.

## Acknowledgments and licenses

- Drone and Gazebo simulation setup: [monemati/PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), [px4_msgs](https://github.com/PX4/px4_msgs), [px4_ros_com](https://github.com/PX4/px4_ros_com), [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent), ROS 2 and Nav2
- Object detection uses [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics), which has its own license terms. Check them before reusing this project.
