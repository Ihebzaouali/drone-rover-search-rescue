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

> TODO: explain how the drone position is converted into the ground robot's map frame.

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

### 5. ROS 2 packages (Gazebo bridge, Nav2, SLAM Toolbox)

```bash
sudo apt install ros-humble-ros-gzgarden \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard ros-humble-slam-toolbox
```

> TODO: check that the Gazebo–ROS bridge package matches Gazebo Harmonic (`ros-humble-ros-gzgarden` is for Gazebo Garden).

### 6. Python dependencies (inside the virtual environment)

```bash
source ~/px4-venv/bin/activate
pip install -U pyros-genmsg setuptools
pip install empy==3.3.4 catkin_pkg lark
pip install mavsdk aioconsole pygame numpy opencv-python ultralytics
```

`empy` must stay on version 3.3.4, otherwise the ROS 2 build fails.

### 7. Build the ROS 2 workspace

```bash
mkdir -p ~/ws_sensor_combined/src
cd ~/ws_sensor_combined/src
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/Ihebzaouali/drone-rover-search-rescue.git
cp -r drone-rover-search-rescue/{ugv_description,URDFF_navigation,URDFF_viz,drone_ugv_mission} .
cd ..
source /opt/ros/humble/setup.bash
colcon build
```

Use the `px4_msgs` branch that matches your PX4-Autopilot version.

> TODO: confirm the repository layout (the four packages at the root of the repo).

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

### Optional: QGroundControl

QGroundControl is not needed to run this project. Install it only if you want a ground-station view of the simulated drone: download `QGroundControl.AppImage` from the official QGroundControl website, run `chmod +x QGroundControl.AppImage`, then start it with `./QGroundControl.AppImage` after the PX4 simulation is running.

## Running the simulation

Open one terminal per step. In terminals 1 to 6 and 9, run first:

```bash
source ~/px4-venv/bin/activate
source ~/ws_sensor_combined/install/setup.bash
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

**Terminal 3: ground robot**
```bash
ros2 launch ugv_description ugv_bringup.launch.py
```

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

### Mission

**Terminal 7: drone and ground robot mission** (no virtual environment)
```bash
cd ~/ws_sensor_combined
source install/setup.bash
ros2 launch drone_ugv_mission mission.launch.py
```

**Terminal 8: drone camera view** (optional, no virtual environment)
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
```

**Terminal 9: drone control**
```bash
cd ~/PX4-Autopilot
python3 control_drone.py
```

> TODO: say what `control_drone.py` does (for example its keyboard controls) and where the script lives in the repo.

## Project structure

- `ugv_description`: ground robot model, bringup and display launch files
- `URDFF_navigation`: SLAM, localization and Nav2 configuration, with the maps
- `URDFF_viz`: RViz configuration for the navigation view
- `drone_ugv_mission`: mission logic coordinating the drone and the ground robot

> TODO: check these descriptions and add the drone control script.

## Limitations

- Runs in simulation only, with no tests on real robots.
- Only the direction drone → rover is implemented. The rover cannot send a position to the drone yet.

## Acknowledgments and licenses

- Drone and Gazebo simulation setup: [monemati/PX4-ROS2-Gazebo-YOLOv8](https://github.com/monemati/PX4-ROS2-Gazebo-YOLOv8)
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot), [px4_msgs](https://github.com/PX4/px4_msgs), [px4_ros_com](https://github.com/PX4/px4_ros_com), [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent), ROS 2 and Nav2
- Object detection uses [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics), which has its own license terms. Check them before reusing this project.

## Author

Iheb Zaouali, [LinkedIn](https://www.linkedin.com/in/iheb-zaouali-673974352)
