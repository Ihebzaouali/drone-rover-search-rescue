"""
mission.launch.py
=================
Launches the full drone-UGV mission pipeline:
  - person_locator_node  (drone side: detection → world position)
  - robot_mission_node   (robot side: world position → Nav2 goal)

Run AFTER:
  Terminal 1: MicroXRCE-DDS agent
  Terminal 2: PX4 + Gazebo
  Terminal 3: ugv_bringup.launch.py  (bridges + RSP)
  Terminal 4: Nav2 + SLAM / AMCL
  Terminal 5: THIS launch file

Usage:
  ros2 launch drone_ugv_mission mission.launch.py

Optional arguments:
  detection_topic:=/your/detection/topic
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── drone_advanced_vision ─────────────────────────────────────────────────
    # Runs YOLO + OpenCV detection on the drone camera.
    # Publishes /drone/person_detected (Bool) when a person is visible.
    # Publishes /drone/vision_avancee_debug (Image) for rqt_image_view.
    drone_vision = Node(
        package='drone_ugv_mission',
        executable='drone_vision',
        name='drone_advanced_vision',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yolo_model':         'yolov8n.pt',
            'camera_topic':       '/world/default/model/x500_depth_0'
                                  '/link/camera_link/sensor/IMX214/image',
            'ai_conf_threshold':  0.25,
        }]
    )

    # ── person_locator_node ───────────────────────────────────────────────────
    # Listens to /drone/person_detected (from drone_advanced_vision above)
    # + drone PX4 position → publishes PoseStamped on /detected_person_pose.
    person_locator = Node(
        package='drone_ugv_mission',
        executable='person_locator',
        name='person_locator_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ── robot_mission_node ────────────────────────────────────────────────────
    # Receives /detected_person_pose → sends NavigateToPose goal to Nav2.
    robot_mission = Node(
        package='drone_ugv_mission',
        executable='robot_mission',
        name='robot_mission_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        drone_vision,
        person_locator,
        robot_mission,
    ])
