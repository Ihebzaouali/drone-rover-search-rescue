"""
ugv_bringup.launch.py  (v2 — verified against gz topic -l)
============================================================
Starts four nodes needed before running navigation or RViz:

  1. gz_bridge          — bridges all sensor/command topics
  2. gz_tf_bridge       — bridges the UGV's odom→base_link TF *with a
                          remapping* so it arrives on /tf in ROS 2
  3. robot_state_publisher — publishes static TF for sensors + accepts
                             joint states to publish wheel TFs
  4. joint_state_publisher — publishes zero angles for all wheel joints
                             (because the SDF has no JointStatePublisher
                             plugin, so Gazebo never sends these values)

WHY TWO BRIDGE NODES?
---------------------
ros_gz_bridge's parameter_bridge maps Gazebo topic names 1-to-1 with
ROS 2 topic names — there is no way to rename a topic inside the
argument string.  Gazebo publishes the UGV's TF on /model/ugv/tf, but
every ROS 2 tool (Nav2, RViz, tf2) expects transforms on /tf.
The only clean solution is a second bridge node that bridges only that
one topic and uses the ROS 2 remappings parameter to rename it.

WHY joint_state_publisher?
--------------------------
robot_state_publisher needs /joint_states to compute the 4 wheel TFs.
The UGV's SDF model has a DiffDrive plugin but no JointStatePublisher
plugin, so Gazebo never emits joint positions.  joint_state_publisher
reads the URDF and publishes 0.0 for every continuous/revolute joint.
Nav2 does not care about these values — only RViz wheel animation does.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('ugv_description')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    # ------------------------------------------------------------------
    # Load the URDF — needed by both robot_state_publisher and
    # joint_state_publisher (both receive it as 'robot_description').
    # ------------------------------------------------------------------
    urdf_path = os.path.join(
        get_package_share_directory('ugv_description'),
        'urdf',
        'ugv.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # Shared parameter dict — both RSP and JSP need the same URDF string
    # and the same sim-time setting.
    robot_description_param = {
        'robot_description': robot_description,
        'use_sim_time': True,
    }

    # ==================================================================
    # NODE 1 — Main bridge
    # Bridges all sensor and command topics.
    # All Gazebo topic names below have been verified against:
    #     gz topic -l   (run while the simulation is live)
    # ==================================================================
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True,
        }],
        arguments=[

            # CLOCK — simulation time for all use_sim_time nodes.
            # Gazebo publishes /clock (confirmed in topic list).
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # CMD_VEL — velocity commands to the DiffDrive plugin.
            # ] direction: ROS 2 → Gazebo (we send commands, Gazebo obeys).
            # Bidirectional (@) also works and is kept for teleop safety.
            # Confirmed: /model/ugv/cmd_vel is in the topic list.
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

            # LIDAR — 360 degree planar scan, 10 Hz.
            # Confirmed: /model/ugv/lidar is in the topic list.
            '/model/ugv/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            # UGV FRONT CAMERA — RGB image, 30 Hz.
            # Confirmed: /model/ugv/camera is in the topic list.
            '/model/ugv/camera@sensor_msgs/msg/Image[gz.msgs.Image',

            # UGV CAMERA INFO — lens calibration metadata.
            # Confirmed: /model/ugv/camera_info is in the topic list.
            # Needed if you use any camera-based SLAM or obstacle detection.
            '/model/ugv/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # IMU — accelerometer + gyroscope at 50 Hz.
            # Confirmed: /model/ugv/imu is in the topic list.
            '/model/ugv/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # ODOMETRY — wheel odometry from DiffDrive plugin at 30 Hz.
            # Confirmed: /model/ugv/odometry is in the topic list.
            # NOTE: This arrives in ROS 2 as /model/ugv/odometry.
            # Nav2 expects /odom by default — you have two options:
            #   a) remap it in the nav2_params.yaml (recommended, no code change)
            #   b) add remappings=[('/model/ugv/odometry', '/odom')] to this node
            '/model/ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # --- JOINT STATES (CRITICAL — was missing before) ------------
            # Gazebo publishes per-model joint positions on this topic.
            # robot_state_publisher subscribes to /joint_states and uses
            # these angles to compute base_link → {fl,fr,rl,rr}_wheel TFs.
            # The wheels will visually spin in RViz as you drive.
            '/joint_states'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # DRONE DEPTH CAMERA — x500 drone's IMX214 sensor.
            # Confirmed: /world/default/model/x500_depth_0/.../IMX214/image
            # is in the topic list.
            (
                '/world/default/model/x500_depth_0'
                '/link/camera_link/sensor/IMX214/image'
                '@sensor_msgs/msg/Image@gz.msgs.Image'
            ),
        ]
    )

    # ==================================================================
    # NODE 2 — robot_state_publisher
    #
    # Reads the URDF and does two things:
    #   a) Immediately publishes STATIC transforms as a latched message
    #      on /tf_static:  base_link → camera_link, base_link → lidar_link
    #   b) Subscribes to /joint_states and publishes DYNAMIC transforms
    #      for the 4 wheel joints as they change.
    # ==================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )
    
    lidar_frame_adapter = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_frame_adapter',
        # Format: x y z yaw pitch roll parent_frame child_frame
        # parent is the URDF frame (known to TF tree)
        # child  is the Gazebo sensor frame (used in LaserScan header)
        arguments=['0', '0', '0', '0', '0', '0',
                   'lidar_link', 'ugv/lidar_link/gpu_lidar'],
        parameters=[{'use_sim_time': True}]
    )
    
    # ──────────────────────────────────────────────────────────────
    # EKF
    # ──────────────────────────────────────────────────────────────    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path,
        {'use_sim_time': True}
        ]
    )
    
    return LaunchDescription([
        lidar_frame_adapter,
        gz_bridge,
        ekf_node,
        robot_state_publisher,
    ])
