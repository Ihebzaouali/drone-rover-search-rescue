#!/usr/bin/env python3
"""
person_locator_node.py
======================
Runs alongside the drone's detection code.

What it does:
  1. Subscribes to your existing detection topic (publishes True when
     the wheelchair person is detected in the current frame).
  2. Subscribes to the drone's PX4 local position (NED frame).
  3. When a detection fires, it estimates the person's position in the
     ROS 2 map frame (ENU) and publishes it as a PoseStamped on
     /detected_person_pose.

Coordinate frame logic:
  PX4 local position is in NED (North-East-Down).
  ROS 2 / Nav2 / your map use ENU (East-North-Up).
  Conversion:  x_enu =  y_ned
               y_enu =  x_ned
               z_enu = -z_ned

Person position estimation:
  We assume the person is on the ground (z = 0 in map frame).
  The x,y comes from the drone's current horizontal position in ENU —
  i.e. the person is approximately directly below the drone when
  detection fires.  This is a reasonable assumption for a drone that
  is hovering and pointing its camera downward.

  If your detection code also provides a pixel + depth estimate, you
  can replace the simple "below the drone" estimate with a proper
  3D back-projection using the camera intrinsics and the drone's
  attitude.  The publishing logic at the bottom stays identical.

HOW TO CONNECT YOUR DETECTION CODE:
  Your detection node should publish std_msgs/msg/Bool on the topic
  defined by DETECTION_TOPIC below.  Publish True when the wheelchair
  person appears in the frame.

  If your detection node publishes a different message type (e.g.
  vision_msgs/Detection2DArray), change the subscriber type and
  callback accordingly — only the condition that sets
  self.detection_active needs to change.

USAGE:
  ros2 run your_package person_locator_node
  # or add to your launch file
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition   # from the px4_ros_com package


# ── Configuration ────────────────────────────────────────────────────────────

# Topic your detection code publishes on.
# Change this to match whatever your detection node uses.
DETECTION_TOPIC = '/drone/person_detected'

# Topic this node publishes the person's world position on.
# The robot_mission_node subscribes to this exact topic.
OUTPUT_TOPIC = '/detected_person_pose'

# Minimum time between consecutive goal publications (seconds).
# Prevents the robot from being spammed if detection fires every frame.
PUBLISH_COOLDOWN_SEC = 3.0

# ─────────────────────────────────────────────────────────────────────────────


class PersonLocatorNode(Node):

    def __init__(self):
        super().__init__('person_locator_node')

        self.get_logger().info('Person locator node starting...')

        # Stores the latest drone position in NED frame
        self._drone_ned_x = 0.0   # North
        self._drone_ned_y = 0.0   # East
        self._drone_ned_z = 0.0   # Down (negative = above ground)

        # Tracks whether we are currently seeing a detection
        self._detection_active = False

        # Timestamp of last published pose (to enforce cooldown)
        self._last_publish_time = self.get_clock().now()

        # QoS profile that matches PX4's publisher settings
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ── Subscribers ──────────────────────────────────────────────────────

        # Drone position from PX4 (NED local frame)
        self._pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._position_callback,
            px4_qos
        )

        # Detection signal from your existing detection code
        self._det_sub = self.create_subscription(
            Bool,
            DETECTION_TOPIC,
            self._detection_callback,
            10
        )

        # ── Publisher ────────────────────────────────────────────────────────

        # PoseStamped in map frame — this is what Nav2 expects for a goal
        self._pose_pub = self.create_publisher(
            PoseStamped,
            OUTPUT_TOPIC,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,  # latch: robot gets it even if it subscribes late
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.get_logger().info(
            f'Listening for detections on: {DETECTION_TOPIC}\n'
            f'Will publish person pose to:  {OUTPUT_TOPIC}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _position_callback(self, msg: VehicleLocalPosition):
        """
        Store the latest drone position in NED.
        This runs at ~50 Hz so it is always fresh when a detection fires.
        """
        self._drone_ned_x = msg.x   # North
        self._drone_ned_y = msg.y   # East
        self._drone_ned_z = msg.z   # Down

    def _detection_callback(self, msg: Bool):
        """
        Called every time your detection node publishes.
        When True is received, compute and publish the person's position.
        """
        if not msg.data:
            # Detection lost — nothing to do
            self._detection_active = False
            return

        # Enforce cooldown so we don't spam Nav2 goals
        now = self.get_clock().now()
        elapsed = (now - self._last_publish_time).nanoseconds / 1e9
        if elapsed < PUBLISH_COOLDOWN_SEC:
            return

        self._detection_active = True
        self._last_publish_time = now

        # ── NED → ENU coordinate conversion ──────────────────────────────────
        #
        #   PX4 local frame (NED):          ROS 2 / Nav2 (ENU):
        #   x = North                       x = East
        #   y = East                        y = North
        #   z = Down (negative above gnd)   z = Up
        #
        #   Therefore:
        #   x_enu =  y_ned  (East  = East)
        #   y_enu =  x_ned  (North = North)
        #   z_enu = -z_ned  (Up    = -Down)
        #
        person_x_enu = self._drone_ned_y   # East  component
        person_y_enu = self._drone_ned_x   # North component
        person_z_enu = 0.0                 # Person is on the ground

        # ── Build and publish the PoseStamped ────────────────────────────────
        pose_msg = PoseStamped()
        pose_msg.header.stamp    = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'   # must match Nav2's global frame

        pose_msg.pose.position.x = person_x_enu
        pose_msg.pose.position.y = person_y_enu
        pose_msg.pose.position.z = person_z_enu

        # Orientation: facing forward (no rotation from map origin).
        # You can compute a meaningful heading here if needed, but Nav2
        # only strictly needs the position for NavigateToPose.
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0

        self._pose_pub.publish(pose_msg)

        self.get_logger().info(
            f'Person detected! Publishing position to robot:\n'
            f'  ENU  x={person_x_enu:.2f} m  y={person_y_enu:.2f} m\n'
            f'  (from drone NED: N={self._drone_ned_x:.2f}  '
            f'E={self._drone_ned_y:.2f}  D={self._drone_ned_z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PersonLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
