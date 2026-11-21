#!/usr/bin/env python3
"""
robot_mission_node.py
=====================
Runs on the robot side (alongside your Nav2 stack).

What it does:
  1. Subscribes to /detected_person_pose (published by person_locator_node).
  2. When a pose arrives, logs a confirmation message.
  3. Sends a NavigateToPose action goal to Nav2.
  4. Monitors the goal — logs when the robot arrives, or if it fails.
  5. Ignores new detections while a goal is already in progress, to
     prevent the robot from being redirected mid-journey.

State machine (simple):
  IDLE ──(pose received)──▶ NAVIGATING ──(goal reached)──▶ IDLE
                                         └─(goal failed) ──▶ IDLE

USAGE:
  ros2 run your_package robot_mission_node
  # or add to your launch file alongside Nav2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── Configuration ────────────────────────────────────────────────────────────

# Must match the OUTPUT_TOPIC in person_locator_node.py
PERSON_POSE_TOPIC = '/detected_person_pose'

# Nav2 action server name — this is the standard Nav2 default.
# Only change this if you remapped it in your nav2 launch file.
NAV2_ACTION_SERVER = 'navigate_to_pose'

# ─────────────────────────────────────────────────────────────────────────────


class RobotMissionNode(Node):

    def __init__(self):
        super().__init__('robot_mission_node')

        self.get_logger().info('Robot mission node starting...')

        # State flag — True while a Nav2 goal is active
        self._navigating = False

        # The current goal handle, used to check status
        self._goal_handle = None

        # ── Nav2 action client ───────────────────────────────────────────────
        #
        # NavigateToPose is the standard Nav2 action for sending a single
        # goal pose.  The action server is started by nav2_bt_navigator.
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            NAV2_ACTION_SERVER
        )

        # ── Subscriber ──────────────────────────────────────────────────────
        #
        # TRANSIENT_LOCAL durability means we receive the last published
        # message even if we subscribed after it was published.
        # This matters if the robot node starts after the drone detects.
        self._pose_sub = self.create_subscription(
            PoseStamped,
            PERSON_POSE_TOPIC,
            self._person_pose_callback,
            qos_profile=QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.get_logger().info(
            f'Waiting for person position on: {PERSON_POSE_TOPIC}\n'
            f'Will send Nav2 goals to:         {NAV2_ACTION_SERVER}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _person_pose_callback(self, pose_msg: PoseStamped):
        """
        Called when person_locator_node publishes a detected position.
        """
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y

        # If we're already navigating, ignore the new detection.
        # The robot finishes its current mission first.
        if self._navigating:
            self.get_logger().warn(
                f'New person position received (x={x:.2f}, y={y:.2f}) '
                f'but robot is already navigating. Ignoring.'
            )
            return

        self.get_logger().info(
            f'\n'
            f'╔══════════════════════════════════════════╗\n'
            f'║   PERSON DETECTED — POSITION RECEIVED   ║\n'
            f'╠══════════════════════════════════════════╣\n'
            f'║  x = {x:>8.3f} m  (East)              ║\n'
            f'║  y = {y:>8.3f} m  (North)             ║\n'
            f'╚══════════════════════════════════════════╝\n'
            f'Sending Nav2 goal...'
        )

        self._send_nav2_goal(pose_msg)

    def _send_nav2_goal(self, pose_msg: PoseStamped):
        """
        Sends a NavigateToPose goal to Nav2.
        Waits for the action server to be ready, then sends asynchronously.
        """

        # Wait up to 5 seconds for Nav2 to be ready.
        # If it's not ready yet (e.g. still loading the map), we warn and abort.
        self.get_logger().info('Waiting for Nav2 action server...')
        server_ready = self._nav_client.wait_for_server(timeout_sec=5.0)

        if not server_ready:
            self.get_logger().error(
                'Nav2 action server not available after 5 seconds.\n'
                'Is Nav2 running? Check that nav2_bringup is launched.'
            )
            return

        # Build the NavigateToPose goal
        goal = NavigateToPose.Goal()
        goal.pose = pose_msg   # PoseStamped directly — frame_id must be 'map'

        # Send the goal asynchronously with callbacks
        self.get_logger().info(
            f'Sending goal to Nav2: '
            f'x={pose_msg.pose.position.x:.2f}, '
            f'y={pose_msg.pose.position.y:.2f}'
        )

        self._navigating = True

        send_goal_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )

        # Add a callback for when Nav2 accepts or rejects the goal
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """
        Called when Nav2 accepts or rejects our goal.
        """
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error(
                'Nav2 REJECTED the goal.\n'
                'Possible causes:\n'
                '  - Goal position is inside an obstacle on the costmap\n'
                '  - Goal is outside the map bounds\n'
                '  - Nav2 is not in the correct state'
            )
            self._navigating = False
            return

        self.get_logger().info(
            'Nav2 ACCEPTED the goal. Robot is now navigating to the person.'
        )

        # Add a callback for when navigation finishes
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _feedback_callback(self, feedback_msg):
        """
        Called periodically by Nav2 while the robot is navigating.
        Logs estimated time remaining and current distance to goal.
        """
        feedback = feedback_msg.feedback

        # NavigateToPose feedback provides estimated_time_remaining
        # and distance_remaining
        try:
            dist = feedback.distance_remaining
            self.get_logger().info(
                f'Navigating... distance remaining: {dist:.2f} m',
                throttle_duration_sec=2.0   # log at most once per 2 seconds
            )
        except AttributeError:
            pass   # some Nav2 versions have slightly different feedback fields

    def _goal_result_callback(self, future):
        """
        Called when navigation finishes (success, failure, or cancellation).
        """
        result = future.result()
        status = result.status

        self._navigating = False
        self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                '\n'
                '╔══════════════════════════════════════════╗\n'
                '║   MISSION COMPLETE — ARRIVED AT PERSON  ║\n'
                '╚══════════════════════════════════════════╝'
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(
                'Navigation was CANCELLED. '
                'Ready to accept a new goal.'
            )
        else:
            self.get_logger().error(
                f'Navigation FAILED (status code: {status}).\n'
                'Possible causes:\n'
                '  - Path blocked by an obstacle not on the saved map\n'
                '  - Robot got stuck\n'
                '  - Planner could not find a valid path\n'
                'Ready to accept a new goal.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
