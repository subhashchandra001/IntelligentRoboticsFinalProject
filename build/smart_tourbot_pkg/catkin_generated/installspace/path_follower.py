#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
path_follower.py

Follows a nav_msgs/Path published by route_planner.py.

- Subscribes:
    /planned_path  (nav_msgs/Path)
    /odom          (nav_msgs/Odometry)

- Publishes:
    /cmd_vel_nav       (geometry_msgs/Twist)
    /nav_goal_reached  (std_msgs/Bool)  <-- NEW: signals when final waypoint reached

Behavior:
    * When a new path arrives, we FILTER OUT any poses that are
      basically at the robot's current position (within tolerance).
      This automatically removes the "start" pose, which is usually
      where the robot already is.
    * We then drive waypoint-by-waypoint until all poses are reached.
    * At each waypoint, we pause for a few seconds (configurable).
    * When the FINAL waypoint is reached, we:
        - publish nav_goal_reached = True
        - stop the robot
        - optionally pause briefly, then consider the path done.
"""

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class PathFollower(object):
    def __init__(self):
        rospy.logwarn("### PathFollower: NEW VERSION (filters start waypoint) ###")
        rospy.loginfo("PathFollower: initializing...")

        # Parameters
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.waypoint_tolerance = rospy.get_param("~waypoint_tolerance", 0.15)
        self.pause_at_waypoint = rospy.get_param("~pause_at_waypoint", 5.0)

        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.25)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.0)
        self.kp_linear = rospy.get_param("~kp_linear", 0.7)
        self.kp_angular = rospy.get_param("~kp_angular", 1.5)

        # State
        self.current_path = None      # nav_msgs/Path
        self.current_index = None     # index of current waypoint
        self.pause_until = None       # rospy.Time or None
        self.last_odom = None         # nav_msgs/Odometry
        self.active = False

        # Publishers / Subscribers
        self.cmd_pub = rospy.Publisher("cmd_vel_nav", Twist, queue_size=1)
        self.goal_reached_pub = rospy.Publisher(
            "nav_goal_reached", Bool, queue_size=1
        )

        self.path_sub = rospy.Subscriber(
            "planned_path", Path, self.path_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )

        # Control loop timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_cb)  # 10 Hz

        rospy.loginfo(
            "PathFollower: node is up (subscribing to /planned_path and /odom)."
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def path_callback(self, msg):
        """Receive a new planned path and filter out poses at the robot's current position."""
        if len(msg.poses) == 0:
            rospy.logwarn("PathFollower: received empty path; ignoring.")
            self.current_path = None
            self.current_index = None
            self.active = False
            return

        # Need an odom reading to filter by distance
        robot_pose = self.get_robot_pose_2d()
        if robot_pose is None:
            rospy.logwarn("PathFollower: no /odom yet; using path as-is.")
            filtered_poses = msg.poses
        else:
            (rx, ry, _) = robot_pose
            filtered_poses = []
            for i, ps in enumerate(msg.poses):
                gx = ps.pose.position.x
                gy = ps.pose.position.y
                dist = math.sqrt((gx - rx) ** 2 + (gy - ry) ** 2)
                if dist < self.waypoint_tolerance:
                    rospy.loginfo(
                        "PathFollower: dropping pose %d at (%.2f, %.2f) because it's too close "
                        "to robot (%.2f, %.2f) [dist=%.3f < tol=%.3f].",
                        i,
                        gx,
                        gy,
                        rx,
                        ry,
                        dist,
                        self.waypoint_tolerance,
                    )
                else:
                    filtered_poses.append(ps)

        if len(filtered_poses) == 0:
            rospy.logwarn(
                "PathFollower: all path poses were within tolerance of current position; "
                "nothing to follow."
            )
            self.current_path = None
            self.current_index = None
            self.active = False
            self.stop_robot()
            return

        # Build a new Path with filtered poses
        self.current_path = Path()
        self.current_path.header = msg.header
        self.current_path.poses = filtered_poses

        self.current_index = 0
        self.pause_until = None
        self.active = True

        rospy.loginfo(
            "PathFollower: received new path with %d poses; after filtering, %d waypoints "
            "remain. Starting from index %d.",
            len(msg.poses),
            len(filtered_poses),
            self.current_index,
        )

        # New path -> reset "goal reached" notification (publish False)
        self.goal_reached_pub.publish(Bool(data=False))

    def odom_callback(self, msg):
        self.last_odom = msg

    # ------------------------------------------------------------------
    # Helper functions
    # ------------------------------------------------------------------
    def get_robot_pose_2d(self):
        """Return (x, y, yaw) from last odom, or None if not available."""
        if self.last_odom is None:
            return None

        p = self.last_odom.pose.pose.position
        o = self.last_odom.pose.pose.orientation
        quat = [o.x, o.y, o.z, o.w]
        (_, _, yaw) = euler_from_quaternion(quat)
        return (p.x, p.y, yaw)

    def get_current_waypoint_2d(self):
        """Return (x, y) of the current waypoint, or None."""
        if (
            self.current_path is None
            or self.current_index is None
            or self.current_index >= len(self.current_path.poses)
        ):
            return None

        pose = self.current_path.poses[self.current_index]
        x = pose.pose.position.x
        y = pose.pose.position.y
        return (x, y)

    def stop_robot(self):
        """Publish zero velocity."""
        twist = Twist()
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------
    def timer_cb(self, event):
        # If no active path, publish STOP and exit
        if not self.active or self.current_path is None:
            self.stop_robot()
            return

        # Must have odom to move
        pose = self.get_robot_pose_2d()
        if pose is None:
            self.stop_robot()
            rospy.logwarn_throttle(5.0, "PathFollower: waiting for /odom...")
            return

        # If we are in a pause at a waypoint, hold still
        if self.pause_until is not None:
            if rospy.Time.now() < self.pause_until:
                self.stop_robot()
                return
            else:
                rospy.loginfo("PathFollower: pause at waypoint finished.")
                self.pause_until = None

        # Get current waypoint
        wp = self.get_current_waypoint_2d()
        if wp is None:
            # No more waypoints -> done
            rospy.loginfo("PathFollower: all waypoints completed; stopping.")
            self.active = False
            self.stop_robot()
            return

        # Compute control toward current waypoint
        (x, y, yaw) = pose
        (gx, gy) = wp
        dx = gx - x
        dy = gy - y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if waypoint is reached
        if dist < self.waypoint_tolerance:
            rospy.loginfo(
                "PathFollower: reached waypoint %d at (%.2f, %.2f).",
                self.current_index,
                gx,
                gy,
            )

            # If this was the last waypoint -> done
            if self.current_index >= len(self.current_path.poses) - 1:
                rospy.loginfo(
                    "PathFollower: final waypoint reached; will stop after pause."
                )
                # Notify TourExecutor that nav goal is reached
                self.goal_reached_pub.publish(Bool(data=True))

                # Optional: small pause while stopped at final waypoint
                self.pause_until = rospy.Time.now() + rospy.Duration(
                    self.pause_at_waypoint
                )

                # Mark path as inactive so we won't drive further
                self.active = False
                self.stop_robot()
                return

            # Otherwise: advance to next waypoint and pause briefly
            self.current_index += 1
            self.pause_until = rospy.Time.now() + rospy.Duration(
                self.pause_at_waypoint
            )
            self.stop_robot()
            return

        # Otherwise, move toward waypoint
        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - yaw)

        # Proportional controller
        linear_cmd = self.kp_linear * dist
        angular_cmd = self.kp_angular * yaw_error

        # Clamp speeds
        linear_cmd = max(
            -self.max_linear_speed, min(self.max_linear_speed, linear_cmd)
        )
        angular_cmd = max(
            -self.max_angular_speed, min(self.max_angular_speed, angular_cmd)
        )

        twist = Twist()
        twist.linear.x = linear_cmd
        twist.angular.z = angular_cmd
        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main():
    rospy.init_node("path_follower")
    node = PathFollower()
    rospy.spin()


if __name__ == "__main__":
    main()

