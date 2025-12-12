#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import random
import math
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def normalize_angle(a):
    """Keep angle in [-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a


class ReactiveController:
    def __init__(self):
        # NOTE: rospy.init_node() is called in __main__, not here.
        rospy.loginfo("ReactiveController: initializing hybrid reactive controller...")

        # Publisher to robot's velocity topic (Kobuki / Gazebo)
        self.cmd_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                       Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/mobile_base/events/bumper',
                         BumperEvent, self.bumper_callback)

        # Teleop comes on a dedicated topic now
        rospy.Subscriber('/cmd_vel_teleop',
                         Twist, self.teleop_callback)

        # High-level navigation commands from planner / path_follower
        rospy.Subscriber('/cmd_vel_nav',
                         Twist, self.nav_callback)

        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # --- State ---
        self.bumped = False
        self.last_teleop_cmd = None   # None means no active teleop command
        self.last_nav_cmd = None      # None means no active nav command

        # Current pose / yaw (from odom)
        self.last_x = None
        self.last_y = None
        self.current_yaw = 0.0

        # Escape (two-phase: back -> turn-to-angle)
        self.escaping = False
        self.escape_phase = None        # "back" or "turn"
        self.escape_end_time = rospy.Time.now()
        self.escape_target_yaw = None   # target heading for 180±30°

        # Random turn (angle-based ±15° after 1 ft movement)
        self.distance_accum = 0.0
        self.turning_random = False
        self.random_target_yaw = None   # target heading for ±15°
        self.k_yaw = 2.0                # P-gain for turning to target angle
        self.w_max = 1.8                # max turn speed (rad/s)
        self.yaw_tolerance = math.radians(3.0)  # stop within ~3°

        # Thresholds (in meters)
        # front_thresh ~ "1 ft" region; side_thresh for wall margin
        self.front_thresh = 0.50
        self.side_thresh = 0.50

        rospy.loginfo("ReactiveController started (bumpers + teleop + nav + scan).")

    # ------------------- BUMPER -------------------
    def bumper_callback(self, msg):
        if msg.state == BumperEvent.PRESSED:
            rospy.logwarn("Bumper pressed! Starting escape maneuver.")
            self.start_escape()
            self.bumped = True
            # Immediate hard stop
            self.cmd_pub.publish(Twist())
        else:
            rospy.loginfo("Bumper released! Robot can move again.")
            self.bumped = False

    # ------------------- TELEOP -------------------
    def teleop_callback(self, msg):
        """Store last teleop command if active, reset if released."""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_teleop_cmd = msg
        else:
            self.last_teleop_cmd = None  # reset when keys released

    # ------------------- NAV (PLANNER) COMMANDS -------------------
    def nav_callback(self, msg):
        """Store last nav command (from planner / path_follower) if active."""
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01:
            self.last_nav_cmd = msg
        else:
            self.last_nav_cmd = None  # zero command = no active nav

    # ------------------- ODOM (distance + yaw) -------------------
    def odom_callback(self, msg):
        """Track distance traveled and update current yaw."""
        # Current position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Current yaw from quaternion
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw  # keep yaw updated for angle-based turns

        # Accumulate planar distance for random-turn trigger
        if self.last_x is not None and self.last_y is not None:
            dx = x - self.last_x
            dy = y - self.last_y
            self.distance_accum += math.sqrt(dx * dx + dy * dy)

        self.last_x = x
        self.last_y = y

        # Optional random-turn logic (currently disabled):
        # if (self.distance_accum >= 0.30 and
        #     not self.turning_random and
        #     not self.escaping and
        #     not self.bumped):
        #     self.start_random_turn()

    # ------------------- ESCAPE MANEUVER -------------------
    def start_escape(self):
        """Start a two-phase escape: reverse 0.5s + angle turn to 180±30° away."""
        self.escaping = True
        self.escape_phase = "back"
        self.escape_end_time = rospy.Time.now() + rospy.Duration.from_sec(0.5)
        self.escape_target_yaw = None

    def do_escape(self):
        """Perform current escape phase (time-based reverse, then angle-based turn)."""
        twist = Twist()
        if self.escape_phase == "back":
            twist.linear.x = -0.2
            self.cmd_pub.publish(twist)
            if rospy.Time.now() >= self.escape_end_time:
                # Switch to angle-turn phase
                self.escape_phase = "turn"
                delta = math.pi + random.uniform(-math.radians(30.0),
                                                 math.radians(30.0))
                self.escape_target_yaw = normalize_angle(self.current_yaw + delta)
        elif self.escape_phase == "turn":
            # angle-based turn using P-control to target yaw
            err = normalize_angle(self.escape_target_yaw - self.current_yaw)
            if abs(err) > self.yaw_tolerance:
                twist.angular.z = max(-self.w_max, min(self.k_yaw * err, self.w_max))
                self.cmd_pub.publish(twist)
            else:
                # Done
                self.escaping = False
                self.escape_phase = None
                self.escape_target_yaw = None
                rospy.loginfo("Escape complete. Resuming normal operation.")

    # ------------------- RANDOM TURN (±15°) -------------------
    def start_random_turn(self):
        """Set a small ±15° target relative to current yaw (angle-based)."""
        self.turning_random = True
        self.distance_accum = 0.0
        delta = random.uniform(-math.radians(15.0), math.radians(15.0))
        self.random_target_yaw = normalize_angle(self.current_yaw + delta)
        rospy.logwarn("Random turn triggered (±15°) after ~1 ft travel.")

    def do_random_turn(self):
        """Angle-based random turn to random_target_yaw."""
        twist = Twist()
        err = normalize_angle(self.random_target_yaw - self.current_yaw)
        if abs(err) > self.yaw_tolerance:
            twist.angular.z = max(-self.w_max, min(self.k_yaw * err, self.w_max))
            self.cmd_pub.publish(twist)
        else:
            self.turning_random = False
            self.random_target_yaw = None

    # ------------------- SCAN + REACTIVE LOGIC -------------------
    def scan_callback(self, msg):
        # Highest-priority behaviors first
        if self.bumped:
            # When bumper is pressed we only allow escape logic to move
            return

        # Escape in progress
        if self.escaping:
            self.do_escape()
            return

        # Optional random-turn behavior (currently disabled)
        # if self.turning_random:
        #     self.do_random_turn()
        #     return

        ranges = msg.ranges
        num_ranges = len(ranges)
        if num_ranges == 0:
            return

        # --- FRONT SECTORS (~70° each) ---
        left_sector = ranges[num_ranges // 2 + 70: num_ranges // 2 + 110]
        right_sector = ranges[num_ranges // 2 - 110: num_ranges // 2 - 70]

        # --- SIDE SECTORS (~30° each at ±90°) ---
        left_side_sector = ranges[num_ranges - 100: num_ranges - 30]   # left side
        right_side_sector = ranges[30:100]                             # right side

        def valid(vals):
            return [r for r in vals
                    if not math.isinf(r) and not math.isnan(r) and r > 0.0]

        left_min = min(valid(left_sector)) if valid(left_sector) else 10.0
        right_min = min(valid(right_sector)) if valid(right_sector) else 10.0
        left_side_min = min(valid(left_side_sector)) if valid(left_side_sector) else 10.0
        right_side_min = min(valid(right_side_sector)) if valid(right_side_sector) else 10.0

        front_thresh = self.front_thresh
        side_thresh = self.side_thresh

        # This "twist" encodes pure reactive behavior (side walls + front avoid)
        reactive_twist = Twist()

        # --- Symmetric Escape (≤ front_thresh both sides, nearly equal) ---
        if (left_min < front_thresh and right_min < front_thresh and
            abs(left_min - right_min) < 0.10):
            rospy.logwarn("Symmetric obstacle detected! Escaping (back + 180±30°).")
            self.start_escape()
            return

        # --- Side Wall Avoidance (keep margin) ---
        if left_side_min < side_thresh:
            rospy.logwarn("Too close to LEFT wall → nudging RIGHT")
            reactive_twist.angular.z = -0.6
            reactive_twist.linear.x = 0.05
        elif right_side_min < side_thresh:
            rospy.logwarn("Too close to RIGHT wall → nudging LEFT")
            reactive_twist.angular.z = 0.6
            reactive_twist.linear.x = 0.05
        # --- Asymmetric Front Avoidance (≤ front_thresh) ---
        elif left_min < front_thresh and right_min >= front_thresh:
            rospy.logwarn("Obstacle on LEFT → turning RIGHT (auto)")
            reactive_twist.angular.z = -1.8
        elif right_min < front_thresh and left_min >= front_thresh:
            rospy.logwarn("Obstacle on RIGHT → turning LEFT (auto)")
            reactive_twist.angular.z = 1.8
        else:
            # Default autonomous forward motion pattern (used ONLY if no nav/teleop).
            reactive_twist.linear.x = 0.25

        # ----------------- ARBITRATION -----------------
        # Priority: TELEOP > NAV > (STOP when idle)
        #
        # IMPORTANT CHANGE FOR FINAL PROJECT:
        #   - If no teleop and no nav, we DO NOT wander reactively.
        #   - We publish a STOP instead, so after the tour or path is done,
        #     the robot stays still.
        if self.last_teleop_cmd is not None:
            # Teleop allowed only if front is clear
            if left_min > front_thresh and right_min > front_thresh:
                self.cmd_pub.publish(self.last_teleop_cmd)
            else:
                rospy.logwarn("Teleop blocked: obstacle ≤ 1 ft! Stopping instead of crashing.")
                self.cmd_pub.publish(Twist())
        elif self.last_nav_cmd is not None:
            # Use nav command as base, but apply front safety and reactive angular help
            safe_nav = Twist()
            safe_nav.linear.x = self.last_nav_cmd.linear.x
            safe_nav.angular.z = self.last_nav_cmd.angular.z

            # If an obstacle is closer than front_thresh ahead, stop forward motion
            if (left_min < front_thresh or right_min < front_thresh) and safe_nav.linear.x > 0.0:
                rospy.logwarn("Nav blocked: obstacle ≤ 1 ft, stopping forward motion.")
                safe_nav.linear.x = 0.0

            # If we are too close to walls or front obstacles, borrow reactive angular z
            if (left_side_min < side_thresh or right_side_min < side_thresh or
                left_min < front_thresh or right_min < front_thresh):
                safe_nav.angular.z = reactive_twist.angular.z

            self.cmd_pub.publish(safe_nav)
        else:
            # FINAL PROJECT BEHAVIOR:
            # No teleop, no nav → FULL STOP (no wandering).
            stop = Twist()
            self.cmd_pub.publish(stop)

    # ------------------- RUN -------------------
    def run(self):
        rospy.loginfo("ReactiveController: spinning.")
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('reactive_controller', anonymous=True)
    controller = ReactiveController()
    controller.run()

