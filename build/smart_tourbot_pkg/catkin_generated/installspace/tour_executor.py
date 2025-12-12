#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
tour_executor.py

High-level tour and P2P executor for SMART-TourBot.

Behavior:
  * Maintains a list of tour stops (semantic node names).
  * Acts as a small state machine with states:
      IDLE -> WAIT_PATH -> TRAVEL -> PRESENT -> NEXT_STOP -> DONE
  * Publishes /tour_goal (std_msgs/String) to trigger A* planning.
  * Subscribes to /planned_path (nav_msgs/Path) to know when a path is ready.
  * Subscribes to /nav_goal_reached (std_msgs/Bool) from path_follower.py
    so it knows when the robot has actually arrived at the goal.
  * Logs what it is doing (for debugging and for your report).

In this version:
  * At each stop, the robot "presents" for present_duration seconds.
  * Plays an audio narration clip for each stop using sound_play.
"""

import os

import rospy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Path
from sound_play.libsoundplay import SoundClient


class TourExecutor(object):
    def __init__(self):
        rospy.loginfo("TourExecutor: initializing...")

        # Parameters
        self.mode = rospy.get_param("~mode", "tour")  # "tour" or "p2p"
        self.frame_id = rospy.get_param("~frame_id", "map")

        # Directory on the ROBOT where audio files live.
        # You override this via a launch-file param.
        self.sounds_root = rospy.get_param(
            "~sounds_root",
            "/home/robot/catkin_ws/src/smart_tourbot_pkg/sounds",
        )

        # Initialize sound client (talks to /soundplay node)
        self.sound_client = SoundClient()
        rospy.loginfo("TourExecutor: using sounds_root = %s", self.sounds_root)

        # ------------------------------------------------------------------
        # Fixed tour order (semantic node names).
        # Must match world_model.py & route_planner.py.
        #
        # We explicitly include the START node so we can play a welcome
        # message at the beginning without moving.
        # ------------------------------------------------------------------
        self.tour_stops = [
            "START_REPF_ENTRANCE",  # Welcome (no motion)
            "REPF_BAYS",
            "REPF_UPPER_WALKWAY",
            "DEH_ATRIUM",
            "DEH_CS_OFFICE",
            "DEH_LABS",
            "HIGHLIGHT_HOUGEN",
        ]

        # Explicit mapping from node names to audio filenames.
        self.audio_files = {
            "START_REPF_ENTRANCE": "start_repf_entrance.wav",
            "REPF_BAYS":           "repf_bays.wav",
            "REPF_UPPER_WALKWAY":  "repf_upper_walkway.wav",
            "DEH_ATRIUM":          "deh_atrium.wav",
            "DEH_CS_OFFICE":       "deh_cs_office.wav",
            "DEH_LABS":            "deh_labs.wav",
            "HIGHLIGHT_HOUGEN":    "highlight_hougen.wav",
        }

        self.current_index = 0
        self.current_goal = None

        # State machine: IDLE, WAIT_PATH, TRAVEL, PRESENT, NEXT_STOP, DONE
        self.state = "IDLE"

        # Whether the user/group is OK (from group_monitor, later)
        self.user_ok = True

        # Latest path received
        self.latest_path = None

        # Navigation-done flag (from path_follower)
        self.nav_goal_reached = False

        # Time when we entered PRESENT state
        self.present_start_time = None
        # How long to pause at each destination (seconds)
        self.present_duration = rospy.get_param("~present_duration", 20.0)

        # Publishers
        self.tour_goal_pub = rospy.Publisher(
            "tour_goal",
            String,
            queue_size=1,
        )

        # Subscribers
        self.path_sub = rospy.Subscriber(
            "planned_path",
            Path,
            self.path_callback,
            queue_size=1,
        )

        self.user_ok_sub = rospy.Subscriber(
            "user_ok",
            Bool,
            self.user_ok_callback,
            queue_size=1,
        )

        # New: subscribe to nav_goal_reached from PathFollower
        self.nav_done_sub = rospy.Subscriber(
            "nav_goal_reached",
            Bool,
            self.nav_goal_reached_callback,
            queue_size=1,
        )

        # Timer to drive the state machine at a fixed rate
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update_state)

        rospy.loginfo(
            "TourExecutor: initialized in state '%s', mode '%s'.",
            self.state,
            self.mode,
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def path_callback(self, msg):
        """Called when a new planned path is received from the planner."""
        rospy.loginfo(
            "TourExecutor: received planned path with %d poses.",
            len(msg.poses),
        )
        self.latest_path = msg

        if self.state == "WAIT_PATH":
            rospy.loginfo(
                "TourExecutor: path ready for goal '%s'. Starting travel.",
                self.current_goal,
            )
            # Now the PathFollower will drive the robot.
            self.state = "TRAVEL"
            self.nav_goal_reached = False

    def user_ok_callback(self, msg):
        """Called when group monitor updates whether user is following."""
        self.user_ok = msg.data
        if not self.user_ok:
            rospy.logwarn("TourExecutor: user/group may be lost or too far behind.")
        else:
            rospy.loginfo("TourExecutor: user/group is OK.")

    def nav_goal_reached_callback(self, msg):
        """
        Called when PathFollower publishes that the current nav goal is reached.

        We only react to this when we're in TRAVEL. Then we switch to PRESENT,
        and audio will start from update_state().
        """
        if not msg.data:
            return

        rospy.loginfo(
            "TourExecutor: received nav_goal_reached for '%s' (state=%s).",
            self.current_goal,
            self.state,
        )

        if self.state == "TRAVEL":
            self.nav_goal_reached = True
            # Switch to PRESENT; update_state will handle audio / timing.
            self.state = "PRESENT"
            self.present_start_time = None

    # ------------------------------------------------------------------
    # Audio helper
    # ------------------------------------------------------------------
    def play_location_audio(self, location_name):
        """
        Play the narration audio clip for the given semantic location.

        Looks up a filename in self.audio_files; if not found, falls back to
        <lowercased_location_name>.wav.

        The full path is: self.sounds_root / filename

        NOTE: This path must be valid on the ROBOT where soundplay_node.py runs.
        """
        if not location_name:
            rospy.logwarn("TourExecutor: no location name for audio; skipping.")
            return

        filename = self.audio_files.get(
            location_name,
            location_name.lower() + ".wav",
        )
        filepath = os.path.join(self.sounds_root, filename)

        if not os.path.exists(filepath):
            rospy.logwarn(
                "TourExecutor: audio file not found for '%s': %s",
                location_name,
                filepath,
            )
            return

        rospy.loginfo(
            "TourExecutor: playing narration for '%s' from %s",
            location_name,
            filepath,
        )
        # Non-blocking; soundplay node handles playback.
        self.sound_client.playWave(filepath)

    # ------------------------------------------------------------------
    # State machine update
    # ------------------------------------------------------------------
    def update_state(self, event):
        """Timer callback driving the high-level state machine."""
        if self.mode != "tour":
            # For now we only implement 'tour' mode.
            return

        # --------------------- IDLE ---------------------
        if self.state == "IDLE":
            # Start the tour.
            if self.current_index < len(self.tour_stops):
                self.current_goal = self.tour_stops[self.current_index]
                rospy.loginfo(
                    "TourExecutor[IDLE]: starting tour. First stop: %s",
                    self.current_goal,
                )

                # Special case: first stop is the START node.
                # Robot is already there, so we don't plan; just present.
                if self.current_goal == "START_REPF_ENTRANCE":
                    rospy.loginfo(
                        "TourExecutor[IDLE]: at START_REPF_ENTRANCE; "
                        "presenting welcome without moving.",
                    )
                    self.state = "PRESENT"
                    self.present_start_time = None
                else:
                    # Normal behavior: ask planner for a path.
                    self.request_plan(self.current_goal)
                    self.state = "WAIT_PATH"
                    self.nav_goal_reached = False
            else:
                rospy.loginfo("TourExecutor: no tour stops defined.")
                self.state = "DONE"

        # --------------------- WAIT_PATH ---------------------
        elif self.state == "WAIT_PATH":
            # Just wait for path_callback to switch to TRAVEL.
            rospy.loginfo_throttle(
                5.0,
                "TourExecutor[WAIT_PATH]: waiting for planned path...",
            )

        # --------------------- TRAVEL ---------------------
        elif self.state == "TRAVEL":
            # Robot is moving under PathFollower control.
            # We only switch to PRESENT when nav_goal_reached_callback fires.
            rospy.loginfo_throttle(
                5.0,
                "TourExecutor[TRAVEL]: driving toward '%s'...",
                self.current_goal,
            )

        # --------------------- PRESENT ---------------------
        elif self.state == "PRESENT":
            # At a highlight: pause here, narrate for present_duration seconds.
            if self.present_start_time is None:
                # First time we enter PRESENT for this stop
                self.present_start_time = rospy.Time.now()
                rospy.loginfo(
                    "TourExecutor[PRESENT]: robot is at '%s'. "
                    "Pausing %.1f seconds here.",
                    self.current_goal,
                    self.present_duration,
                )
                # Play audio narration once, when we first arrive.
                self.play_location_audio(self.current_goal)
            else:
                elapsed = (rospy.Time.now() - self.present_start_time).to_sec()
                if elapsed >= self.present_duration:
                    # Done presenting at this stop; go to NEXT_STOP.
                    self.state = "NEXT_STOP"

        # --------------------- NEXT_STOP ---------------------
        elif self.state == "NEXT_STOP":
            self.current_index += 1
            if self.current_index >= len(self.tour_stops):
                rospy.loginfo(
                    "TourExecutor[NEXT_STOP]: tour complete! No more highlights.",
                )
                self.state = "DONE"
            else:
                self.current_goal = self.tour_stops[self.current_index]
                rospy.loginfo(
                    "TourExecutor[NEXT_STOP]: next tour stop is '%s'.",
                    self.current_goal,
                )

                # For all later stops, we DO expect movement.
                self.request_plan(self.current_goal)
                self.state = "WAIT_PATH"
                self.nav_goal_reached = False

        # --------------------- DONE ---------------------
        elif self.state == "DONE":
            rospy.loginfo_throttle(
                10.0,
                "TourExecutor[DONE]: tour finished. Waiting...",
            )

        # --------------------- FALLBACK ---------------------
        else:
            rospy.logwarn(
                "TourExecutor: unknown state '%s'. Resetting to IDLE.",
                self.state,
            )
            self.state = "IDLE"

    # ------------------------------------------------------------------
    # Helper methods
    # ------------------------------------------------------------------
    def request_plan(self, goal_name):
        """Publish a tour_goal to trigger A* planning."""
        if not goal_name:
            rospy.logwarn("TourExecutor: cannot request plan for empty goal.")
            return
        msg = String()
        msg.data = goal_name
        rospy.loginfo(
            "TourExecutor: requesting plan to '%s'.",
            goal_name,
        )
        # Reset per-goal flags
        self.nav_goal_reached = False
        self.latest_path = None
        self.tour_goal_pub.publish(msg)


def main():
    rospy.init_node("tour_executor")
    node = TourExecutor()
    rospy.loginfo("TourExecutor node is up.")
    rospy.spin()


if __name__ == "__main__":
    main()

