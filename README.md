IntelligentRoboticFinalProject/
├── catkin_ws/
│   ├── build/          # ROS build artifacts (generated)
│   ├── devel/          # ROS devel space (generated)
│   └── src/
│       └── smart_tourbot_pkg/
│           ├── launch/
│           │   ├── smart_tourbot_real.launch
│           │   └── robot_audio.launch
│           ├── scripts/
│           │   ├── world_model.py
│           │   ├── route_planner.py
│           │   ├── path_follower.py
│           │   ├── reactive_controller.py
│           │   └── tour_executor.py
│           ├── sounds/
│           │   ├── start_repf_entrance.wav
│           │   ├── repf_bays.wav
│           │   ├── repf_upper_walkway.wav
│           │   ├── deh_atrium.wav
│           │   ├── deh_cs_office.wav
│           │   ├── deh_labs.wav
│           │   └── highlight_hougen.wav
│           ├── rviz/
│           │   └── smart_tourbot.rviz
│           ├── package.xml
│           └── CMakeLists.txt
└── README.md



System Overview

SMART-TourBot uses a hybrid deliberative–reactive architecture:

Semantic World Model: Named locations and connectivity graph

Global Planning: A* search over semantic nodes

Path Execution: Proportional waypoint following

Reactive Safety: Obstacle avoidance, wall correction, bumper recovery

Human Interaction: Audio narration at tour stops

Hardware Requirements

TurtleBot 2 (Kobuki base)

Robot-mounted laptop (Ubuntu + ROS)

External desktop/laptop for visualization and high-level control

Sensors:

Wheel odometry

Depth sensor / laser scan

Bumper sensors

Speakers on robot laptop (for audio narration)

Software Requirements

Ubuntu 18.04

ROS Melodic

TurtleBot packages

sound_play package for audio output

Install required ROS packages:

sudo apt update
sudo apt install ros-melodic-turtlebot \
                 ros-melodic-turtlebot-bringup \
                 ros-melodic-turtlebot-navigation \
                 ros-melodic-sound-play

Audio Setup (Important)

Audio narration is handled using ROS’s sound_play node.

Ensure your robot laptop:

Has working speakers

Can play .wav files locally

Has sound_play installed

Test audio manually:

rosrun sound_play soundplay_node.py

Build Instructions

From the root of catkin_ws:

cd IntelligentRoboticFinalProject/catkin_ws
catkin_make
source devel/setup.bash


You should add this to your .bashrc (recommended):

source ~/IntelligentRoboticFinalProject/catkin_ws/devel/setup.bash

🚀 Running the System (Real Robot)
Step 1: On the Robot Laptop

Bring up TurtleBot base, sensors, and navigation stack:

roslaunch turtlebot_bringup minimal.launch


In a new terminal:

roslaunch turtlebot_navigation gmapping_demo.launch


Start audio playback node:

roslaunch smart_tourbot_pkg robot_audio.launch

Step 2: On the Desktop / Control Laptop

Set ROS master to the robot:

export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_HOSTNAME=<DESKTOP_IP>


Launch the SMART-TourBot system:

roslaunch smart_tourbot_pkg smart_tourbot_real.launch


This will start:

Semantic world model

A* route planner

Path follower

Reactive controller

Tour executor

RViz visualization

🗺 Tour Behavior

The robot executes the following tour sequence:

START_REPF_ENTRANCE (welcome narration)

REPF_BAYS

REPF_UPPER_WALKWAY

DEH_ATRIUM

DEH_CS_OFFICE

DEH_LABS

HIGHLIGHT_HOUGEN

At each location:

Robot stops

Plays corresponding .wav narration

Waits for a configurable duration (default: 20 seconds)

🎛 Key ROS Nodes
Node	Purpose
world_model.py	Semantic graph of locations
route_planner.py	A* planning over semantic graph
path_follower.py	Waypoint tracking and motion control
reactive_controller.py	Obstacle avoidance & safety
tour_executor.py	Tour state machine & audio narration
soundplay_node.py	Audio playback
📊 Visualization (RViz)

RViz displays:

Map and grid

Global planned path

Robot pose and trajectory

Sensor data

Use the provided configuration:

smart_tourbot_pkg/rviz/smart_tourbot.rviz

⚠️ Known Limitations

Uses a hand-crafted semantic map

No SLAM-based dynamic replanning

Map drift may occur due to odometry errors

Audio depends on local speaker configuration

🔮 Future Work

Replace semantic map with SLAM

Add dynamic replanning for blocked paths

Improve localization robustness

Add richer human–robot interaction

Support multi-floor navigation

👥 Team Contributions

Subhash Chandra
System design, ROS architecture, semantic modeling, planning, control, audio integration, testing, and documentation.

Brandon Aviles
Brainstorming, environment measurements, system understanding, testing, debugging, and live demonstration support.

🙏 Acknowledgements

Special thanks to Prof. Dean Hougen
for guidance and instruction in CS 4023/5023 – Intelligent Robotics,
and to the University of Oklahoma School of Computer Science.

📜 License

This project is intended for academic and educational use.

If you want, next I can:

Generate a LICENSE file

Create a short README for GitHub front page

Add troubleshooting section

Convert this into GitHub Wiki pages



