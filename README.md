# SMART-TourBot: Hybrid Autonomous Tour-Guide Robot (ROS + TurtleBot 2)

This repository contains the **complete final project implementation** for  
**CS 4023/5023 – Intelligent Robotics (University of Oklahoma)**.

The project implements **SMART‑TourBot**, a fully autonomous indoor tour‑guide robot
using a **hybrid deliberative–reactive architecture** on a **TurtleBot 2 (Kobuki base)**.
The system performs semantic navigation between predefined locations, robust obstacle
avoidance, and synchronized audio narration at key tour stops.

---

## 1. Repository Structure

This repository is a **full ROS catkin workspace** and should be cloned as‑is.

```
IntelligentRoboticFinalProject/
├── catkin_ws/
│   ├── build/
│   ├── devel/
│   └── src/
│       └── smart_tourbot_pkg/
│           ├── launch/
│           │   ├── smart_tourbot_real.launch
│           │   ├── robot_audio.launch
│           │   └── (other launch files)
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
│           ├── package.xml
│           └── CMakeLists.txt
└── README.md
```

---

## 2. Hardware Setup

- **Mobile Base:** TurtleBot 2 (Kobuki)
- **On‑Robot Laptop:** Linux laptop mounted on TurtleBot
- **Desktop Machine:** External desktop or laptop for high‑level control and RViz
- **Sensors:**
  - Wheel odometry
  - Depth sensor (converted to LaserScan)
  - Bumper sensors
- **Audio Output:** TurtleBot onboard speakers (via sound_play)

---

## 3. Software Requirements

### Operating System
- Ubuntu 18.04

### ROS
- ROS Melodic

### Required ROS Packages
```bash
sudo apt install ros-melodic-turtlebot                  ros-melodic-turtlebot-bringup                  ros-melodic-turtlebot-navigation                  ros-melodic-sound-play                  ros-melodic-slam-gmapping
```

---

## 4. Network Configuration (IMPORTANT)

Both machines must be on the **same Wi‑Fi network**.

### On Robot Laptop (`~/.bashrc`)
```bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_HOSTNAME=<ROBOT_IP>
```

### On Desktop (`~/.bashrc`)
```bash
export ROS_MASTER_URI=http://<ROBOT_IP>:11311
export ROS_HOSTNAME=<DESKTOP_IP>
```

Reload:
```bash
source ~/.bashrc
```

---

## 5. Build Instructions (Both Machines)

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 6. Launch Sequence (CRITICAL)

### Step 1: Launch Core Robot Drivers (ROBOT LAPTOP)

```bash
roslaunch turtlebot_bringup minimal.launch
```

This starts:
- Kobuki base
- Bumper sensors
- Odometry
- `/cmd_vel` interface

---

### Step 2: Launch Mapping & Localization (ROBOT LAPTOP)

```bash
roslaunch turtlebot_navigation gmapping_demo.launch
```

This provides:
- `/map`
- `/tf`
- Localization and SLAM support

---

### Step 3: Launch Audio System (ROBOT LAPTOP)

```bash
roslaunch smart_tourbot_pkg robot_audio.launch
```

This starts:
- `soundplay_node`
- Sets `sounds_root` to:
  `/home/chan0239/catkin_ws/src/smart_tourbot_pkg/sounds`
- Enables WAV playback on robot speakers

---

### Step 4: Launch SMART‑TourBot System (DESKTOP)

```bash
roslaunch smart_tourbot_pkg smart_tourbot_real.launch
```

This starts:
- Semantic world model
- A* route planner
- Path follower
- Reactive controller
- Tour executor (state machine)
- RViz visualization

---

## 7. System Architecture Overview

SMART‑TourBot uses a **hybrid architecture**:

- **Deliberative Layer**
  - Semantic world model (graph of named locations)
  - A* planner over semantic nodes

- **Execution Layer**
  - Path follower using proportional control

- **Reactive Layer**
  - Bumper recovery
  - Obstacle avoidance
  - Wall correction
  - Velocity arbitration

- **Human Interaction**
  - Audio narration synchronized with tour stops

---

## 8. Tour Execution Logic

The tour is managed by a finite‑state machine:

```
IDLE → PRESENT → NEXT_STOP → TRAVEL → PRESENT → ... → DONE
```

- Audio narration plays during `PRESENT`
- Navigation occurs during `TRAVEL`
- Stops and pauses are configurable via ROS parameters

---

## 9. RViz Notes

- **Blue Path:** Global planned path (`/planned_path`)
- **Green Arrow:** Robot orientation and motion
- **Map Drift Warning:**  
  If paths and grid diverge, verify:
  - TF tree consistency
  - `/use_sim_time` is false on real robot
  - Correct ROS_MASTER_URI on both machines

---

## 10. Team Contributions

| Member | Contributions |
|------|---------------|
| **Subhash Chandra** | System architecture, semantic model, A* planner, path follower, reactive controller, tour executor, audio integration, debugging, testing, documentation |
| **Brandon Aviles** | Brainstorming system design, environment measurement, semantic layout planning, testing, debugging support, live demo assistance |

---

## 11. Acknowledgements

We thank **Prof. Dean Hougen** for guidance throughout the course and for providing
the TurtleBot platform and laboratory facilities used in this project.

---

## 12. Course Information

- **Course:** CS 4023/5023 – Intelligent Robotics
- **Institution:** University of Oklahoma
- **Semester:** Fall 2024

---

## 13. License

This project is for **academic use only** as part of coursework at the University of Oklahoma.
