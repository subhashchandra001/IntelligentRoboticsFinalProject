# SMART-TourBot  
## Hybrid Autonomous Tour-Guide Robot Using ROS and TurtleBot 2

This repository contains the complete **final project** for  
**CS 4023/5023 – Intelligent Robotics** at the **University of Oklahoma**.

The project implements **SMART-TourBot**, an autonomous indoor tour-guide robot that combines **semantic task-level planning**, **A\* global route planning**, **path following**, **reactive obstacle avoidance**, and **audio narration**.  
The system runs on a **TurtleBot 2 (Kobuki base)** using **ROS Melodic**.

---

## Repository Structure

```
IntelligentRoboticFinalProject/
├── catkin_ws/
│   ├── build/
│   ├── devel/
│   └── src/
│       └── smart_tourbot_pkg/
│           ├── launch/
│           ├── scripts/
│           ├── sounds/
│           ├── rviz/
│           ├── package.xml
│           └── CMakeLists.txt
└── README.md
```

---

## System Overview

SMART-TourBot uses a **hybrid deliberative–reactive architecture** combining global planning and local safety behaviors to deliver reliable autonomous tours in indoor environments.

---

## Hardware Requirements

- TurtleBot 2 (Kobuki base)
- Robot-mounted laptop (Ubuntu + ROS)
- External desktop/laptop for visualization
- Wheel odometry, depth sensor, bumper
- Speakers for audio narration

---

## Software Requirements

- Ubuntu 18.04
- ROS Melodic
- TurtleBot ROS packages
- sound_play package

---

## Build Instructions

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

---

## Running the System

```bash
roslaunch smart_tourbot_pkg smart_tourbot_real.launch
```

---

## Team Contributions

- **Subhash Chandra**: Architecture, planning, control, audio, testing, documentation  
- **Brandon Aviles**: Brainstorming, measurements, testing, debugging, demo support

---

## Acknowledgements

Thanks to **Prof. Dean Hougen** for guidance and support.

---

## License

Academic and educational use only.
