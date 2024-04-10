# Design And Development Of Autonomous Mobile Robot For Food Delivery In Restaurants

Welcome to the Autonomous Mobile Robot project repository. This project focuses on the design, development, and implementation of an autonomous mobile robot using ROS (Robot Operating System) for navigation and control.

## Project Overview

This project aims to develop an autonomous food delivery robot with the capability to deliver customer orders. Initially, the plan included designing an Android app for customers to place orders. However, due to the lack of documentation on ROS and Android integration and time constraints, the project shifted towards implementing an alternative approach. The chosen method involves the robot autonomously navigating a cluttered restaurant space, dynamically re-planning its path from the home location to customer tables, delivering orders, and finally returning to its home base.

The primary focus is on creating a seamless and efficient food delivery system, optimizing the robot's navigation in complex environments. The project strives to enhance customer experience through reliable autonomous delivery and obstacle-avoidance mechanisms.


## Specifications
* The robot should establish bidirectional communication with a laptop using a designated transceiver setup.
* The robot must possess the capability to detect and process images obtained from the designated transceiver.
* The robot should demonstrate autonomous navigation from its home location to specified customer tables and back.
* The robot needs to dynamically adjust its path in response to detecting new obstacles within the environment.
* The robot should incorporate a feature to play varied music based on received feedback.

1. *Robot Hardware:*
   - Design and construction of the physical robot platform.
   - Integration of sensor, harwares and microprocesser such as Lidar, DC Encoder Motors, Motor Driver and  Nvidia Jetson Nano

2. *ROS Software Stack:*
   - Utilization of ROS for communication, control, and simulation.
   - Development of ROS nodes for sensor interfacing and actuator control.

3. *Sensor Fusion:*
   - Implementation of sensor fusion techniques for comprehensive environment perception.

4. *Localization and Mapping:*
   - Implementation of SLAM (Simultaneous Localization and Mapping) for mapping the robot's environment.
   - Integration of localization techniques to determine the robot's position in real-time.

5. *Path Planning and Navigation:*
   - Development of path planning algorithms for autonomous navigation.
   - Integration with ROS navigation stack for obstacle avoidance and dynamic path adaptation.

6. *User Interface (UI) for Robot:*
   - Design and implement a user-friendly interface for intuitive robot interaction and monitoring.

## The steps involved in this project have been described in detail as follows:-

1. *Modeling in fusion 360 and Converting into URDF file to run in GAZEBO simulator.*
2. *Gazebo Plugins*
3. *Intro to R-viz and RQT graphs*
4. *SLAM (Simultaneous localization and mapping)*
5. *Navigation Stack to Self-design robot*

### *Sample*

•  We used fusion 360 software for designing and developing the robot and then that design model has been converted into a URDF file.

### *Final model in Fusion360*

![Screenshot 2023-10-29 190840](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/221de9c6-b8cd-45f7-80aa-fd29c70808b5)
![gif for UI - Made with Clipchamp](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/d119f21b-0a5f-4722-90f9-97bc45177634)

### *Gazebo simulation of exported .urdf and .launch*
![WhatsApp Image 2024-01-11 at 11 53 19](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/a24478e2-d0c2-44f6-b509-7ba19cfc518c)

![gif for UI - Made with Clipchamp](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/3e7f9ba0-9e5d-4975-9d8d-04642a499fbb)

## 3D models using Fusion 360, and now, we've seamlessly converted them into URDF format for integration into ROS. 🎨➡️ Now, with teleoperation commands, we're able to control and navigate these robots effortlessly.

https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/13c8da64-cd97-456a-a61b-00fc7f58c072



## ROS (Robot Operating System) Information

### Installation

Make sure you have ROS Noetic installed on your system. If not, you can install it by following the instructions on the [official ROS website](http://wiki.ros.org/noetic/Installation).

## ROS Navigation

### SLAM (Simultaneous Localization and Mapping)

SLAM was implemented to allow the robot to create a map of its environment while navigating.

#### Gmapping Setup

To implement SLAM, the `gmapping` package was used. The robot was teleoperated to explore the environment, and the map was generated using the following steps:

1. Start Gazebo: `roslaunch your_robot_package gazebo.launch`
2. Start Teleoperation: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
3. Run Gmapping Node: `rosrun gmapping slam_gmapping scan:=/scan`
4. Visualize the Map in RVIZ.

#### Map Saving

After successful mapping, the map was saved using the `map_server` package:

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/your_robot_package/maps/map_name
```

## Reference
* [Programming Robots with ROS (Official ROS Wiki)](https://wiki.ros.org/)
* [ROS Navigation Stack Documentation](https://wiki.ros.org/navigation)
* [The Dynamic Window Approach to Collision Avoidance](https://ieeexplore.ieee.org/document/580977/)
* [A Comparison of Path Planning Algorithms for Omni-Directional Robots in Dynamic Environments](https://ieeexplore.ieee.org/document/4133821/)
* [Sebastian Thrun](https://en.wikipedia.org/wiki/Sebastian_Thrun) - Renowned in the field of autonomous mobile robots.
* [Cynthia Breazeal](https://en.wikipedia.org/wiki/Cynthia_Breazeal) - A pioneer in the development of robots in restaurant applications.
* [Rodney Brooks](https://en.wikipedia.org/wiki/Rodney_Brooks) - Contributed significantly to the advancement of autonomous mobile robots.

