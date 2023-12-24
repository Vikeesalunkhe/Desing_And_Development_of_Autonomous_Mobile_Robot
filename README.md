# Design and Fabrication of Autonomous Mobile Robot for Food Delivery in Restaurant

## ROS (Robot Operating System) Information

### Installation

Make sure you have ROS Noetic installed on your system. If not, you can install it by following the instructions on the [official ROS website](http://wiki.ros.org/noetic/Installation).

### URDF Model

The robot model was created using Fusion 360 and converted to URDF for simulation in Gazebo.

#### Fusion 360 Model

![Screenshot 2023-10-29 190840](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/221de9c6-b8cd-45f7-80aa-fd29c70808b5)
#### URDF Model

![botex1](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/73d205e0-b5dd-4d4e-bf76-df9d99f4c6da)
### Gazebo Simulation

The URDF model was successfully uploaded to Gazebo for simulation. Below is an image of the robot in the Gazebo environment.

![gif for UI - Made with Clipchamp](https://github.com/Vikeesalunkhe/Desing_And_Development_of_Autonomous_Mobile_Robot/assets/117392336/3e7f9ba0-9e5d-4975-9d8d-04642a499fbb)
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
