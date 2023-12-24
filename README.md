# Design and Fabrication of Autonomous Mobile Robot for Food Delivery in Restaurant

## ROS (Robot Operating System) Information

### Installation

Make sure you have ROS Noetic installed on your system. If not, you can install it by following the instructions on the [official ROS website](http://wiki.ros.org/noetic/Installation).

### URDF Model

The robot model was created using Fusion 360 and converted to URDF for simulation in Gazebo.

#### Fusion 360 Model

![Fusion 360 Model](path/to/fusion360_model_image.jpg)

#### URDF Model

![URDF Model](path/to/urdf_model_image.jpg)

### Gazebo Simulation

The URDF model was successfully uploaded to Gazebo for simulation. Below is an image of the robot in the Gazebo environment.

![Gazebo Simulation](path/to/gazebo_simulation_image.jpg)

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
