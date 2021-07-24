# Programming a Real Self-Driving Car

This is the final project of the [Udacity](https://www.udacity.com) [Self-Driving Car Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013): Programming a Real Self-Driving Car. 

For this project, I wrote ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. 

# Project Specification

Section | Criteria | Specification | Status
:--- | :--- | :--- | :---
**Running the Code** | The code is built successfully and connects to the simulator. | Running `catkin_make`, source devel/setup.sh and roslaunch [launch/styx.launch](ros/launch/styx.launch) within the [ros](ros) directory results in no errors and allows the program to connect to the simulator. | Open 
**Control and Planning** | Waypoints are published to plan Carla’s route around the track. | Waypoints should be published to `/final_waypoints` to plan the vehicle’s path around the track. No unnecessary moves (excessive lane changes, unnecessary turning, unprompted stops) should occur. **Acceleration** should not exceed `10 m/s^2` and **jerk** should not exceed `10 m/s^3`. The top speed of the vehicle is limited to the km/h velocity set by the `velocity` `rosparam` in `waypoint_loader`. | Open
| | Controller commands are published to operate Carla’s throttle, brake, and steering. | `dbw_node.py` has been implemented to calculate and provide appropriate throttle, brake, and steering commands. The commands are published to `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and `/vehicle/steering_cmd`, as applicable. | Open
**Successful Navigation** | Successfully navigate the full track more than once. | The vehicle is able to complete more than one full loop of the track without running off road or any other navigational issues (incorrect turns, random stops, teleportation, etc.). | Open


# System Architecture

The following is a system architecture diagram showing the ROS nodes and topics used in the project. The [Code Structure section](#code-structure) below provides a summary of the ROS nodes and topics shown in the diagram.

![](images/system_architecture.png)

# Code Structure

The code for this project is in the [ros/src](ros/src) directory. This directory contains the following ROS packages:

## Package: Traffic Light Detector

The [tl_detector](ros/src/tl_detector) package contains the traffic light detection node: [tl_detector.py](ros/src/tl_detector/tl_detector.py). This node takes in data from the `/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic lights to the `/traffic_waypoint` topic.

The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints` provides a complete list of waypoints the car will be following.

![](images/tl-detector-ros-graph.png)

## Package: Waypoint Updater

The [waypoint_updater](ros/src/waypoint_updater) package contains the waypoint updater node: [waypoint_updater.py](ros/src/waypoint_updater/waypoint_updater.py). The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node will subscribe to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

![](images/waypoint-updater-ros-graph.png)

## Package: Twist Controller

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. The [twist_controller](ros/src/twist_controller) package contains the files that are responsible for control of the vehicle: the node [dbw_node.py](ros/src/twist_controller/dbw_node.py) and the file [twist_controller.py](ros/src/twist_controller/twist_controller.py), along with a PID and lowpass filter that are used in the implementation. The `dbw_node` subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates whether the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

![](images/dbw-node-ros-graph.png)

## Additional Packages

The `styx` and `styx_msgs` packages are used to provide a link between the simulator and ROS, and to provide custom ROS message types:

* **styx**: The [styx](ros/src/styx) package contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
* **styx_msgs**: The [styx_msgs](ros/src/styx_msgs) package includes definitions of the custom ROS message types used in the project.
* **Waypoint Loader**: The [waypoint_loader](ros/src/waypoint_loader) package loads the static waypoint data and publishes to `/base_waypoints`.
* **Waypoint Follower**: The [waypoint_follower](ros/src/waypoint_follower) package containing code from [Autoware](https://github.com/CPFL/Autoware) which subscribes to `/final_waypoints` and publishes target vehicle linear and angular velocities in the form of twist commands to the `/twist_cmd` topic.

# References

