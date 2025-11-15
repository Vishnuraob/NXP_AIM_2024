NXP-CODES
This repository contains the core ROS nodes used to run an autonomous NXP-style car in the Gazebo simulator.
The project focuses on three main parts: lane following, steering vector generation, and basic object recognition for obstacle handling.

Main Files
1. ros_line_follower.py
This node is responsible for lane detection and basic control.

Subscribes to the camera feed from Gazebo
Detects lane lines / track boundaries
Calculates the lateral error and heading error
Sends target steering values (or errors) to be handled by the control logic
In short: this file makes the car “see” the track and decide how much it should turn.

2. ros_edge_vectors.py
This node focuses on steering vector generation.

Takes the lane/position error from the line follower
Computes a smooth steering vector for the car
Can include simple PID or math-based smoothing to avoid jerky motion
In short: this file converts lane information into a clean steering command.

3. ros_obj_recg.py (Object Recognition)
This node deals with obstacle and object detection.

Processes sensor or image data to detect objects on or near the track
Flags obstacles that are too close to the car
Can publish stop/slow-down signals or warnings to other nodes
In short: this file helps the car stay safe by reacting to obstacles.

Together, these modules create a simple end-to-end autonomous driving pipeline in simulation: the car detects the lane, computes how to steer, and reacts to objects in its path.
