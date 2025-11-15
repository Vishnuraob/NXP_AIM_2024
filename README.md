NXP-CODES
This repository contains the core ROS nodes used to run an autonomous NXP-style vehicle in the Gazebo simulator. The system is organized into three main parts: lane extraction, steering vector generation, and simple obstacle handling.


ros_line_follower.py
This node is responsible for interpreting the simulated camera feed and identifying the track.




Reads the camera data from Gazebo


Detects lane boundaries or track edges


Computes lateral offset and heading deviation


Publishes steering-related errors or target angles


In short, this file gives the car the ability to understand the lane and determine the required turning direction.


ros_edge_vectors.py
This node generates the steering command from the lane information.




Receives deviation and heading error from the line follower


Produces a stable steering vector


Uses smoothing or basic feedback control to avoid sudden movements


This module converts raw perception outputs into a smooth and usable steering instruction.


ros_obj_recg.py
This node handles obstacle detection and basic safety logic.




Processes sensor or image data to identify objects on or near the track


Checks whether any detected object could block the vehicle’s path


Publishes warnings or slow/stop signals


This file ensures the vehicle can respond safely when obstacles appear ahead.
Overall, these three modules form a simple perception-to-control pipeline: the system detects the lane, computes the steering direction, and responds to obstacles during simulation.
