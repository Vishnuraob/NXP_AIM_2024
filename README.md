NXP-CODES
This repository hosts the essential ROS components that drive an NXP-style autonomous car inside the Gazebo environment. The system is organized around three core functions: lane perception, steering computation, and obstacle awareness.
Main Components


ros_line_follower.py
This module handles visual lane interpretation and preliminary control decisions.




Reads the simulated camera stream


Extracts lane boundaries or track markings


Evaluates how far the vehicle is from the ideal center line and how much it is misaligned


Publishes steering-related errors or target angles for the next stage


In essence, this node gives the car the ability to perceive the track and determine the direction it should aim for.


ros_edge_vectors.py
This module is dedicated to generating stable steering commands.




Receives deviation and orientation errors from the line follower


Produces a refined steering vector


Applies smoothing logic or simple feedback control to avoid abrupt motion


Effectively, this node turns raw lane information into a steady and usable steering command for the vehicle.


ros_obj_recg.py
This component manages object and obstacle awareness.




Analyzes camera or sensor inputs to identify nearby obstacles


Determines whether an object may interfere with the vehicle’s path


Issues alerts or slow/stop signals to the rest of the system


Overall, this node keeps the vehicle aware of its surroundings so it can react safely.
These three modules together form a complete pipeline in simulation: the car understands the lane, computes smooth steering, and responds intelligently to objects encountered along the track.
