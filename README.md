NXP-CODES
This repository contains the core ROS nodes used to operate an autonomous NXP-style vehicle inside the Gazebo simulator. The system is built around three major functions: lane interpretation, steering computation, and basic obstacle handling.

Overview
The project is structured into three main ROS nodes:


Lane extraction and tracking


Steering vector generation


Object and obstacle recognition


Together they create a simple perception-to-control pipeline for autonomous driving in simulation.

1. ros_line_follower.py
This node performs lane perception and produces initial control cues.
Key Functions


Subscribes to the simulated camera feed


Identifies lane boundaries or track edges


Computes lateral offset and heading deviation


Publishes steering targets or error values for downstream control


Summary
Provides the vehicle with lane understanding and decides the general turning direction.

2. ros_edge_vectors.py
This node converts lane-related errors into a smooth steering command.
Key Functions


Receives deviation and orientation errors from the line follower


Generates a stable steering vector


Uses smoothing or a simple control approach to limit sudden movements


Summary
Transforms raw perception data into a clean steering instruction for the vehicle.

3. ros_obj_recg.py
This node focuses on basic obstacle awareness.
Key Functions


Processes camera or sensor data to detect nearby objects


Determines potential risks or obstructions


Publishes warnings or slow/stop signals to the control system


Summary
Ensures the vehicle can react safely when obstacles appear on the track.

Pipeline
These three components combine to form a functional autonomous driving flow in Gazebo:
Lane Detection → Steering Vector Generation → Obstacle Handling
The result is a simple, modular, and simulation-ready driving stack for NXP-style autonomous vehicles.
