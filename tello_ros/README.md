# tello_ros

| Package Name | Description | Notes |
|--|--|--|
| tello_bringup | Contains launch file and configuration to start all relevant Tello ROS nodes. | |
| tello_description | Provides the URDF/Xacro model description of the Tello drone, including frames and physical parameters. | Not implemented. |
| tello_driver | Driver package that handles communication with the real DJI Tello drone (e.g., commands, status data, video stream). | |
| tello_drone | Meta-package for all tello packages. | |
| tello_joy  Node for controlling the Tello drone using a joystick/gamepad via ROS joy messages. | |
| tello_meshes | Contains the 3D models/meshes (e.g., STL/DAE) of the Tello drone for visualization in RViz or simulation. | Not implemented. |
| tello_msgs | Defines the required ROS messages and services for communication and control of the Tello drone. | |


### Attribution

The navigation part of this project is derived from the [tiiuae/navigation repository](https://github.com/tiiuae/navigation), licensed under BSD-3-Clause.
