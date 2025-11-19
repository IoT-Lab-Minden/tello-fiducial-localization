# Assisted Localization of MAVs for Navigation in Indoor Environments Using Fiducial Markers

**Please note: This repository is not actively maintained and therefore provided as-is.**

This repository provides a collection of ROS 2 packages for controlling and localising a Ryze Tello EDU MAV in indoor environments using a fiducial marker map.

The project is structured into two main subsystems:

- tello_ros/ – All packages related to the Tello drone (driver, control, messages, bringup, etc.)
- fiducials/ – Marker-based localisation tools based on ArUco and fiducial maps

A more detailed overview of the individual ROS packages can be found in the README files of the subsystems.

## Installation

Before compiling the project, you need to install a [custom version](https://github.com/IoT-Lab-Minden/ctello) of the *ctello* library. All other installation steps are the same as with every other ROS 2 project.

## Usage

Example:

```bash
# Launch full Tello bringup
ros2 launch tello_bringup tello_full.launch.py

# Start joystick control
ros2 run tello_joy joy_logitech


# Start marker detection from MAV camera
ros2 launch aruco_detect aruco_detect.launch.py

# Launch multi-marker localization
ros2 run multi_marker_eval multi_marker_eval
```

**Note:** *For full navigation and path planning capabilities, an octomap needs to be provided, which is not part of this repository.*

## Citation

Please cite the following paper in your publications, if you use this work:

```bibtex
@inproceedings{AssistedLocalization,
    author    = "Kirsch, André and Riechmann, Malte and Koenig, Matthias",
    title     = "Assisted Localization of MAVs for Navigation in Indoor Environments Using Fiducial Markers",
    year      = 2023,
    booktitle = "European Conference on Mobile Robots (ECMR)",
    doi       = "10.1109/ECMR59166.2023.10256424"
}
```

## License

This [work](https://github.com/IoT-Lab-Minden/fiducial_localisation_tello) is licensed under [BSD-3-Clause license](LICENSE.md).
