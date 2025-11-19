# fiducials

| Package Name | Description | Notes |
|--|--|--|
| aruco_detect | Detects fiducial markers on images. Required for drone images. | |
| eval_data_recorder | Recordes pose data for paper evaluation. | Not required for normal usage. |
| fiducial_localisation_3d | Single-marker localisation of the Tello drone. | Choose between this and multi_marker_eval. |
| fiducial_map_localisation | Creates a fiducial marker map based on tablet position data. | Only required for map creation. |
| fiducial_msgs | Defines ROS messages and services for interacting with the fiducial localisation. | |
| marker_publisher | Publishes fiducial marker poses from a csv file. | |
| multi_marker_eval | Multi-marker localisation of the Tello drone. | Choose between this an fiducial_localisation_3d |

### Attribution

This project is based on the fiducials implementation from the [UbiquityRobotics/fiducials repository](https://github.com/UbiquityRobotics/fiducials), licensed under BSD-3-Clause.
