import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    tello_config = launch.substitutions.LaunchConfiguration('tello_config')

    return launch.LaunchDescription([

        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_pub_ipad_to_ipad_camera', arguments = ["0.0", "0.0", "0.0", "1.0", "0.0", "0.0", "0.0", "ipad", "ipad_camera"]
        ),

        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_pub_base_link_to_eval_pose', arguments = ["-0.09", "0.1225", "0.08", "0.5", "-0.5", "0.5", "-0.5", "base_link", "eval_pose"]
        ),

        launch_ros.actions.Node(
            package='eval_data_recorder', executable='eval_data_recorder'
        )
    ])