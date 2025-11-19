import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    tello_config = launch.substitutions.LaunchConfiguration('tello_config')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('tello_config',
            default_value=launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('tello_bringup'), 'config', 'tello.config.yaml'
            ))
        ),

        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('tello_joy'), 'launch', 'teleop_logitech.launch.py'
            ))
        ),

        # TODO: Replace with URDF
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_pub_map_to_odom', arguments = ["0.0", "0.0", "0", "0", "0", "0", "1", "map", "odom"]
        ),
        launch_ros.actions.Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='tf_pub_base_link_to_camera', arguments = ["0.04", "0.00", "0", "-0.5213", "0.5213", "-0.4777", "0.4777", "base_link", "camera"]
        ),

        launch_ros.actions.Node(
            package='tello_driver', executable='tello_driver', parameters=[tello_config]
        )
    ])
