import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

def generate_launch_description():
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    teleop_config = launch.substitutions.LaunchConfiguration('teleop_config')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('teleop_config',
            default_value=launch.substitutions.TextSubstitution(text=os.path.join(
                get_package_share_directory('tello_joy'), 'config', 'teleop_xbox.config.yaml'
            ))
        ),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.2,
                'autorepeat_rate': 20.0,
            }]
        ),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[teleop_config]
        ),
        launch_ros.actions.Node(
            package='tello_joy', executable='joy_xbox',
            name='joy_xbox_node'
        )
    ])
