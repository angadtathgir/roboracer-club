from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Bridge only: the RL trainer / evaluator publishes throttle and steering.
    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                FindPackageShare('foxglove_bridge').find('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        )
    )

    return LaunchDescription([
        Node(
            package='autodrive_roboracer',
            executable='autodrive_bridge',
            name='autodrive_bridge',
            emulate_tty=True,
            output='screen',
        ),
        foxglove_bridge_launch,
    ])
