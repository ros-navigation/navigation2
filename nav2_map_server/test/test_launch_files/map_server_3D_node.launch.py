import os

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()
    return LaunchDescription([
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[os.path.join(os.getenv('TEST_DIR'),
                                     'map_server_3D_params.yaml')])
    ])
