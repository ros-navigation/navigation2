from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    default_pose_file = os.path.join(os.environ['HOME'], 'last_known_pose.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('auto_start_saving', default_value='true'),
        DeclareLaunchArgument('auto_restore_pose', default_value='false'),
        DeclareLaunchArgument(
            'pose_file_path',
            default_value=default_pose_file,
            description='Full path to save the pose file'
        ),
        DeclareLaunchArgument('save_interval_sec', default_value='0.5'),

        Node(
            package='nav2_toolkit',
            executable='pose_saver_node',
            name='pose_saver',
            output='screen',
            parameters=[{
                'auto_start_saving': LaunchConfiguration('auto_start_saving'),
                'auto_restore_pose': LaunchConfiguration('auto_restore_pose'),
                'pose_file_path': LaunchConfiguration('pose_file_path'),
                'save_interval_sec': LaunchConfiguration('save_interval_sec'),
            }]
        )
    ])
