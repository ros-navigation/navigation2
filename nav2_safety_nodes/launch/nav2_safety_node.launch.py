from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '10',
                '--qos-profile', 'sensor_data',
                [LaunchConfiguration(variable_name='scanner'), '/scan'],
                'sensor_msgs/msg/LaserScan', yaml.dump({
                    'header': {'frame_id': 'scan'}, 'angle_min': -1.0,
                    'angle_max': 1.0, 'angle_increment': 0.1, 'range_max': 10.0,
                    'ranges': [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
                })
            ],
            name='scan_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'scan']
        ),
        Node(
            package='nav2_safety_nodes',
            executable='nav2_safety_node',
            name='nav2_safety_node',
            remappings=[('laser_scan', [LaunchConfiguration(variable_name='scanner'), '/scan']),
                        ('cloud', [LaunchConfiguration(variable_name='scanner'), '/cloud'])],
            parameters=[{'safety_polygon': '[[-0.325, -0.325], [-0.325, 0.325], [0.325, 0.325], [0.46, 0.0], [0.325, -0.325]]', 
                'base_frame': 'base_link', 'tf_tolerance': 0.01, 'zone_action': 0.0, 'zone_priority' : 1, 'zone_num_pts' : 1 }]
        ),
    ])
