"""
spawn_turtlebot.py

Script used to spawn a turtlebot in a generic position
"""
import os
import sys
import rclpy
import argparse
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xml.etree.ElementTree as ET


def main():
    """ Main for spawning turtlebot node """
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn Turtlebot3 into Gazebo')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-t', '--turtlebot_type', type=str, default='waffle',
                        choices=['waffle', 'burger'])
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    args = parser.parse_args()

    # Start node
    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    node.get_logger().info('spawning a `{}` with name `{}` on namespace `{}` at {}, {}, {}'.format(
        args.turtlebot_type, args.robot_name, args.robot_namespace, args.x, args.y, args.z))

    # Get path to the turtlebot3 burgerbot
    sdf_file_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models",
        "turtlebot3_{}".format(args.turtlebot_type), "model.sdf")

    # We need to remap the transform (/tf) topic so each robot has its own.
    # We do this by adding `ROS argument entries` to the sdf file for
    # each plugin broadcasting a transform. These argument entries provide the
    # remapping rule, i.e. /tf -> /<robot_id>/tf
    tree = ET.parse(sdf_file_path)
    root = tree.getroot()
    for plugin in root.iter('plugin'):
        if 'turtlebot3_diff_drive' in plugin.attrib.values():
            # The only plugin we care for now is 'diff_drive' which is
            # broadcasting a transform between`odom` and `base_footprint`
            break

    ros_params = plugin.find('ros')
    ros_tf_remap = ET.SubElement(ros_params, 'argument')
    ros_tf_remap.text = '/tf:=/' + args.robot_namespace + '/tf'

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.xml = ET.tostring(root, encoding="unicode")
    request.robot_namespace = args.robot_namespace
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
