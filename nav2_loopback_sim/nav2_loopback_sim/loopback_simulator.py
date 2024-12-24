# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from nav2_simple_commander.line_iterator import LineIterator
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
import tf_transformations

from .utils import (
    addYawToQuat,
    getMapOccupancy,
    matrixToTransform,
    transformStampedToMatrix,
    worldToMap,
)

"""
This is a loopback simulator that replaces a physics simulator to create a
frictionless, inertialess, and collisionless simulation environment. It
accepts cmd_vel messages and publishes odometry & TF messages based on the
cumulative velocities received to mimic global localization and simulation.
It also accepts initialpose messages to set the initial pose of the robot
to place anywhere.
"""


class LoopbackSimulator(Node):

    def __init__(self):
        super().__init__(node_name='loopback_simulator')
        self.curr_cmd_vel = None
        self.curr_cmd_vel_time = self.get_clock().now()
        self.initial_pose = None
        self.timer = None
        self.setupTimer = None
        self.map = None
        self.mat_base_to_laser = None

        self.declare_parameter('update_duration', 0.01)
        self.update_dur = self.get_parameter('update_duration').get_parameter_value().double_value

        self.declare_parameter('base_frame_id', 'base_footprint')
        self.base_frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self.declare_parameter('map_frame_id', 'map')
        self.map_frame_id = self.get_parameter('map_frame_id').get_parameter_value().string_value

        self.declare_parameter('odom_frame_id', 'odom')
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value

        self.declare_parameter('scan_frame_id', 'base_scan')
        self.scan_frame_id = self.get_parameter('scan_frame_id').get_parameter_value().string_value

        self.declare_parameter('enable_stamped_cmd_vel', True)
        use_stamped = self.get_parameter('enable_stamped_cmd_vel').get_parameter_value().bool_value

        self.declare_parameter('scan_publish_dur', 0.1)
        self.scan_publish_dur = self.get_parameter(
            'scan_publish_dur').get_parameter_value().double_value

        self.declare_parameter('publish_map_odom_tf', True)
        self.publish_map_odom_tf = self.get_parameter(
            'publish_map_odom_tf').get_parameter_value().bool_value

        self.declare_parameter('publish_clock', True)
        self.publish_clock = self.get_parameter('publish_clock').get_parameter_value().bool_value

        self.t_map_to_odom = TransformStamped()
        self.t_map_to_odom.header.frame_id = self.map_frame_id
        self.t_map_to_odom.child_frame_id = self.odom_frame_id
        self.t_odom_to_base_link = TransformStamped()
        self.t_odom_to_base_link.header.frame_id = self.odom_frame_id
        self.t_odom_to_base_link.child_frame_id = self.base_frame_id

        self.tf_broadcaster = TransformBroadcaster(self)

        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose', self.initialPoseCallback, 10)
        if not use_stamped:
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                'cmd_vel', self.cmdVelCallback, 10)
        else:
            self.cmd_vel_sub = self.create_subscription(
                TwistStamped,
                'cmd_vel', self.cmdVelStampedCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', sensor_qos)

        if self.publish_clock:
            self.clock_timer = self.create_timer(0.1, self.clockTimerCallback)
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        self.setupTimer = self.create_timer(0.1, self.setupTimerCallback)

        self.map_client = self.create_client(GetMap, '/map_server/map')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.getMap()

        self.info('Loopback simulator initialized')

    def getBaseToLaserTf(self):
        try:
            # Wait for transform and lookup
            transform = self.tf_buffer.lookup_transform(
                self.base_frame_id, self.scan_frame_id, rclpy.time.Time())
            self.mat_base_to_laser = transformStampedToMatrix(transform)

        except Exception as ex:
            self.get_logger().error('Transform lookup failed: %s' % str(ex))

    def setupTimerCallback(self):
        # Publish initial identity odom transform & laser scan to warm up system
        self.tf_broadcaster.sendTransform(self.t_odom_to_base_link)
        if self.mat_base_to_laser is None:
            self.getBaseToLaserTf()

    def clockTimerCallback(self):
        msg = Clock()
        msg.clock = self.get_clock().now().to_msg()
        self.clock_pub.publish(msg)

    def cmdVelCallback(self, msg):
        self.debug('Received cmd_vel')
        if self.initial_pose is None:
            # Don't consider velocities before the initial pose is set
            return
        self.curr_cmd_vel = msg
        self.curr_cmd_vel_time = self.get_clock().now()

    def cmdVelStampedCallback(self, msg):
        self.debug('Received cmd_vel')
        if self.initial_pose is None:
            # Don't consider velocities before the initial pose is set
            return
        self.curr_cmd_vel = msg.twist
        self.curr_cmd_vel_time = rclpy.time.Time.from_msg(msg.header.stamp)

    def initialPoseCallback(self, msg):
        self.info('Received initial pose!')
        if self.initial_pose is None:
            # Initialize transforms (map->odom as input pose, odom->base_link start from identity)
            self.initial_pose = msg.pose.pose
            self.t_map_to_odom.transform.translation.x = self.initial_pose.position.x
            self.t_map_to_odom.transform.translation.y = self.initial_pose.position.y
            self.t_map_to_odom.transform.translation.z = 0.0
            self.t_map_to_odom.transform.rotation = self.initial_pose.orientation
            self.t_odom_to_base_link.transform.translation = Vector3()
            self.t_odom_to_base_link.transform.rotation = Quaternion()
            self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link)

            # Start republication timer and velocity processing
            if self.setupTimer is not None:
                self.setupTimer.cancel()
                self.setupTimer.destroy()
                self.setupTimer = None
            self.timer = self.create_timer(self.update_dur, self.timerCallback)
            self.timer_laser = self.create_timer(self.scan_publish_dur, self.publishLaserScan)
            return

        self.initial_pose = msg.pose.pose

        # Adjust map->odom transform based on new initial pose, keeping odom->base_link the same
        t_map_to_base_link = TransformStamped()
        t_map_to_base_link.header = msg.header
        t_map_to_base_link.child_frame_id = self.base_frame_id
        t_map_to_base_link.transform.translation.x = self.initial_pose.position.x
        t_map_to_base_link.transform.translation.y = self.initial_pose.position.y
        t_map_to_base_link.transform.translation.z = 0.0
        t_map_to_base_link.transform.rotation = self.initial_pose.orientation
        mat_map_to_base_link = transformStampedToMatrix(t_map_to_base_link)
        mat_odom_to_base_link = transformStampedToMatrix(self.t_odom_to_base_link)
        mat_base_link_to_odom = tf_transformations.inverse_matrix(mat_odom_to_base_link)
        mat_map_to_odom = \
            tf_transformations.concatenate_matrices(mat_map_to_base_link, mat_base_link_to_odom)
        self.t_map_to_odom.transform = matrixToTransform(mat_map_to_odom)

    def timerCallback(self):
        # If no data, just republish existing transforms without change
        one_sec = Duration(seconds=1)
        if self.curr_cmd_vel is None or self.get_clock().now() - self.curr_cmd_vel_time > one_sec:
            self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link)
            self.curr_cmd_vel = None
            return

        # Update odom->base_link from cmd_vel
        dx = self.curr_cmd_vel.linear.x * self.update_dur
        dy = self.curr_cmd_vel.linear.y * self.update_dur
        dth = self.curr_cmd_vel.angular.z * self.update_dur
        q = [self.t_odom_to_base_link.transform.rotation.x,
             self.t_odom_to_base_link.transform.rotation.y,
             self.t_odom_to_base_link.transform.rotation.z,
             self.t_odom_to_base_link.transform.rotation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(q)
        self.t_odom_to_base_link.transform.translation.x += dx * math.cos(yaw) - dy * math.sin(yaw)
        self.t_odom_to_base_link.transform.translation.y += dx * math.sin(yaw) + dy * math.cos(yaw)
        self.t_odom_to_base_link.transform.rotation = \
            addYawToQuat(self.t_odom_to_base_link.transform.rotation, dth)

        self.publishTransforms(self.t_map_to_odom, self.t_odom_to_base_link)
        self.publishOdometry(self.t_odom_to_base_link)

    def publishLaserScan(self):
        # Publish a bogus laser scan for collision monitor
        self.scan_msg = LaserScan()
        self.scan_msg.header.stamp = (self.get_clock().now()).to_msg()
        self.scan_msg.header.frame_id = self.scan_frame_id
        self.scan_msg.angle_min = -math.pi
        self.scan_msg.angle_max = math.pi
        # 1.5 degrees
        self.scan_msg.angle_increment = 0.0261799
        self.scan_msg.time_increment = 0.0
        self.scan_msg.scan_time = 0.1
        self.scan_msg.range_min = 0.05
        self.scan_msg.range_max = 30.0
        num_samples = int(
            (self.scan_msg.angle_max - self.scan_msg.angle_min) /
            self.scan_msg.angle_increment)
        self.scan_msg.ranges = [0.0] * num_samples
        self.getLaserScan(num_samples)
        self.scan_pub.publish(self.scan_msg)

    def publishTransforms(self, map_to_odom, odom_to_base_link):
        map_to_odom.header.stamp = \
            (self.get_clock().now() + Duration(seconds=self.update_dur)).to_msg()
        odom_to_base_link.header.stamp = self.get_clock().now().to_msg()
        if self.publish_map_odom_tf:
            self.tf_broadcaster.sendTransform(map_to_odom)
        self.tf_broadcaster.sendTransform(odom_to_base_link)

    def publishOdometry(self, odom_to_base_link):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = odom_to_base_link.transform.translation.x
        odom.pose.pose.position.y = odom_to_base_link.transform.translation.y
        odom.pose.pose.orientation = odom_to_base_link.transform.rotation
        odom.twist.twist = self.curr_cmd_vel
        self.odom_pub.publish(odom)

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    def getMap(self):
        request = GetMap.Request()
        if self.map_client.wait_for_service(timeout_sec=5.0):
            # Request to get map
            future = self.map_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.map = future.result().map
                self.get_logger().info('Laser scan will be populated using map data')
            else:
                self.get_logger().warn(
                    'Failed to get map, '
                    'Laser scan will be populated using max range'
                )
        else:
            self.get_logger().warn(
                'Failed to get map, '
                'Laser scan will be populated using max range'
            )

    def getLaserPose(self):
        mat_map_to_odom = transformStampedToMatrix(self.t_map_to_odom)
        mat_odom_to_base = transformStampedToMatrix(self.t_odom_to_base_link)

        mat_map_to_laser = tf_transformations.concatenate_matrices(
            mat_map_to_odom,
            mat_odom_to_base,
            self.mat_base_to_laser
        )
        transform = matrixToTransform(mat_map_to_laser)

        x = transform.translation.x
        y = transform.translation.y
        theta = tf_transformations.euler_from_quaternion([
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ])[2]

        return x, y, theta

    def getLaserScan(self, num_samples):
        if self.map is None or self.initial_pose is None or self.mat_base_to_laser is None:
            self.scan_msg.ranges = [self.scan_msg.range_max - 0.1] * num_samples
            return

        x0, y0, theta = self.getLaserPose()

        mx0, my0 = worldToMap(x0, y0, self.map)

        if not 0 < mx0 < self.map.info.width or not 0 < my0 < self.map.info.height:
            # outside map
            self.scan_msg.ranges = [self.scan_msg.range_max - 0.1] * num_samples
            return

        for i in range(num_samples):
            curr_angle = theta + self.scan_msg.angle_min + i * self.scan_msg.angle_increment
            x1 = x0 + self.scan_msg.range_max * math.cos(curr_angle)
            y1 = y0 + self.scan_msg.range_max * math.sin(curr_angle)

            mx1, my1 = worldToMap(x1, y1, self.map)

            line_iterator = LineIterator(mx0, my0, mx1, my1, 0.5)

            while line_iterator.isValid():
                mx, my = int(line_iterator.getX()), int(line_iterator.getY())

                if not 0 < mx < self.map.info.width or not 0 < my < self.map.info.height:
                    # if outside map then check next ray
                    break

                point_cost = getMapOccupancy(mx, my, self.map)

                if point_cost >= 60:
                    self.scan_msg.ranges[i] = math.sqrt(
                        (int(line_iterator.getX()) - mx0) ** 2 +
                        (int(line_iterator.getY()) - my0) ** 2
                    ) * self.map.info.resolution
                    break

                line_iterator.advance()


def main():
    rclpy.init()
    loopback_simulator = LoopbackSimulator()
    rclpy.spin(loopback_simulator)
    loopback_simulator.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
