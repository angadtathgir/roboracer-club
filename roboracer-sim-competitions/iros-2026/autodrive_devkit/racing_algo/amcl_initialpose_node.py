#!/usr/bin/env python3
"""
Seed Nav2 AMCL with /initialpose from current odometry.

The transform and message layout are the playground's
``localize.seed_initial_pose`` (copied into this package as ``localize.py``
+ ``store.py``). This node is only the ROS 2 stand-in for the playground
gateway, which is not part of the IROS image.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from localize import world_to_map, yaw_to_quat
import store


class AmclInitialPose(Node):
    def __init__(self):
        super().__init__('amcl_initialpose')
        self.declare_parameter('map_name', 'track-20260731')
        self.declare_parameter('publish_hz', 3.0)
        self.declare_parameter('amcl_fresh_s', 2.5)

        self._map_name = str(self.get_parameter('map_name').value)
        hz = max(0.5, float(self.get_parameter('publish_hz').value))
        self._fresh_s = float(self.get_parameter('amcl_fresh_s').value)

        self._m2w = None
        try:
            meta = store.map_meta(self._map_name)
            self._m2w = meta.get('map_to_world')
        except store.StoreError as e:
            self.get_logger().warn(f'Could not load map meta: {e}')

        self._odom = None
        self._amcl_mono = 0.0
        self._done = False
        self._published = 0

        self.create_subscription(
            Odometry, '/autodrive/roboracer_1/odom', self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        # Same topic / layout as playground localize.LocalizeManager.seed_initial_pose
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f'Seeding /initialpose from odom (map={self._map_name}, '
            f'map_to_world={self._m2w}) until /amcl_pose is fresh')

    def _odom_cb(self, msg: Odometry):
        self._odom = msg

    def _amcl_cb(self, _msg: PoseWithCovarianceStamped):
        self._amcl_mono = time.monotonic()

    def _tick(self):
        if self._done:
            return
        if self._amcl_mono > 0.0 and (
                time.monotonic() - self._amcl_mono) < self._fresh_s:
            self._done = True
            self.get_logger().info(
                f'AMCL pose is live after {self._published} /initialpose '
                'publishes; stopping seed')
            return
        if self._odom is None:
            return

        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        yaw_w = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        mx, my, myaw = world_to_map(
            float(p.x), float(p.y), yaw_w, self._m2w)
        qx, qy, qz, qw = yaw_to_quat(myaw)

        msg = PoseWithCovarianceStamped()
        # Stamp 0 → AMCL uses latest TF (avoids extrapolation warnings).
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(mx)
        msg.pose.pose.position.y = float(my)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = 0.25
        cov[7] = 0.25
        cov[35] = 0.068
        msg.pose.covariance = cov
        self._pub.publish(msg)
        self._published += 1


def main(args=None):
    rclpy.init(args=args)
    node = AmclInitialPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
