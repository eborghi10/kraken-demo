#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Kinematic stand-in for the O3DE scene.

The demo's subject is what the localisation filter does when its sensors go
bad, not the fidelity of the vehicle model. O3DE gives a much better looking
answer to the second question but needs a GPU, an authored level and a long
build, none of which fit in CI. This node exposes the same topic contract
(ground truth, GNSS, IMU, wheel odometry) from a unicycle model so the
scenarios can run headless and deterministically.

It owns /clock, so scenarios run at `real_time_factor` times wall speed and
produce the same numbers on every machine.
"""
import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from kraken_sim.terrain import TractionField

EARTH_RADIUS_M = 6378137.0


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


class HeadlessSim(Node):

    def __init__(self):
        super().__init__('headless_sim')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('real_time_factor', 3.0)
        self.declare_parameter('seed', 0)
        # Must match navsat_transform's datum, or the estimate and the ground
        # truth end up in frames offset by the distance to that other datum.
        self.declare_parameter('datum', [52.2297, 21.0122, 0.0])

        self.declare_parameter('gnss_rate_hz', 10.0)
        self.declare_parameter('gnss_noise_stddev', 0.02)
        self.declare_parameter('imu_rate_hz', 50.0)
        self.declare_parameter('imu_yaw_noise_stddev', 0.004)
        self.declare_parameter('imu_yaw_rate_noise_stddev', 0.002)
        self.declare_parameter('imu_yaw_rate_bias', 0.0005)
        self.declare_parameter('wheel_rate_hz', 50.0)
        self.declare_parameter('wheel_speed_noise_stddev', 0.01)
        self.declare_parameter('wheel_lateral_slip_stddev', 0.01)
        self.declare_parameter('wheel_scale_error', 1.01)
        self.declare_parameter('cmd_vel_timeout', 1.0)
        # JSON, because a ROS 2 parameter cannot carry a list of dicts. Empty
        # means uniform traction, which reproduces the original flat model.
        self.declare_parameter('terrain', '')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self._rate = self.get_parameter('rate_hz').value
        self._rtf = self.get_parameter('real_time_factor').value
        self._datum = self.get_parameter('datum').value
        self._world_frame = self.get_parameter('world_frame').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._cmd_timeout = self.get_parameter('cmd_vel_timeout').value

        self._gnss_sigma = self.get_parameter('gnss_noise_stddev').value
        self._imu_yaw_sigma = self.get_parameter('imu_yaw_noise_stddev').value
        self._imu_rate_sigma = self.get_parameter('imu_yaw_rate_noise_stddev').value
        self._imu_rate_bias = self.get_parameter('imu_yaw_rate_bias').value
        self._wheel_sigma = self.get_parameter('wheel_speed_noise_stddev').value
        self._wheel_lateral_sigma = self.get_parameter('wheel_lateral_slip_stddev').value
        self._wheel_scale = self.get_parameter('wheel_scale_error').value
        self._terrain = TractionField.from_json(self.get_parameter('terrain').value)

        # Integer nanoseconds so the published clock never accumulates drift.
        self._dt_ns = int(1e9 / self._rate)
        self._dt = self._dt_ns * 1e-9
        self._now_ns = 0

        self._random = random.Random(self.get_parameter('seed').value)

        self._x = self._y = self._yaw = 0.0
        self._odom_x = self._odom_y = self._odom_yaw = 0.0
        self._cmd_v = self._cmd_w = 0.0
        self._last_cmd_ns = None

        self._gnss_every = max(1, round(self._rate / self.get_parameter('gnss_rate_hz').value))
        self._imu_every = max(1, round(self._rate / self.get_parameter('imu_rate_hz').value))
        self._wheel_every = max(1, round(self._rate / self.get_parameter('wheel_rate_hz').value))
        self._tick = 0

        clock_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._clock_pub = self.create_publisher(Clock, '/clock', clock_qos)
        self._truth_pub = self.create_publisher(Odometry, 'ground_truth/odom', 10)
        self._gnss_pub = self.create_publisher(NavSatFix, 'gnss/fix', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self._wheel_pub = self.create_publisher(Odometry, 'wheel/odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

        self.create_timer(self._dt / self._rtf, self._step)
        self.get_logger().info(
            'headless sim at %.0f Hz, %.1fx real time, datum %.6f %.6f, terrain %s'
            % (self._rate, self._rtf, self._datum[0], self._datum[1],
               'uniform' if self._terrain.uniform else 'patched'))

    def _on_cmd_vel(self, msg):
        self._cmd_v = msg.linear.x
        self._cmd_w = msg.angular.z
        self._last_cmd_ns = self._now_ns

    def _stamp(self):
        stamp = Clock().clock
        stamp.sec = self._now_ns // 1_000_000_000
        stamp.nanosec = self._now_ns % 1_000_000_000
        return stamp

    def _step(self):
        self._now_ns += self._dt_ns
        clock = Clock()
        clock.clock = self._stamp()
        self._clock_pub.publish(clock)

        stale = (self._last_cmd_ns is None
                 or (self._now_ns - self._last_cmd_ns) * 1e-9 > self._cmd_timeout)
        v = 0.0 if stale else self._cmd_v
        w = 0.0 if stale else self._cmd_w

        # The wheels turn at the commanded rate wherever the robot is; the
        # ground decides how much of that becomes motion.
        traction = self._terrain.traction_at(self._x, self._y)
        v_true = v * traction
        w_true = w * traction

        self._x += v_true * math.cos(self._yaw) * self._dt
        self._y += v_true * math.sin(self._yaw) * self._dt
        self._yaw = math.atan2(math.sin(self._yaw + w_true * self._dt),
                               math.cos(self._yaw + w_true * self._dt))

        # Wheel odometry sees a slightly wrong wheel radius plus noise, which is
        # what makes it drift without bound once GNSS stops correcting it.
        v_wheel = v * self._wheel_scale + self._random.gauss(0.0, self._wheel_sigma)
        w_wheel = w * self._wheel_scale + self._random.gauss(0.0, self._wheel_sigma)
        self._odom_x += v_wheel * math.cos(self._odom_yaw) * self._dt
        self._odom_y += v_wheel * math.sin(self._odom_yaw) * self._dt
        self._odom_yaw += w_wheel * self._dt

        self._tick += 1
        self._publish_truth(v_true, w_true)
        if self._tick % self._gnss_every == 0:
            self._publish_gnss()
        if self._tick % self._imu_every == 0:
            self._publish_imu(w_true)
        if self._tick % self._wheel_every == 0:
            self._publish_wheel(v_wheel, w_wheel)

    def _publish_truth(self, v, w):
        msg = Odometry()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._world_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        q = yaw_to_quaternion(self._yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self._truth_pub.publish(msg)

    def _publish_gnss(self):
        east = self._x + self._random.gauss(0.0, self._gnss_sigma)
        north = self._y + self._random.gauss(0.0, self._gnss_sigma)

        msg = NavSatFix()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._base_frame
        msg.status.status = NavSatStatus.STATUS_GBAS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self._datum[0] + math.degrees(north / EARTH_RADIUS_M)
        msg.longitude = self._datum[1] + math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(self._datum[0]))))
        msg.altitude = self._datum[2]
        var = self._gnss_sigma ** 2
        msg.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 9.0 * var]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self._gnss_pub.publish(msg)

    def _publish_imu(self, w):
        msg = Imu()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = 'imu_link'
        yaw = self._yaw + self._random.gauss(0.0, self._imu_yaw_sigma)
        q = yaw_to_quaternion(yaw)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        msg.orientation_covariance = [1e-6, 0.0, 0.0,
                                      0.0, 1e-6, 0.0,
                                      0.0, 0.0, self._imu_yaw_sigma ** 2]
        msg.angular_velocity.z = (w + self._imu_rate_bias
                                  + self._random.gauss(0.0, self._imu_rate_sigma))
        msg.angular_velocity_covariance = [1e-6, 0.0, 0.0,
                                           0.0, 1e-6, 0.0,
                                           0.0, 0.0, self._imu_rate_sigma ** 2]
        msg.linear_acceleration_covariance[0] = -1.0
        self._imu_pub.publish(msg)

    def _publish_wheel(self, v_wheel, w_wheel):
        msg = Odometry()
        msg.header.stamp = self._stamp()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        q = yaw_to_quaternion(self._odom_yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = v_wheel
        msg.twist.twist.angular.z = w_wheel
        var = self._wheel_sigma ** 2
        msg.twist.covariance[0] = var
        # A differential-drive base cannot travel sideways. Publishing that zero
        # with a covariance is what lets the filter fuse it as a constraint;
        # without it, lateral velocity is unobservable the moment GNSS stops.
        msg.twist.covariance[7] = self._wheel_lateral_sigma ** 2
        msg.twist.covariance[35] = var
        self._wheel_pub.publish(msg)


def main():
    rclpy.init()
    node = HeadlessSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
