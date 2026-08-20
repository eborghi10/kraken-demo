#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Turn Twist commands into the steering the O3DE Kraken actually accepts.

Nav2 and the scenario runner speak geometry_msgs/Twist, because that is what
controllers in this ecosystem emit. The Kraken is an Ackermann vehicle: it
subscribes to ackermann_msgs/AckermannDrive, is told a speed and a steering
angle, and its yaw rate is whatever the physics makes of the two.

The conversion is the bicycle model read backwards, w = v tan(d) / L. A command
to turn on the spot therefore has no answer, and is relayed as a stop rather
than as a guess: a vehicle that has to roll in order to turn should say so
plainly, not creep forward to make the caller's request look satisfiable.

The defaults describe the Kraken. Its prefab serialises no limits at all, so
the drive model runs on AckermannModelLimits' own defaults, of which the
steering limit of 0.7 rad is the one that binds here.
"""
import math

import rclpy
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AckermannBridge(Node):

    def __init__(self):
        super().__init__('ackermann_bridge')

        self.declare_parameter('wheelbase', 2.2)
        self.declare_parameter('max_steering_angle', 0.7)
        self.declare_parameter('min_speed', 0.05)

        self._wheelbase = self.get_parameter('wheelbase').value
        self._limit = self.get_parameter('max_steering_angle').value
        self._min_speed = self.get_parameter('min_speed').value

        self._drive_pub = self.create_publisher(AckermannDrive, 'ackermann_vel', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)

    def _on_cmd_vel(self, msg):
        command = AckermannDrive()
        speed = msg.linear.x
        if abs(speed) < self._min_speed:
            if abs(msg.angular.z) > 0.0:
                self.get_logger().warn(
                    'asked to turn at %.2f rad/s while standing still; an Ackermann '
                    'robot cannot, so it stops' % msg.angular.z,
                    throttle_duration_sec=5.0)
            self._drive_pub.publish(command)
            return

        angle = math.atan(self._wheelbase * msg.angular.z / speed)
        command.speed = speed
        command.steering_angle = max(-self._limit, min(self._limit, angle))
        if abs(angle) > self._limit:
            self.get_logger().warn(
                'steering %.2f rad requested, %.2f rad available; the turn will be '
                'wider than asked' % (angle, self._limit),
                throttle_duration_sec=5.0)
        self._drive_pub.publish(command)


def main():
    rclpy.init()
    node = AckermannBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
