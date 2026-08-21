# SPDX-License-Identifier: BSD-3-Clause
"""Publish the transform O3DE reports but never broadcasts.

The simulator sends odometry as a topic and stops there, so tf2 has no
odom->base_link link and every costmap, controller and pose lookup downstream
fails. The frames are taken from the message rather than from parameters, so a
namespaced robot lands under its own prefix with no further configuration.

The same message is forwarded to `odometry/filtered` on a reliable connection.
O3DE publishes best effort, nav2 subscribes reliably, and the two never match:
the controller silently receives no odometry at all and steers believing the
robot is permanently stationary. Downstream this is the topic the EKF publishes
in the headless stack, so nav2 needs no remapping either way.
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


class OdomTf(Node):

    def __init__(self):
        super().__init__('odom_tf')
        self._broadcaster = TransformBroadcaster(self)
        self._odom = self.create_publisher(Odometry, 'odometry/filtered', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg):
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)
        self._odom.publish(msg)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
