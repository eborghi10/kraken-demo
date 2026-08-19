#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Corrupts sensor streams on demand, between the simulator and the filter.

The injector is a shim: it subscribes to a raw sensor topic and republishes a
possibly-degraded copy on another topic, and the localisation stack is wired to
the republished one. Nothing inside the simulator or the filter has to know
that fault injection exists, which is what lets the same scenarios run against
O3DE and against the headless sim.
"""
import math
import random

import rclpy
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from kraken_interfaces.msg import FaultSpec
from kraken_interfaces.srv import SetFault

EARTH_RADIUS_M = 6378137.0

MODE_NAMES = {
    FaultSpec.MODE_HEALTHY: 'healthy',
    FaultSpec.MODE_DROPOUT: 'dropout',
    FaultSpec.MODE_DEGRADE: 'degrade',
    FaultSpec.MODE_BIAS_RAMP: 'bias_ramp',
    FaultSpec.MODE_SLIP: 'slip',
}

SUPPORTED = {
    'navsat': {FaultSpec.MODE_HEALTHY, FaultSpec.MODE_DROPOUT,
               FaultSpec.MODE_DEGRADE, FaultSpec.MODE_BIAS_RAMP},
    'imu': {FaultSpec.MODE_HEALTHY, FaultSpec.MODE_DROPOUT,
            FaultSpec.MODE_DEGRADE, FaultSpec.MODE_BIAS_RAMP},
    'odometry': {FaultSpec.MODE_HEALTHY, FaultSpec.MODE_DROPOUT,
                 FaultSpec.MODE_DEGRADE, FaultSpec.MODE_SLIP},
    # No slip mode: not being fooled by slip is the whole reason the sensor is
    # on the vehicle. It can still fail, drift or go noisy.
    'twist': {FaultSpec.MODE_HEALTHY, FaultSpec.MODE_DROPOUT,
              FaultSpec.MODE_DEGRADE, FaultSpec.MODE_BIAS_RAMP},
}

MSG_TYPES = {'navsat': NavSatFix, 'imu': Imu, 'odometry': Odometry,
             'twist': TwistWithCovarianceStamped}


class Channel:

    def __init__(self, name, kind, publish_tf):
        self.name = name
        self.kind = kind
        self.publish_tf = publish_tf
        self.spec = FaultSpec(channel=name, mode=FaultSpec.MODE_HEALTHY)
        self.started_at = None
        self.expires_at = None
        self.bias = 0.0
        self.last_transform = None


class FaultInjector(Node):

    def __init__(self):
        super().__init__('fault_injector')

        self.declare_parameter('channels', [''])
        self.declare_parameter('seed', 0)
        names = [n for n in self.get_parameter('channels').value if n]

        self._random = random.Random(self.get_parameter('seed').value)
        self._tf = TransformBroadcaster(self)
        self._channels = {}

        for name in names:
            self.declare_parameter('%s.type' % name, 'navsat')
            self.declare_parameter('%s.input' % name, '')
            self.declare_parameter('%s.output' % name, '')
            self.declare_parameter('%s.publish_tf' % name, False)

            kind = self.get_parameter('%s.type' % name).value
            source = self.get_parameter('%s.input' % name).value
            sink = self.get_parameter('%s.output' % name).value
            if kind not in MSG_TYPES:
                raise ValueError('channel %s has unknown type %r' % (name, kind))

            channel = Channel(name, kind, self.get_parameter('%s.publish_tf' % name).value)
            channel.publisher = self.create_publisher(MSG_TYPES[kind], sink, 10)
            self.create_subscription(
                MSG_TYPES[kind], source,
                lambda msg, c=channel: self._relay(c, msg), 10)
            self._channels[name] = channel
            self.get_logger().info('channel %s (%s): %s -> %s' % (name, kind, source, sink))

        self.create_service(SetFault, '~/set_fault', self._on_set_fault)
        self.create_service(Trigger, '~/clear_faults', self._on_clear_faults)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_set_fault(self, request, response):
        spec = request.fault
        channel = self._channels.get(spec.channel)
        if channel is None:
            response.success = False
            response.message = 'no channel named %r' % spec.channel
            return response
        if spec.mode not in SUPPORTED[channel.kind]:
            response.success = False
            response.message = '%s channels do not support mode %s' % (
                channel.kind, MODE_NAMES.get(spec.mode, spec.mode))
            return response

        channel.spec = spec
        channel.started_at = self._now()
        channel.bias = 0.0
        seconds = spec.duration.sec + spec.duration.nanosec * 1e-9
        channel.expires_at = channel.started_at + seconds if seconds > 0.0 else None

        response.success = True
        response.message = '%s -> %s' % (spec.channel, MODE_NAMES.get(spec.mode, spec.mode))
        self.get_logger().info(response.message)
        return response

    def _on_clear_faults(self, request, response):
        del request
        for channel in self._channels.values():
            channel.spec = FaultSpec(channel=channel.name, mode=FaultSpec.MODE_HEALTHY)
            channel.started_at = None
            channel.expires_at = None
            channel.bias = 0.0
        response.success = True
        response.message = 'all channels healthy'
        return response

    def _active_mode(self, channel):
        if channel.expires_at is not None and self._now() >= channel.expires_at:
            channel.spec = FaultSpec(channel=channel.name, mode=FaultSpec.MODE_HEALTHY)
            channel.expires_at = None
            channel.bias = 0.0
        return channel.spec.mode

    def _relay(self, channel, msg):
        mode = self._active_mode(channel)

        if mode == FaultSpec.MODE_BIAS_RAMP and channel.started_at is not None:
            channel.bias = channel.spec.bias_rate * (self._now() - channel.started_at)

        if channel.kind == 'navsat':
            out = self._apply_navsat(channel, mode, msg)
        elif channel.kind == 'imu':
            out = self._apply_imu(channel, mode, msg)
        elif channel.kind == 'twist':
            out = self._apply_twist(channel, mode, msg)
        else:
            out = self._apply_odometry(channel, mode, msg)

        if out is not None:
            channel.publisher.publish(out)
        if channel.publish_tf:
            self._broadcast_tf(channel, out if out is not None else None)

    def _apply_navsat(self, channel, mode, msg):
        # A receiver that loses lock keeps talking, it just reports NO_FIX. That
        # is a harder case for the filter than silence, so it is what we send.
        if mode == FaultSpec.MODE_DROPOUT:
            msg.status.status = NavSatStatus.STATUS_NO_FIX
            msg.position_covariance = [0.0] * 9
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            return msg

        spec = channel.spec
        offset_north = offset_east = 0.0
        if mode == FaultSpec.MODE_DEGRADE:
            offset_north = self._random.gauss(0.0, spec.noise_stddev)
            offset_east = self._random.gauss(0.0, spec.noise_stddev)
            scale = spec.covariance_scale if spec.covariance_scale > 0.0 else 1.0
            msg.position_covariance = [v * scale for v in msg.position_covariance]
            msg.status.status = NavSatStatus.STATUS_FIX  # RTK lock downgraded
        elif mode == FaultSpec.MODE_BIAS_RAMP:
            # A spoof that walks the fix away along a fixed bearing. The
            # covariance stays small, so the filter has no reason to distrust it.
            offset_north = channel.bias * math.cos(math.radians(45.0))
            offset_east = channel.bias * math.sin(math.radians(45.0))

        if offset_north or offset_east:
            msg.latitude += math.degrees(offset_north / EARTH_RADIUS_M)
            msg.longitude += math.degrees(
                offset_east / (EARTH_RADIUS_M * math.cos(math.radians(msg.latitude))))
        return msg

    def _apply_imu(self, channel, mode, msg):
        if mode == FaultSpec.MODE_DROPOUT:
            return None
        spec = channel.spec
        if mode == FaultSpec.MODE_DEGRADE:
            msg.angular_velocity.z += self._random.gauss(0.0, spec.noise_stddev)
            scale = spec.covariance_scale if spec.covariance_scale > 0.0 else 1.0
            msg.angular_velocity_covariance = [v * scale for v in msg.angular_velocity_covariance]
            msg.orientation_covariance = [v * scale for v in msg.orientation_covariance]
        elif mode == FaultSpec.MODE_BIAS_RAMP:
            msg.angular_velocity.z += channel.bias
        return msg

    def _apply_odometry(self, channel, mode, msg):
        # Dropping the topic must not drop the transform: the filter needs an
        # unbroken odom -> base_link chain to publish map -> odom at all.
        if mode == FaultSpec.MODE_DROPOUT:
            return None
        spec = channel.spec
        if mode == FaultSpec.MODE_DEGRADE:
            msg.twist.twist.linear.x += self._random.gauss(0.0, spec.noise_stddev)
            msg.twist.twist.angular.z += self._random.gauss(0.0, spec.noise_stddev)
            scale = spec.covariance_scale if spec.covariance_scale > 0.0 else 1.0
            msg.twist.covariance = [v * scale for v in msg.twist.covariance]
        elif mode == FaultSpec.MODE_SLIP:
            msg.twist.twist.linear.x *= spec.slip_ratio
            msg.twist.twist.angular.z *= spec.slip_ratio
        return msg

    def _apply_twist(self, channel, mode, msg):
        if mode == FaultSpec.MODE_DROPOUT:
            return None
        spec = channel.spec
        if mode == FaultSpec.MODE_DEGRADE:
            msg.twist.twist.linear.x += self._random.gauss(0.0, spec.noise_stddev)
            scale = spec.covariance_scale if spec.covariance_scale > 0.0 else 1.0
            msg.twist.covariance = [v * scale for v in msg.twist.covariance]
        elif mode == FaultSpec.MODE_BIAS_RAMP:
            # A radar that starts reading off the crop canopy instead of the
            # ground reports a speed that is wrong but still confident.
            msg.twist.twist.linear.x += channel.bias
        return msg

    def _broadcast_tf(self, channel, msg):
        if msg is not None:
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x = msg.pose.pose.position.x
            transform.transform.translation.y = msg.pose.pose.position.y
            transform.transform.rotation = msg.pose.pose.orientation
            channel.last_transform = transform
        elif channel.last_transform is not None:
            transform = channel.last_transform
            transform.header.stamp = self.get_clock().now().to_msg()
        else:
            return
        self._tf.sendTransform(transform)


def main():
    rclpy.init()
    node = FaultInjector()
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
