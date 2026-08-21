# SPDX-License-Identifier: BSD-3-Clause
"""Drive every aisle of the orchard, turning at the headlands in between.

The mission is a list of legs alternating between a row and a turn. Rows are
sent to Nav2 as ordinary NavigateToPose goals; turns are not sent anywhere,
because a headland turn is geometry rather than navigation.

Rows are not taken in order. A turn between antiparallel rows only fits without
reversing when they are at least a turning diameter apart, and the Kraken turns
on 2.61 m against a 3.5 m row spacing, so neighbours do not qualify. Taking
every second row on the way out and the ones left over on the way back makes
the offset 7 m, which does, and costs no extra distance because the return pass
was going to be driven anyway. The skip is not written down here: it is
computed from the turning radius and the row spacing, so a different machine or
a different orchard gets its own answer. Exactly one turn, where the outward
pass meets the return pass, is still between neighbours; no order of the aisles
avoids it, so it is given the deeper of the two headlands.

Rows are Nav2's work and turns are not. In a row the costmap sees trees the map
never knew about and the planner has to route around them, which is what Nav2
is for. A headland turn has no such freedom: the turning radius, the row
spacing and the depth of the headland between them leave one manoeuvre, and
asking a sampling controller to rediscover it every 50 ms produced a turn that
reversed five times, moved the steering 0.3 rad between consecutive commands,
reversed the steering about seven times a second and took four minutes. So the
turn is chosen as geometry (headland.py), driven along its own arc at a fixed
steering angle and a fixed speed, and corrected only for the difference between
the arithmetic and the ground.

How much headland there is decides which manoeuvre is legal, so it is measured
off the costmap at the row end rather than assumed. The two headlands here are
not alike -- one is about 5 m deep before the next block of trees, the other
nearly 20 -- and the same turn does not fit in both.

Aisle 0 is the one the robot is standing in and aisle numbers increase to its
left. The mission is written in that row-aligned frame -- x down the row, y
across the aisles -- and anchored once, at the start, to wherever the robot
then is in the map frame. That keeps the orchard's geometry in the terms it is
actually known in (a pitch between rows, a length along one) while letting the
map frame be whatever localisation makes it: welded to odom, or GNSS aligned
with east and north, in which case the rows lie at some arbitrary angle to the
axes and none of the numbers below would otherwise mean anything.

Anchoring is done once rather than per leg on purpose. Re-anchoring to the
current pose would fold every metre of tracking error into the definition of
where the next row is, and the mission would walk sideways out of the orchard.

The anchor takes its position from localisation and its heading from a
parameter, because those two are not equally knowable. Standing still, GNSS
fixes the machine to a few centimetres but says nothing at all about which way
it is pointing, and this simulator's IMU reports yaw relative to wherever the
robot started rather than to north. The filter's heading is therefore worth
nothing until the machine has moved a few metres: measured at the moment the
mission starts it read 0 where the rows actually run at -90, and the first leg
was aimed 46 m sideways across the orchard. A row direction is surveyed
knowledge about a field, the same kind of thing as the spacing between its
rows, so it is given here rather than guessed from a stationary sensor.
"""
import math
import time
from collections import namedtuple

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from kraken_scenarios import headland

Row = namedtuple('Row', 'what x y heading')
Turn = namedtuple('Turn', 'what row_y')


def aisle_order(count, skip):
    """Aisle order that leaves no aisle unvisited and almost no turn too tight.

    One pass per remainder class: aisles 0, skip, 2*skip ... on the way out,
    then the ones in between on the way back, each pass running opposite to the
    last so the machine ends each pass where the next one starts. Every turn
    inside a pass is a `skip` offset; only the junction between two passes is
    between neighbours.
    """
    order = []
    for offset in range(skip):
        block = list(range(offset, count, skip))
        order.extend(block if offset % 2 == 0 else block[::-1])
    return order


class OrchardCoverage(Node):

    def __init__(self):
        super().__init__('orchard_coverage')

        self.declare_parameter('aisles', 4)
        self.declare_parameter('aisle_pitch', 3.5)
        # Zero asks for the skip that makes every turn but one a plain u-turn,
        # worked out from the turning radius and the row spacing.
        self.declare_parameter('aisle_skip', 0)
        # Where a row leg finishes. Short of the trees that close the row, so
        # the goal is somewhere the robot can actually stand.
        self.declare_parameter('row_near_x', 3.0)
        self.declare_parameter('row_far_x', 46.0)
        self.declare_parameter('leg_timeout', 400.0)
        # The machine, as the turn geometry needs it: what it can steer to, and
        # how much of it swings when it does. The footprint is the one the
        # costmap is given, with the rear axle at the origin.
        self.declare_parameter('wheelbase', 2.2)
        self.declare_parameter('max_steering_angle', 0.7)
        self.declare_parameter('front_overhang', 2.5)
        self.declare_parameter('rear_overhang', 0.6)
        self.declare_parameter('half_width', 0.45)
        self.declare_parameter('turn_speed', 0.4)
        # Straight out of the row before anything swings, so the back of the
        # machine is past the last trunks before the wheels go over.
        self.declare_parameter('turn_entry', 0.8)
        # How deep the headland is taken to be. Zero measures it off the
        # costmap at the row end, which is the only honest source: the two
        # headlands here differ by 15 m.
        self.declare_parameter('headland_depth', 0.0)
        # How much of the steering left over at the chosen radius may be spent
        # correcting. The turn is meant to be driven, not searched for.
        self.declare_parameter('correction_share', 0.3)
        # Which way the rows run, in the map frame. The orchard's 18 row spawn
        # points all face -90 degrees, so this is a property of the level, not
        # of the robot. Leave it non-finite to fall back on whatever heading
        # localisation reports, which is only meaningful once moving.
        self.declare_parameter('row_heading_deg', -90.0)

        self._pitch = self.get_parameter('aisle_pitch').value
        self._near_x = self.get_parameter('row_near_x').value
        self._far_x = self.get_parameter('row_far_x').value
        self._timeout = self.get_parameter('leg_timeout').value
        self._speed = self.get_parameter('turn_speed').value
        self._entry = self.get_parameter('turn_entry').value
        self._share = self.get_parameter('correction_share').value

        self._wheelbase = self.get_parameter('wheelbase').value
        self._min_radius = self._wheelbase / math.tan(
            self.get_parameter('max_steering_angle').value)
        front = self.get_parameter('front_overhang').value
        rear = -self.get_parameter('rear_overhang').value
        side = self.get_parameter('half_width').value
        self._footprint = [(front, side), (front, -side), (rear, -side), (rear, side)]
        self._half_width = side

        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._controller = self.create_publisher(String, 'controller_selector', latched)
        self._goal_checker = self.create_publisher(String, 'goal_checker_selector', latched)
        self._cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        costmap_qos = QoSProfile(depth=1)
        costmap_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        costmap_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._costmap = None
        self.create_subscription(OccupancyGrid, 'global_costmap/costmap',
                                 self._on_costmap, costmap_qos)

        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer, self)
        self._base = (self.get_namespace().strip('/') + '/base_link').lstrip('/')
        self._anchor = None

        skip = self.get_parameter('aisle_skip').value
        if skip < 1:
            skip = headland.skip_for(self._min_radius, self._pitch)
            self.get_logger().info(
                'turning radius %.2f m over %.2f m rows: skipping %d, so a turn '
                'crosses %.1f m and never has to reverse'
                % (self._min_radius, self._pitch, skip, skip * self._pitch))
        self._order = aisle_order(self.get_parameter('aisles').value, skip)

    def _on_costmap(self, grid):
        self._costmap = grid

    def anchor(self, timeout=30.0):
        """Pin the row-aligned mission frame to where the robot is standing.

        Waiting is timed on the wall clock, not the node's. Under simulated
        time the clock reads zero until the first /clock arrives and then jumps
        to whatever the simulator has already counted, which retires a deadline
        set from it before the loop has run once.
        """
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            here = self._pose()
            if here is None:
                continue
            x, y, yaw = here
            given = self.get_parameter('row_heading_deg').value
            heading = math.radians(given) if math.isfinite(given) else yaw
            self._anchor = (x, y, heading)
            self.get_logger().info(
                'rows anchored at map (%.1f, %.1f) heading %+.0f deg '
                '(localisation reported %+.0f)'
                % (x, y, math.degrees(heading), math.degrees(yaw)))
            return True
        self.get_logger().error('no map -> %s transform; is localisation up?' % self._base)
        return False

    def _pose(self):
        try:
            tf = self._buffer.lookup_transform('map', self._base, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, math.atan2(2.0 * (r.w * r.z + r.x * r.y),
                                    1.0 - 2.0 * (r.y * r.y + r.z * r.z))

    def _to_map(self, x, y, heading):
        ax, ay, ayaw = self._anchor
        c, s = math.cos(ayaw), math.sin(ayaw)
        return ax + c * x - s * y, ay + s * x + c * y, ayaw + heading

    def plan(self):
        """The whole mission: a row leg per aisle, and the turn between each pair."""
        legs, outbound = [], True
        for index, aisle in enumerate(self._order):
            y = aisle * self._pitch
            heading = 0.0 if outbound else math.pi
            end_x = self._far_x if outbound else self._near_x
            legs.append(Row('aisle %d' % aisle, end_x, y, heading))

            if index + 1 < len(self._order):
                legs.append(Turn('aisle %d to %d' % (aisle, self._order[index + 1]),
                                 self._order[index + 1] * self._pitch))
                outbound = not outbound
        return legs

    def _send(self, x, y, heading):
        self._controller.publish(String(data='FollowPath'))
        self._goal_checker.publish(String(data='general_goal_checker'))

        x, y, heading = self._to_map(x, y, heading)
        here = self._pose()
        if here is not None:
            self.get_logger().info('  from map: (%.2f, %.2f) facing %+.0f deg'
                                   % (here[0], here[1], math.degrees(here[2])))
        self.get_logger().info('  goal in map: (%.2f, %.2f) facing %+.0f deg'
                               % (x, y, math.degrees(heading)))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        sent = self._nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, sent, timeout_sec=20.0)
        handle = sent.result()
        if handle is None:
            self.get_logger().warn('no answer to the goal request')
            return False
        if not handle.accepted:
            self.get_logger().warn('goal rejected')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=self._timeout)
        if not result.done():
            self.get_logger().warn('leg ran past %.0f s; cancelling' % self._timeout)
            rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=10.0)
            return False
        status = result.result().status
        if status != 4:
            self.get_logger().warn('navigation ended with status %d' % status)
        return status == 4

    def _free_depth(self, pose, limit=25.0):
        """How far the machine could run straight on before something stopped it.

        Read off the global costmap along the way the robot is pointing, in a
        corridor its own width; taken at a row end, that is the depth of the
        headland. Unknown ground counts as free, because the far side of a
        headland is often past anything the lidar has swept and calling that an
        obstacle would rule out every turn that has not already been driven.
        """
        grid = self._costmap
        if grid is None:
            return None
        x, y, heading = pose
        c, s = math.cos(heading), math.sin(heading)
        info = grid.info
        distance = 0.0
        while distance < limit:
            for lateral in (-self._half_width, 0.0, self._half_width):
                px = x + c * distance - s * lateral
                py = y + s * distance + c * lateral
                col = int((px - info.origin.position.x) / info.resolution)
                row = int((py - info.origin.position.y) / info.resolution)
                if not (0 <= col < info.width and 0 <= row < info.height):
                    return distance
                if grid.data[row * info.width + col] >= 99:
                    return distance
            distance += info.resolution
        return limit

    def turn(self, row_y):
        """Choose a headland manoeuvre for the room there is, and drive it.

        The turn is aimed at where the next row actually is rather than at a
        fixed offset from the machine. A turn is a relative manoeuvre, so any
        drift left over from the row it just drove would otherwise be carried
        into the next one and the one after that; measuring the offset against
        the anchored row spacing spends the turn correcting it instead.
        """
        here = self._pose()
        if here is None:
            self.get_logger().warn('no pose to turn from')
            return False

        _, across_rows, heading = self._from_map(here)
        facing = 1.0 if math.cos(heading) > 0.0 else -1.0
        offset = (row_y - across_rows) * facing

        given = self.get_parameter('headland_depth').value
        depth = given if given > 0.0 else self._free_depth(here)
        if depth is None:
            self.get_logger().warn('no costmap, so no idea how deep the headland is')
            return False

        chosen = headland.plan_turn(offset, self._min_radius, depth - self._entry,
                                    self._footprint)
        if chosen is None:
            self.get_logger().warn(
                'headland measures %.1f m; no turn onto a row %.2f m across fits '
                'in it, not even a three-point one' % (depth, abs(offset)))
            return False

        length = self._entry + sum(segment.length for segment in chosen.segments)
        self.get_logger().info(
            '  %s %.2f m to its %s in %.1f m of headland, needing %.1f m: radius '
            '%.2f m at %.2f rad of steering, %.1f m of path, about %.0f s'
            % (chosen.name, abs(offset), 'left' if offset > 0 else 'right', depth,
               self._entry + chosen.depth, chosen.radius,
               math.atan(self._wheelbase / chosen.radius), length, length / self._speed))

        # A turn changes which row the machine is in, not how far along it is.
        # A bulb ends six metres deeper into the headland than it started, and a
        # row leg planned from out there makes Smac fan out across open ground
        # until it exhausts its iterations; coming back to the row mouth first
        # puts the planner's start in the corridor, where it belongs.
        manoeuvre = headland.trace(chosen.segments)
        segments = ([headland.Segment(0.0, self._entry, False)]
                    + list(chosen.segments)
                    + [headland.Segment(0.0, max(0.0, self._entry + manoeuvre[-1].x), False)])
        return self._follow(headland.trace(segments), here)

    def _from_map(self, pose):
        """A map pose in the row-aligned mission frame: along a row, across them."""
        ax, ay, aheading = self._anchor
        c, s = math.cos(aheading), math.sin(aheading)
        dx, dy = pose[0] - ax, pose[1] - ay
        heading = pose[2] - aheading
        return (c * dx + s * dy, -s * dx + c * dy,
                math.atan2(math.sin(heading), math.cos(heading)))

    def _follow(self, path, origin):
        """Drive a traced path at fixed speed and fixed steering per segment.

        The steering each segment wants is already known, so it is commanded
        outright and the feedback only trims it. The trim is capped at a share
        of the steering left unused at the chosen radius, which is what stops a
        disagreement with localisation turning into the wheel-sawing this
        replaced: the machine drives the arc it planned, or it stops.
        """
        ox, oy, oheading = origin
        c, s = math.cos(oheading), math.sin(oheading)
        path = [headland.Pose(ox + c * p.x - s * p.y, oy + s * p.x + c * p.y,
                              oheading + p.heading, p.curvature, p.reverse)
                for p in path]

        limit = 1.0 / self._min_radius
        allowance = self._share * limit
        index, worst, steering = 0, 0.0, []
        period = 0.05
        due = time.monotonic()
        deadline = time.monotonic() + self._timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now < due:
                continue
            due = now + period
            here = self._pose()
            if here is None:
                continue

            index = self._advance(path, index, here)
            if index >= len(path) - 1:
                self._halt()
                moves = [abs(b - a) for a, b in zip(steering, steering[1:])]
                self.get_logger().info(
                    '  turn driven: %.2f m off the arc at worst, steering moved at '
                    'most %.3f rad and on average %.3f rad between commands'
                    % (worst, max(moves) if moves else 0.0,
                       sum(moves) / len(moves) if moves else 0.0))
                return True

            target = path[index]
            across = (-(here[0] - target.x) * math.sin(target.heading)
                      + (here[1] - target.y) * math.cos(target.heading))
            drift = math.atan2(math.sin(here[2] - target.heading),
                               math.cos(here[2] - target.heading))
            worst = max(worst, abs(across))
            if abs(across) > 0.6:
                self._halt()
                self.get_logger().warn('%.2f m off the turn it was driving; stopping'
                                       % across)
                return False

            # Backing along an arc reverses which way the steering has to go to
            # close a lateral gap; the heading term is unaffected.
            sideways = -across if target.reverse else across
            trim = max(-allowance, min(allowance, -(0.8 * drift + 0.6 * sideways)))
            curvature = max(-limit, min(limit, target.curvature + trim))
            speed = -self._speed if target.reverse else self._speed

            command = Twist()
            command.linear.x = speed
            command.angular.z = speed * curvature
            self._cmd.publish(command)
            steering.append(math.atan(self._wheelbase * curvature))

        self._halt()
        self.get_logger().warn('turn ran past %.0f s' % self._timeout)
        return False

    @staticmethod
    def _advance(path, index, here):
        """Walk the index forward past every point the machine has already passed."""
        while index < len(path) - 1:
            point = path[index]
            ahead = ((here[0] - point.x) * math.cos(point.heading)
                     + (here[1] - point.y) * math.sin(point.heading))
            if point.reverse:
                ahead = -ahead
            if ahead <= 0.0:
                break
            index += 1
        return index

    def _halt(self):
        for _ in range(3):
            self._cmd.publish(Twist())

    def run(self):
        if not self._nav.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('navigate_to_pose never appeared; is Nav2 up?')
            return False
        if not self.anchor():
            return False

        legs = self.plan()
        failed = []
        for number, leg in enumerate(legs, start=1):
            if isinstance(leg, Row):
                what = 'row %s' % leg.what
                self.get_logger().info(
                    'leg %d/%d: %s to (%.1f, %.1f) facing %+.0f deg'
                    % (number, len(legs), what, leg.x, leg.y, math.degrees(leg.heading)))
                done = self._send(leg.x, leg.y, leg.heading)
            else:
                what = 'turn %s' % leg.what
                self.get_logger().info('leg %d/%d: %s' % (number, len(legs), what))
                done = self.turn(leg.row_y)
            if not done:
                # One aisle the machine could not reach is a gap in the day's
                # work, not a reason to abandon the rest of the orchard.
                self.get_logger().warn('leg %d failed: %s' % (number, what))
                failed.append(what)

        if failed:
            self.get_logger().warn('%d of %d legs failed: %s'
                                   % (len(failed), len(legs), ', '.join(failed)))
        else:
            self.get_logger().info('covered %d aisles, all %d legs succeeded'
                                   % (len(self._order), len(legs)))
        return not failed


def main():
    rclpy.init()
    node = OrchardCoverage()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
