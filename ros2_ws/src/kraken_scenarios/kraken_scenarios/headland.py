# SPDX-License-Identifier: BSD-3-Clause
"""Headland turns worked out as geometry, before the machine moves.

A headland turn is not a search problem. The vehicle has one turning radius,
the rows have one spacing and the headland has one depth; between them they
decide which manoeuvre is possible, and once it is chosen there is nothing left
to decide while driving. That is the whole point of doing it here: a turn made
of constant-curvature segments can be driven at a fixed steering angle and a
fixed speed, and the controller is left correcting the difference between the
arithmetic and the ground rather than re-deciding the manoeuvre twenty times a
second.

Three manoeuvres, in the order they are preferred:

  u-turn   a quarter circle, a straight, and a quarter circle. Needs the next
           row to be at least a turning diameter away and about one turning
           radius of headland. Shortest, shallowest, and never reverses.
  bulb     when the next row is nearer than a turning diameter, swing away from
           it first and come back round the outside. Still never reverses, but
           it costs roughly three radii of headland, so it wants a deep one.
  3-point  when the headland is too shallow for either, reversing is the only
           way round. Slowest, and the only manoeuvre that puts the machine's
           blind end towards the trees, so it is the last resort.

Depth is measured to the swept footprint rather than to the axle. The Kraken
carries 2.5 m of itself ahead of its rear axle, so its nose is already that far
into the headland before the turn starts and its outer front corner reaches
about 1.3 m further out than the radius the axle turns on. Planning on the axle
alone is how a turn that fits on paper takes the front of the machine through a
tree.

Curvature here is tan(steer)/wheelbase -- the circle the wheels are set to, not
the circle the body traces. It keeps its sign when the machine backs along it,
which is what makes a three-point turn expressible as three arcs.
"""
import math
from collections import namedtuple

Segment = namedtuple('Segment', 'curvature length reverse')
Pose = namedtuple('Pose', 'x y heading curvature reverse')
Turn = namedtuple('Turn', 'name segments radius depth')


def trace(segments, step=0.05):
    """Sample a segment list into poses, starting at the origin heading +x."""
    x = y = heading = 0.0
    path = [Pose(0.0, 0.0, 0.0, segments[0].curvature, segments[0].reverse)]
    for segment in segments:
        if segment.length <= 1e-6:
            continue
        count = max(1, int(math.ceil(segment.length / step)))
        ds = segment.length / count * (-1.0 if segment.reverse else 1.0)
        for _ in range(count):
            middle = heading + 0.5 * ds * segment.curvature
            x += ds * math.cos(middle)
            y += ds * math.sin(middle)
            heading += ds * segment.curvature
            path.append(Pose(x, y, heading, segment.curvature, segment.reverse))
    return path


def extent(path, footprint):
    """How far the swept footprint reaches ahead of the start, and to each side."""
    ahead = left = right = 0.0
    for pose in path:
        c, s = math.cos(pose.heading), math.sin(pose.heading)
        for fx, fy in footprint:
            px = pose.x + c * fx - s * fy
            py = pose.y + s * fx + c * fy
            ahead = max(ahead, px)
            left = max(left, py)
            right = min(right, py)
    return ahead, left, right


def u_turn(offset, radius):
    side = math.copysign(1.0, offset)
    quarter = radius * math.pi / 2.0
    return [Segment(side / radius, quarter, False),
            Segment(0.0, abs(offset) - 2.0 * radius, False),
            Segment(side / radius, quarter, False)]


def bulb_turn(offset, radius):
    side = math.copysign(1.0, offset)
    gamma = math.acos(min(1.0, abs(offset) / (2.0 * radius)))
    return [Segment(-side / radius, radius * gamma, False),
            Segment(side / radius, radius * (math.pi + gamma), False)]


def fishtail(offset, radius, first, back):
    """Forward arc, reverse arc, forward arc; the three points of a three-point turn.

    All three turn the machine the same way. The middle one does it backwards,
    which is why its wheels are set the other way.
    """
    side = math.copysign(1.0, offset)
    last = math.pi - first - back
    return [Segment(side / radius, radius * first, False),
            Segment(-side / radius, radius * back, True),
            Segment(side / radius, radius * last, False)]


def _landing(offset, radius, first, back):
    return trace(fishtail(offset, radius, first, back), step=0.2)[-1].y


def three_point_turn(offset, radius, footprint, depth=None, step=0.05):
    """The best three-point turn that lands the next row, for the depth given.

    The first and last arcs trade against each other, so the pair is searched:
    for each first arc the reverse arc that lands the row is found by
    bisection. Of the solutions that fit, the one with the longest first arc
    wins rather than the shallowest. The shallowest is always the degenerate
    one that barely turns before backing up, and it lands the row a metre out
    if the machine did not start exactly where it thought; spending spare
    headland on a balanced manoeuvre buys that back. With no depth to spend,
    the shallowest is all there is.
    """
    found = []
    angles = [0.1 * n for n in range(1, 31)]
    for first in angles:
        previous = None
        bracket = None
        for back in angles:
            if first + back >= math.pi:
                break
            error = _landing(offset, radius, first, back) - offset
            if previous is not None and previous[1] * error <= 0.0:
                bracket = (previous[0], back)
                break
            previous = (back, error)
        if bracket is None:
            continue
        low, high = bracket
        low_error = _landing(offset, radius, first, low) - offset
        for _ in range(24):
            middle = 0.5 * (low + high)
            if (_landing(offset, radius, first, middle) - offset) * low_error <= 0.0:
                high = middle
            else:
                low, low_error = middle, _landing(offset, radius, first, middle) - offset
        segments = fishtail(offset, radius, first, 0.5 * (low + high))
        found.append(Turn('3-point', segments, radius,
                          extent(trace(segments, step), footprint)[0]))

    if not found:
        return None
    fits = [turn for turn in found if depth is not None and turn.depth <= depth]
    if fits:
        return fits[-1]
    return min(found, key=lambda turn: turn.depth)


def _radii(most, least, step=0.05):
    """Radii from the roomiest down to the tightest, widest first."""
    count = max(0, int((most - least) / step))
    return [most - n * step for n in range(count)] + [least]


def plan_turn(offset, min_radius, depth, footprint, step=0.05):
    """Choose the turn into a row `offset` across, given the headland available.

    Every manoeuvre is tried from the roomiest radius down to the tightest, and
    the first that fits the headland wins. Radius is taken as large as the
    ground allows rather than as small as the machine can manage, because a
    turn driven at full lock has nothing left to correct with: every trim can
    only widen it, so the first metre of drift is unrecoverable and the
    manoeuvre becomes a one-way bet on the arithmetic. Backing off a third of
    the way leaves about a fifth of the steering in hand, which is what the
    tracker trims with. This is not a refinement -- driven at full lock the
    bulb turn below ran 0.6 m wide and had to be abandoned.
    """
    roomy = 1.35 * min_radius

    if abs(offset) >= 2.0 * min_radius:
        for radius in _radii(min(0.5 * abs(offset), roomy), min_radius):
            segments = u_turn(offset, radius)
            reach = extent(trace(segments, step), footprint)[0]
            if reach <= depth:
                return Turn('u-turn', segments, radius, reach)

    for radius in _radii(roomy, min_radius):
        segments = bulb_turn(offset, radius)
        reach = extent(trace(segments, step), footprint)[0]
        if reach <= depth:
            return Turn('bulb', segments, radius, reach)

    for radius in _radii(roomy, min_radius, 0.2):
        turn = three_point_turn(offset, radius, footprint, depth, step)
        if turn is not None and turn.depth <= depth:
            return turn
    return None


def skip_for(min_radius, pitch):
    """How many rows to leave between passes so a turn need never reverse.

    A turn between antiparallel rows fits without reversing when the rows are
    at least a turning diameter apart, so this is the diameter measured in row
    spacings, rounded up. Taking every skip'th row on the way out and the ones
    left over on the way back covers the block either way; what the skip buys
    is that every turn but one is a plain u-turn.
    """
    return max(1, int(math.ceil(2.0 * min_radius / pitch)))
