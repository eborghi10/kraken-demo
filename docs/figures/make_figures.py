#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Draw the headland geometry the planner actually uses.

This mirrors `kraken_nav/src/headland.cpp` segment for segment, and asserts its
own output against the reference table in `test/test_headland.cpp` before it
draws anything. If the C++ changes and this does not, the assertions fail and
the figures do not silently go stale.

    python3 docs/figures/make_figures.py
"""
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

WHEELBASE = 2.2
MAX_STEER = 0.7
MIN_RADIUS = WHEELBASE / math.tan(MAX_STEER)
FOOTPRINT = [(2.5, 0.45), (2.5, -0.45), (-0.6, -0.45), (-0.6, 0.45)]
PITCH = 3.5
ROOMY = 1.35 * MIN_RADIUS

INK = '#1b2a41'
PATH = '#0b6e4f'
SWEEP = '#c0c8d1'
BLOCK = '#b3402f'


def trace(segments, step=0.05):
    x = y = heading = 0.0
    out = [(0.0, 0.0, 0.0)]
    for curvature, length, reverse in segments:
        if length <= 1e-6:
            continue
        count = max(1, int(math.ceil(length / step)))
        ds = length / count * (-1.0 if reverse else 1.0)
        for _ in range(count):
            middle = heading + 0.5 * ds * curvature
            x += ds * math.cos(middle)
            y += ds * math.sin(middle)
            heading += ds * curvature
            out.append((x, y, heading))
    return out


def reach(path):
    return max(x + math.cos(h) * fx - math.sin(h) * fy
               for x, _, h in path for fx, fy in FOOTPRINT)


def u_turn(offset, radius):
    side = math.copysign(1.0, offset)
    quarter = radius * math.pi / 2.0
    return [(side / radius, quarter, False),
            (0.0, abs(offset) - 2.0 * radius, False),
            (side / radius, quarter, False)]


def bulb_turn(offset, radius):
    side = math.copysign(1.0, offset)
    gamma = math.acos(min(1.0, abs(offset) / (2.0 * radius)))
    return [(-side / radius, radius * gamma, False),
            (side / radius, radius * (math.pi + gamma), False)]


def fishtail(offset, radius, first, back):
    side = math.copysign(1.0, offset)
    last = math.pi - first - back
    return [(side / radius, radius * first, False),
            (-side / radius, radius * back, True),
            (side / radius, radius * last, False)]


def three_point(offset, radius, depth, step=0.05):
    """Longest first arc that still fits, exactly as threePointTurn picks it."""
    def landing(first, back):
        return trace(fishtail(offset, radius, first, back), 0.2)[-1][1]

    found = []
    for n in range(1, 31):
        first = 0.1 * n
        low = high = previous_back = previous_error = 0.0
        bracketed = have_previous = False
        for m in range(1, 31):
            back = 0.1 * m
            if first + back >= math.pi:
                break
            error = landing(first, back) - offset
            if have_previous and previous_error * error <= 0.0:
                low, high, bracketed = previous_back, back, True
                break
            previous_back, previous_error, have_previous = back, error, True
        if not bracketed:
            continue
        low_error = landing(first, low) - offset
        for _ in range(24):
            middle = 0.5 * (low + high)
            error = landing(first, middle) - offset
            if error * low_error <= 0.0:
                high = middle
            else:
                low, low_error = middle, error
        segments = fishtail(offset, radius, first, 0.5 * (low + high))
        found.append((segments, reach(trace(segments, step))))
    if not found:
        return None
    for segments, needed in reversed(found):
        if needed <= depth:
            return segments, needed
    return min(found, key=lambda item: item[1])


def radii(most, least, step=0.05):
    count = max(0, int((most - least) / step))
    return [most - n * step for n in range(count)] + [least]


def plan_turn(offset, depth, step=0.05):
    if abs(offset) >= 2.0 * MIN_RADIUS:
        for radius in radii(min(0.5 * abs(offset), ROOMY), MIN_RADIUS):
            segments = u_turn(offset, radius)
            needed = reach(trace(segments, step))
            if needed <= depth:
                return 'u-turn', segments, radius, needed
    for radius in radii(ROOMY, MIN_RADIUS):
        segments = bulb_turn(offset, radius)
        needed = reach(trace(segments, step))
        if needed <= depth:
            return 'bulb', segments, radius, needed
    for radius in radii(ROOMY, MIN_RADIUS, 0.2):
        picked = three_point(offset, radius, depth, step)
        if picked is None:
            continue
        segments, needed = picked
        if needed <= depth:
            return '3-point', segments, radius, needed
    return None


def check():
    """Against test_headland.cpp, to six decimals."""
    expected = [
        (7.0, 6.4, 'u-turn', 3.500000, 4.674579),
        (3.5, 17.4, 'bulb', 3.526108, 10.819108),
        (3.5, 4.2, '3-point', 3.526108, 4.100218),
        (7.0, 3.2, None, 0, 0),
    ]
    for offset, depth, name, radius, needed in expected:
        got = plan_turn(offset, depth)
        if name is None:
            assert got is None, 'expected no turn for %.1f/%.1f, got %s' % (offset, depth, got)
            continue
        assert got is not None, 'expected %s for %.1f/%.1f' % (name, offset, depth)
        assert got[0] == name, '%s != %s' % (got[0], name)
        assert abs(got[2] - radius) < 1e-6, '%r != %r' % (got[2], radius)
        assert abs(got[3] - needed) < 1e-6, '%r != %r' % (got[3], needed)
    print('geometry agrees with test_headland.cpp')


def footprint_at(x, y, heading):
    c, s = math.cos(heading), math.sin(heading)
    pts = [(x + c * fx - s * fy, y + s * fx + c * fy) for fx, fy in FOOTPRINT]
    return pts + [pts[0]]


def draw_turn(axis, title, offset, depth):
    name, segments, radius, needed = plan_turn(offset, depth)
    path = trace(segments)
    entry = 0.8

    # The machine drives `entry` metres out of the row before it turns, and the
    # depth it needs is measured from where the trees stop.
    for x, y, heading in path[::28]:
        pts = footprint_at(x + entry, y, heading)
        axis.plot([p[0] for p in pts], [p[1] for p in pts], color=SWEEP, lw=0.6, zorder=1)
    axis.plot([entry + p[0] for p in path], [p[1] for p in path],
              color=PATH, lw=2.0, zorder=3, label='rear axle')

    axis.axvline(0.0, color=INK, lw=1.4)
    axis.axvline(depth, color=BLOCK, lw=1.2, ls='--')
    axis.text(depth, -1.4, ' headland %.1f m' % depth, color=BLOCK, fontsize=7,
              rotation=90, va='bottom')
    axis.axvline(entry + needed, color=PATH, lw=1.0, ls=':')
    axis.text(entry + needed, -1.4, ' reach %.1f m' % (entry + needed), color=PATH,
              fontsize=7, rotation=90, va='bottom')

    for row in (0.0, offset):
        axis.plot([-11, 0], [row, row], color=INK, lw=6, alpha=0.18, solid_capstyle='butt')
    axis.annotate('', xy=(-9.0, offset), xytext=(-9.0, 0.0),
                  arrowprops=dict(arrowstyle='<->', color=INK, lw=0.9))
    axis.text(-8.7, 0.5 * offset, '%.1f m' % abs(offset), fontsize=7, color=INK)

    axis.set_title('%s\n%s, R = %.2f m' % (title, name, radius), fontsize=9)
    axis.set_aspect('equal')
    axis.set_xlim(-11, max(13, depth + 2))
    axis.set_ylim(min(-3.0, offset - 3.0), max(3.0, offset + 3.0))
    axis.set_xlabel('along the row, out of the mouth (m)', fontsize=7)
    axis.tick_params(labelsize=6)
    for side in ('top', 'right'):
        axis.spines[side].set_visible(False)


def figure_turns(out):
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 11.0))
    draw_turn(axes[0], 'Two rows across, deep headland', 2 * PITCH, 8.0)
    draw_turn(axes[1], 'One row across, deep headland', PITCH, 17.4)
    draw_turn(axes[2], 'One row across, shallow headland', PITCH, 4.2)
    axes[0].legend(fontsize=7, loc='upper left', frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'turns.svg'))
    plt.close(fig)


def aisle_order(count, skip):
    order = []
    for start in range(skip):
        one_pass = list(range(start, count, skip))
        if start % 2:
            one_pass.reverse()
        order += one_pass
    return order


def figure_order(out):
    count, skip = 18, math.ceil(2 * MIN_RADIUS / PITCH)
    order = aisle_order(count, skip)
    fig, axis = plt.subplots(figsize=(9.0, 4.4))

    near, far = 3.0, 43.0
    for aisle in range(count):
        axis.plot([aisle * PITCH, aisle * PITCH], [near, far],
                  color=INK, lw=5, alpha=0.10, solid_capstyle='butt')

    for n, aisle in enumerate(order):
        outbound = n % 2 == 0
        y0, y1 = (near, far) if outbound else (far, near)
        axis.annotate('', xy=(aisle * PITCH, y1), xytext=(aisle * PITCH, y0),
                      arrowprops=dict(arrowstyle='->', color=PATH, lw=1.6))
        axis.text(aisle * PITCH, near - 3.0, str(n + 1), fontsize=6.5,
                  ha='center', color=INK)
        if n + 1 < len(order):
            across = (order[n + 1] - aisle) * PITCH
            side = far if outbound else near
            bulge = 6.0 if outbound else -6.0
            axis.plot([aisle * PITCH, aisle * PITCH + 0.5 * across, order[n + 1] * PITCH],
                      [side, side + bulge, side],
                      color=BLOCK if abs(across) < 1.5 * PITCH else PATH,
                      lw=1.0, alpha=0.85)

    axis.text(0, near - 6.5, 'leg number', fontsize=7, color=INK)
    axis.set_title('Coverage order over %d aisles, skipping %d\n'
                   'red is the one neighbour-to-neighbour turn, which needs the bulb'
                   % (count, skip), fontsize=9)
    axis.set_xlabel('across the rows (m)', fontsize=8)
    axis.set_ylabel('along the rows (m)', fontsize=8)
    axis.set_aspect('equal')
    axis.tick_params(labelsize=7)
    for side in ('top', 'right'):
        axis.spines[side].set_visible(False)
    fig.tight_layout()
    fig.savefig(os.path.join(out, 'coverage_order.svg'))
    plt.close(fig)


def main():
    check()
    out = os.path.dirname(os.path.abspath(__file__))
    figure_turns(out)
    figure_order(out)
    print('min turning radius %.6f m, skip %d'
          % (MIN_RADIUS, math.ceil(2 * MIN_RADIUS / PITCH)))
    print('wrote turns.svg, coverage_order.svg')


if __name__ == '__main__':
    main()
