#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Draw the geofence the machine is not allowed to leave.

A keepout mask is an occupancy grid like any other, so it is easier to derive
it from the survey than to draw it by hand: the orchard block is known, the
headlands it needs at either end are a consequence of the turning radius, and
everything else is off limits.

Black is forbidden, white is allowed. Run it from anywhere; it writes beside
itself.
"""
import os

# The orchard, from Main.prefab: eighteen rows, 3.5 m apart, the first at
# x = -63.25, running from the mouth at y = 42 to the far end at y = -1.
FIRST_ROW_X = -63.25
ROW_PITCH = 3.5
ROWS = 18
ROW_MOUTH_Y = 42.0
ROW_END_Y = -1.0

# A u-turn onto a row two across sweeps 5.5 m beyond the row end and the
# machine is 3.1 m long, so ten metres of headland is the working minimum.
HEADLAND = 10.0
# Half a row pitch past the outermost trunks.
SHOULDER = ROW_PITCH / 2.0

RESOLUTION = 0.5
MARGIN = 25.0


def main():
    allowed = (
        FIRST_ROW_X - SHOULDER,
        ROW_END_Y - HEADLAND,
        FIRST_ROW_X + (ROWS - 1) * ROW_PITCH + SHOULDER,
        ROW_MOUTH_Y + HEADLAND,
    )
    origin = (allowed[0] - MARGIN, allowed[1] - MARGIN)
    width = int(round((allowed[2] - allowed[0] + 2 * MARGIN) / RESOLUTION))
    height = int(round((allowed[3] - allowed[1] + 2 * MARGIN) / RESOLUTION))

    rows = []
    for row in range(height):
        # PGM starts at the top, the map origin is the bottom-left corner.
        y = origin[1] + (height - 1 - row + 0.5) * RESOLUTION
        inside_y = allowed[1] <= y <= allowed[3]
        line = bytearray(width)
        for column in range(width):
            x = origin[0] + (column + 0.5) * RESOLUTION
            inside = inside_y and allowed[0] <= x <= allowed[2]
            line[column] = 255 if inside else 0
        rows.append(bytes(line))

    here = os.path.dirname(os.path.abspath(__file__))
    image = 'orchard_keepout.pgm'
    with open(os.path.join(here, image), 'wb') as handle:
        handle.write(b'P5\n%d %d\n255\n' % (width, height))
        for line in rows:
            handle.write(line)

    with open(os.path.join(here, 'orchard_keepout.yaml'), 'w') as handle:
        handle.write(
            'image: %s\n'
            'resolution: %s\n'
            'origin: [%s, %s, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
            'mode: trinary\n'
            % (image, RESOLUTION, origin[0], origin[1]))

    print('%dx%d cells at %.2f m, origin %s' % (width, height, RESOLUTION, (origin,)))
    print('allowed x %.2f..%.2f, y %.2f..%.2f' % (allowed[0], allowed[2], allowed[1], allowed[3]))


if __name__ == '__main__':
    main()
