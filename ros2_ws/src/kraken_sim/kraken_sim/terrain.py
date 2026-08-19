# SPDX-License-Identifier: BSD-3-Clause
"""Ground traction as a function of position.

The existing `wheel_slip` scenario makes the encoders lie while the robot keeps
moving exactly as commanded. That is a sensor fault, and the filter can catch it
by comparing against any other source. This is the other kind: the wheels turn
at the commanded rate, the encoders report it honestly, and the robot does not
get there. Nothing in the message stream is wrong, so there is nothing to
cross-check, and the error is correlated with where the robot is rather than
with time.

Patches are axis-aligned rectangles over a uniform default. Edges are hard on
purpose: a step change in traction is easy to see in a plot and easy to reason
about when a run goes wrong.
"""
import json


class TractionField:
    """Traction in (0, 1]. 1.0 means the commanded velocity is achieved."""

    def __init__(self, patches=(), default=1.0):
        self._default = self._checked(default, 'default')
        self._patches = []
        for index, patch in enumerate(patches):
            where = 'patch %d' % (index,)
            x_min, x_max = self._span(patch, 'x', where)
            y_min, y_max = self._span(patch, 'y', where)
            self._patches.append(
                (x_min, x_max, y_min, y_max, self._checked(patch['traction'], where)))

    @classmethod
    def from_json(cls, text):
        """Build from a JSON string, which is how a scenario passes one through.

        ROS 2 parameters are flat and cannot carry a list of dicts, so the
        scenario yaml embeds the field and the launch file forwards it as text.
        """
        if not text:
            return cls()
        spec = json.loads(text)
        return cls(spec.get('patches', ()), spec.get('default', 1.0))

    @property
    def uniform(self):
        return not self._patches

    def traction_at(self, x, y):
        # Later patches win, so a scenario can lay a track over a background.
        value = self._default
        for x_min, x_max, y_min, y_max, traction in self._patches:
            if x_min <= x <= x_max and y_min <= y <= y_max:
                value = traction
        return value

    @staticmethod
    def _checked(value, where):
        value = float(value)
        if not 0.0 < value <= 1.0:
            raise ValueError(
                'traction for %s must be in (0, 1], got %r' % (where, value))
        return value

    @staticmethod
    def _span(patch, axis, where):
        low, high = (float(v) for v in patch[axis])
        if high < low:
            raise ValueError('%s has %s reversed: %r' % (where, axis, patch[axis]))
        return low, high
