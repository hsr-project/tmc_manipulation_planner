'''
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
'''
# !/usr/bin/env python
# -*- coding: utf-8 -*-
u"""Module that specifies the vocation point and generates a space/time orbit.

Subsection points are expressed as POINT = [0,0,0] as an array, and the number of differentiated floors is different.
Point [0] is displaced, POINT [1] is on the first floor, POINT [2] is different
"""

import bisect

import matplotlib
# Change AGG to TKAGG when Debug
matplotlib.use('Agg')

# Subsequent IMPORT is written after matplotlib.use, so pass through with noqa.
import matplotlib.pyplot as plt  # noqa

import numpy as np  # noqa

import scipy  # noqa


if scipy.version.version >= '0.18.0':
    _NEW_SCIPY = True
else:
    _NEW_SCIPY = False

if _NEW_SCIPY:
    from scipy.interpolate import CubicSpline
else:
    from nspline import NaturalCubicSpline


class TrajectoryDict(dict):
    u"""Trajectory management class."""

    def __init__(self, length, items={}):
        u"""Give the length and initialize."""
        super(TrajectoryDict, self).__init__()
        self.length = length
        for (name, traj) in items.items():
            self[name] = traj(length)

    def __call__(self, x):
        u"""Returns DICT at the interpotation."""
        if not isinstance(x, (int, float)):
            raise KeyError('must be a number')
        y = {}
        for name in self:
            y[name] = self[name](x)
        return y

    def append(self, name, traj_type):
        super(TrajectoryDict, self).__setitem__(name, traj_type(self.length))


class Trajectory(dict):
    u"""DICT class of (x0, x1, x2) with parameters as key."""

    def __init__(self, length, scale=3):
        u"""Initialize.

        Args:
           length FLOAT: Parameter range
           scale INT: Resolution up to a few points n
        """
        super(Trajectory, self).__init__()
        self.length = length
        self.scale = int(10 ** int(scale))
        self.sorted_keys = None
        self[0] = [0.0, 0.0, 0.0]

    def __setitem__(self, x, point):
        u"""Settings via points."""
        if not isinstance(x, (int, float)):
            raise KeyError('must be a number')
        if x > self.length:
            raise KeyError('must be in the length %f' % self.length)
        if not isinstance(point, (list, tuple)):
            raise ValueError('must be a tuple or list')
        key = np.round(x * float(self.scale)) / float(self.scale)
        self.sorted_keys = None
        return super(Trajectory, self).__setitem__(key, list(point))

    def __getitem__(self, x):
        u"""Acquisition of voting points."""
        key = np.round(x * float(self.scale)) / float(self.scale)
        return super(Trajectory, self).__getitem__(key)

    def __call__(self, x):
        u"""Returns the interpolation point."""
        if not isinstance(x, (int, float)):
            raise ValueError('must be a number')
        if x < 0:
            x = 0
        if x > self.length:
            raise ValueError(
                'x=%f must be in the length %f' % (x, self.length))
        if not self.sorted_keys:
            self.update()
        # Search by two -part search algorithm
        key = bisect.bisect_right(self.sorted_keys, x)
        if key:
            return list(super(Trajectory, self).__getitem__(
                self.sorted_keys[key - 1]))
        raise ValueError('invalid value %f' % x)

    def update(self):
        self.sorted_keys = sorted(self.keys())

    def calc(self, seq, step):
        u"""Calculate the interpolation point together."""
        if not self.sorted_keys:
            self.update()
        lst = [self[key] for key in self.sorted_keys]
        return [lst[int(round(x / step))] for x in seq]


class LinearTrajectory(Trajectory):
    u"""Linear interpolation."""

    def __init__(self, length, level=0):
        super(LinearTrajectory, self).__init__(length)
        self.level = level
        self.a = {}

    def __call__(self, x):
        u"""Returns the interpolation point."""
        if not isinstance(x, (int, float)):
            raise ValueError('must be a number')
        if x < 0:
            x = 0
        if x > self.length:
            raise ValueError('must be in the length %f' % self.length)
        if not self.sorted_keys:
            self.update()
        # Search by two -part search algorithm
        key = bisect.bisect_right(self.sorted_keys, x)
        if key:
            key = self.sorted_keys[key - 1]
            return [self.a[key][0] + self.a[key][1] * (x - key), self.a[key][1], 0]

    def update(self):
        u"""Calculate the interpolation parameter."""
        super(LinearTrajectory, self).update()
        # If there is only 0 via points
        if len(self.sorted_keys) == 1:
            x0 = self.sorted_keys[0]
            self.a[x0] = [self[x0][0], self[x0][1]]
            return
        for (x0, x1) in zip(self.sorted_keys, self.sorted_keys[1:]):
            if self.level == 0:
                (y0, y1) = (self[x0][0], self[x1][0])
                self.a[x0] = [y0, (y1 - y0) / (x1 - x0)]
                self[x0][1] = (y1 - y0) / (x1 - x0)
            elif self.level == 1:
                self.a[x0] = [0, self[x0][1]]
            else:
                raise(ValueError, 'Invalid interpolation level')
        (x0, x1) = (self.sorted_keys[-2], self.sorted_keys[-1])
        # Make the inclination of the end of the end to 0
        self.a[x1] = [self[x1][0], self.a[x0][1]]
        self[x1][1] = 0

    def calc(self, seq, step):
        u"""Calculate the interpolation point together."""
        if not self.sorted_keys:
            self.update()
        idx = [int(round(x / step)) for x in seq]
        if len(idx) == len(self.sorted_keys):
            keys = [self.sorted_keys[x] for x in idx]
            return [[self.a[key][0] + self.a[key][1] * (x - key),
                     self.a[key][1], 0] for x, key in zip(seq, keys)]
        else:
            return [self._interpolate(x) for x in seq]

    def _interpolate(self, x):
        u"""Linear interpolation calculation, assumption that Update () is called"""
        # IF statement for guarding the case that is 0 with bisect.bisect_left
        if x < self.sorted_keys[0]:
            return [self.a[self.sorted_keys[0]], 0, 0]
        else:
            index = bisect.bisect_left(self.sorted_keys, x) - 1
            a = self.a[self.sorted_keys[index]]
            return [a[0] + a[1] * (x - self.sorted_keys[index]), a[1], 0]


class Poly3Trajectory(Trajectory):
    u"""Tertiary polymorphism."""

    def __init__(self, length, scale=3):
        super(Poly3Trajectory, self).__init__(length, scale)
        self.a = {}

    def __call__(self, x):
        u"""Calculate the interpolation point and return it"""
        if not isinstance(x, (int, float)):
            raise ValueError('must be a number')
        if x < 0:
            x = 0
        if x > self.length:
            raise ValueError('must be in the length %f' % self.length)
        if not self.sorted_keys:
            self.update()
        for x0 in reversed(self.sorted_keys):
            if x >= x0:
                a = self.a[x0]
                x = x - x0
                y0 = a[0] + a[1] * x + a[2] * x ** 2 + a[3] * x ** 3
                y1 = a[1] + 2 * a[2] * x + 3 * a[3] * x ** 2
                y2 = 2 * a[2] + 6 * a[3] * x
                return [y0, y1, y2]

    def update(self):
        u"""Calculate the interpolation parameter."""
        super(Poly3Trajectory, self).update()
        if len(self.sorted_keys) == 1:
            x0 = self.sorted_keys[0]
            self.a[x0] = [self[x0][0], self[x0][1], self[x0][2] / 2, 0]
            return
        for (x0, x1) in zip(self.sorted_keys, self.sorted_keys[1:]):
            (p0, p1) = (self[x0], self[x1])
            (d0, v0, d1, v1) = (p0[0], p0[1], p1[0], p1[1])
            xd = (x1 - x0)
            self.a[x0] = [d0,
                          v0,
                          -(3 * d0 - 3 * d1 + (2 * v0 + v1) * xd) / (xd ** 2),
                          -(-2 * d0 + 2 * d1 - (v0 + v1) * xd) / (xd ** 3)]
        (x0, x1) = (self.sorted_keys[-2], self.sorted_keys[-1])
        # The inclination of the end is adjusted to the last
        self.a[x1] = [self[x1][0], self[x1][1], 0, 0]


class Poly5Trajectory(Trajectory):
    u"""5th interpolation orbital according to the fifth polyal"""

    def __init__(self, length, scale=3):
        super(Poly5Trajectory, self).__init__(length, scale)
        self.a = {}

    def __call__(self, x):
        u"""Calculate the interpolation point and return it"""
        if not isinstance(x, (int, float)):
            raise ValueError('must be a number')
        if x < 0:
            x = 0
        if x > self.length:
            raise ValueError('must be in the length %f' % self.length)
        if not self.sorted_keys:
            self.update()
        for x0 in reversed(self.sorted_keys):
            if x >= x0:
                a = self.a[x0]
                x = x - x0
                y0 = a[0] + a[1] * x + a[2] * x ** 2 + a[3] * x ** 3 + a[4] * x ** 4 + a[5] * x ** 5
                y1 = a[1] + 2 * a[2] * x + 3 * a[3] * x ** 2 + 4 * a[4] * x ** 3 + 5 * a[5] * x ** 4
                y2 = 2 * a[2] + 6 * a[3] * x + 12 * a[4] * x ** 2 + 20 * a[5] * x ** 3
                return [y0, y1, y2]

    def update(self):
        u"""Calculate the interpolation parameters"""
        super(Poly5Trajectory, self).update()
        if len(self.sorted_keys) == 1:
            x0 = self.sorted_keys[0]
            self.a[x0] = [self[x0][0], self[x0][1], self[x0][2] / 2, 0, 0, 0]
            return
        for (x0, x1) in zip(self.sorted_keys, self.sorted_keys[1:]):
            (p0, p1) = (self[x0], self[x1])
            (d0, v0, a0, d1, v1, a1) = (
                p0[0], p0[1], p0[2], p1[0], p1[1], p1[2])
            xd = (x1 - x0)
            self.a[x0] = [d0,
                          v0,
                          a0 / 2,
                          (20 * d1 - 20 * d0 - (8 * v1 + 12 * v0) * xd - (3 * a0 - a1) * xd ** 2) / (2 * xd ** 3),
                          (30 * d0 - 30 * d1 + (14 * v1 + 16 * v0) * xd + (3 * a0 - 2 * a1) * xd ** 2) / (2 * xd ** 4),
                          (12 * d1 - 12 * d0 - (6 * v1 + 6 * v0) * xd - (a0 - a1) * xd ** 2) / (2 * xd ** 5)]
        (x0, x1) = (self.sorted_keys[-2], self.sorted_keys[-1])
        # The inclination of the end is adjusted to the last
        self.a[x1] = [self[x1][0], self[x1][1], self[x1][2] / 2, 0, 0, 0]


class NaturalCubicSplineTrajectory(Trajectory):
    u"""3rd natural spline interpolation orbit."""

    def __init__(self, length):
        super(NaturalCubicSplineTrajectory, self).__init__(length)
        self.spline3 = None
        self.memo = {}

    def __call__(self, x):
        u"""Calculate the interpolation point and return it."""
        if x in self.memo:
            return self.memo[x]

        if not isinstance(x, (int, float)):
            raise ValueError('must be a number')
        if x < 0:
            x = 0
        if x > self.length:
            raise ValueError(
                'x=%f must be in the length %f' % (x, self.length))
        if not self.sorted_keys:
            self.update()
        [yd, yv, ya] = [self.spline3(x, i) for i in range(3)]
        self.memo[x] = (float(yd), float(yv), float(ya))
        return self.memo[x]

    def update(self):
        u"""Calculate the interpolation parameter."""
        super(NaturalCubicSplineTrajectory, self).update()

        # Some joints do not set up scores, but two are set.
        if len(self.sorted_keys) == 1:
            x = [self.sorted_keys[0], self.length]
            y = [self[x[0]][0], self[x[0]][0]]
        else:
            x = self.sorted_keys
            y = [self[i][0] for i in self.sorted_keys]
            if _NEW_SCIPY:
                self.spline3 = CubicSpline(np.array(x, dtype=np.float64),
                                           np.array(y, dtype=np.float64),
                                           bc_type='natural')
            else:
                self.spline3 = NaturalCubicSpline(np.array(x),
                                                  np.array(y))
        self.memo = {}

    def calc(self, seq, step):
        u"""Calculate the interpolation point together."""
        if not self.sorted_keys:
            self.update()
        if _NEW_SCIPY:
            x = np.array(seq, dtype=np.float64)
            return [[float(s0), float(s1), float(s2)] for (s0, s1, s2)
                    in np.c_[self.spline3(x, 0), self.spline3(x, 1),
                             self.spline3(x, 2)]]
        else:
            return [[float(self.spline3(x, 0)), float(self.spline3(x, 1)),
                     float(self.spline3(x, 2))] for x in seq]


def plot_trajectory(traj, name, step=0.01):
    x = [i * step for i in range(int(traj.length / step) + 1)]
    y0 = [traj[name](s)[0] for s in x]
    y1 = [traj[name](s)[1] for s in x]
    y2 = [traj[name](s)[2] for s in x]
    plt.title(name)
    plt.plot(x, y0, '-', label='point[0]')
    plt.plot(x, y1, '-', label='point[1]')
    plt.plot(x, y2, '-', label='point[2]')
    plt.legend()
    plt.show()
