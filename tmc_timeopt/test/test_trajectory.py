#!/usr/bin/env python
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
# -*- coding: utf-8 -*-

import unittest

from tmc_timeopt.trajectory import LinearTrajectory
from tmc_timeopt.trajectory import Poly3Trajectory
from tmc_timeopt.trajectory import Poly5Trajectory
from tmc_timeopt.trajectory import Trajectory
from tmc_timeopt.trajectory import TrajectoryDict


class TrajectoryTestCase(unittest.TestCase):

    def assertPointEqual(self, point0, point1):
        for p0, p1 in zip(point0, point1):
            self.assertAlmostEqual(p0, p1, places=5)

    def setUp(self):
        self.traj = TrajectoryDict(1.0)
        self.traj.append('Trajectory', Trajectory)
        self.traj.append('LinearTrajectory', LinearTrajectory)
        self.traj.append('Poly3Trajectory', Poly3Trajectory)
        self.traj.append('Poly5Trajectory', Poly5Trajectory)

    def test_trajectory_dict(self):
        self.assertEqual(self.traj.length, 1.0)
        point = self.traj(0)
        for name in point:
            self.assertEqual(point[name][0], 0)

    def test_trajectory(self):
        u"""Trajectory test."""
        traj = self.traj['Trajectory']
        # Initial state
        self.assertPointEqual(traj[0], [0, 0, 0])
        # Exceptions when accessing the score that does not exist
        with self.assertRaises(KeyError):
            traj[1.0]
        with self.assertRaises(KeyError):
            traj[0.5]
        # Settings via points
        for i in range(11):
            s = 0.1 * i
            traj[s] = (s, s * 0.1, s * 0.01)
        # Only tuple or list can be set
        with self.assertRaises(ValueError):
            traj[0] = 1.0
        with self.assertRaises(ValueError):
            traj[0] = 'hello'
        # Confirm that the interpolation point returns the left point
        for i in range(10):
            s = 0.1 * i
            self.assertPointEqual(traj(s + 0.05), (s, s * 0.1, s * 0.01))
        self.assertPointEqual(traj(1), traj[1])
        # Exceptions when trying to interpose of points outside the range
        with self.assertRaises(ValueError):
            traj(2)
        # Roll it to 0 for the negative person
        self.assertPointEqual(traj(-1), [0.0, 0.0, 0.0])

    def test_linear_trajectory(self):
        u"""Lineartrajectory test."""
        traj = self.traj['LinearTrajectory']
        # Initial state
        self.assertPointEqual(traj[0], [0, 0, 0])
        # Settings via points
        traj[0.0] = (0, 0, 0)
        traj[0.5] = (0.5, 0, 0)
        traj[1.0] = (0, 0, 0)
        # Exceptions when accessing the score that does not exist
        with self.assertRaises(KeyError):
            traj[0.7]
        # Only tuple or list can be set
        with self.assertRaises(ValueError):
            traj[0.5] = 1.0
        with self.assertRaises(ValueError):
            traj[0.5] = 'hello'
        # Confirm the interpolation value
        self.assertPointEqual(traj(0.0), [0.0, 1.0])
        self.assertPointEqual(traj(0.2), [0.2, 1.0])
        self.assertPointEqual(traj(0.5), [0.5, -1.0])
        self.assertPointEqual(traj(0.7), [0.3, -1.0])
        self.assertPointEqual(traj(1.0), [0.0, -1.0])
        # Exceptions when trying to interpose of points outside the range
        with self.assertRaises(ValueError):
            traj(2)
        # Roll it to 0 for the negative person
        self.assertPointEqual(traj(-1), [0.0, 1.0])

    def test_poly3_trajectory(self):
        u"""Poly3TRAJECTORY test."""
        traj = self.traj['Poly3Trajectory']
        # Initial state
        self.assertPointEqual(traj[0], [0, 0, 0])
        # Settings via points
        traj[0.0] = (0.0, 1.0, 0.0)
        traj[0.5] = (0.5, -1.0, 0.0)
        traj[1.0] = (0.0, 1.0, 0.0)
        traj.update()
        a_ans = {0.0: (0, 1, 4, -8),
                 0.5: (0.5, -1, -4, 8),
                 1.0: (0, 1, 0, 0)}
        # Confirmation of interpolation parameters
        for x in a_ans:
            self.assertPointEqual(traj.a[x], a_ans[x])

    def test_poly5_trajectory(self):
        u"""Poly5trajectory test."""
        traj = self.traj['Poly5Trajectory']
        # Initial state
        self.assertPointEqual(traj[0], [0, 0, 0])
        # Settings via points
        traj[0.0] = (0.0, 1.0, 0.0)
        traj[0.5] = (0.5, -1.0, 0.0)
        traj[1.0] = (0.0, 1.0, 0.0)
        traj.update()
        a_ans = {0.0: (0, 1, 0),
                 0.5: (0.5, -1, 0),
                 1.0: (0, 1, 0)}
        # Confirmation of interpolation parameters
        for x in a_ans:
            self.assertPointEqual(traj.a[x], a_ans[x])


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_trajectory_py',
                    TrajectoryTestCase)
