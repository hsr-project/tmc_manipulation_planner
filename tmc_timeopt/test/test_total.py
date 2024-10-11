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
u"""Overall operation test.

Increased random and can be diverted to tests during development
"""

import unittest

from nose.tools import assert_greater_equal

import numpy as np

from tmc_timeopt.target import Target
from tmc_timeopt.timeopt import Timeopt
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt.trajectory import TrajectoryDict

# SEED is fixed in the automatic test
np.random.seed(10)


class MyTarget(Target):
    u"""My Target class."""

    def __init__(self):
        self.names = ['joint0',
                      'joint1',
                      'joint2']
        self.point = {}

    def update_kinematics(self, point):
        u"""Update TARGET athletic.

        Args:
            Point: DICT of state amount
        """
        self.point = point

    def update_dynamics(self):
        u"""I don't do anything because it is an acceleration level restraint."""
        pass

    def get_dynamics(self):
        u"""Return the dynamic spalameter (A, B, C, D).

        Each (a, b, c, d) is a key ('variable name', 'restriction type'), respectively, and the value is Value.
        """
        (a, b, c, d) = ({}, {}, {}, {})
        for name in self.names:
            a[name, 'acceleration'] = self.point[name][1]
            b[name, 'acceleration'] = self.point[name][2]
            c[name, 'acceleration'] = 0
            d[name, 'acceleration'] = 0
        return a, b, c, d


class TotalTestCase(unittest.TestCase):
    u"""Overall test."""

    VEL_LIMIT = {'joint0': 0.1, 'joint1': 0.2, 'joint2': 0.3}
    ACC_LIMIT = {'joint0': 0.1, 'joint1': 0.1, 'joint2': 0.1}
    u"""Test case."""
    def setUp(self):
        self.target = MyTarget()
        self.timeopt = Timeopt(self.target)
        kinematics = self.timeopt.kinematics()
        dynamics = self.timeopt.dynamics()
        joint_names = self.target.names

        for joint in joint_names:
            dynamics.set_limit(joint, 'acceleration',
                               (-self.ACC_LIMIT[joint], self.ACC_LIMIT[joint]))
            kinematics.set_limit(joint,
                                 'velocity',
                                 (-self.VEL_LIMIT[joint],
                                  self.VEL_LIMIT[joint]))

    def count_score(self, opt_traj):
        u"""How much optimization score is given for testing.

        Conditions are in speed or acceleration limit
        The ratio of all hours is the time when there is a joint that has reached 90%
        Args:
           Opt_traj: Optimized orbit
        Returns:
           Score(double)
        """
        satification = []
        joint_names = self.target.names
        MERGIN = 0.9
        for point in opt_traj:
            satification.append(any([abs(point[1][joint][1]) > MERGIN * self.VEL_LIMIT[joint]
                                     or abs(point[1][joint][2]) > MERGIN * self.ACC_LIMIT[joint]
                                     for joint in joint_names]))
        return float(satification.count(True)) / float(len(satification))

    def test_normal_case1(self):
        u"""Test using simple orbitals.

        Passed if Score exceeded 0.9
        """
        _POINTS = 30

        traj = TrajectoryDict(_POINTS)
        for name in self.target.names:
            traj.append(name, NaturalCubicSplineTrajectory)

        for i in range(_POINTS + 1):
            traj['joint0'][i] = (np.sin(0.1 * i), 0.0, 0)
            traj['joint1'][i] = (np.cos(0.1 * i), 0.0, 0)
            traj['joint2'][i] = (0.3 * np.sin(0.4 * i + 0.1), 0.0, 0)

        self.timeopt.set_trajectory(traj)
        self.timeopt.preprocess(0.1)
        self.timeopt.update()
        opt_traj = self.timeopt.get_optimal_trajectory()
        score = self.count_score(opt_traj)
        assert_greater_equal(score, 0.9)

    def test_normal_case2(self):
        u"""Test using short orbit.

        Passed if Score exceeded 0.9
        """
        _POINTS = 2

        traj = TrajectoryDict(_POINTS)
        for name in self.target.names:
            traj.append(name, NaturalCubicSplineTrajectory)

        for i in range(_POINTS + 1):
            traj['joint0'][i] = (np.sin(0.1 * i), 0.0, 0)
            traj['joint1'][i] = (np.cos(0.1 * i), 0.0, 0)
            traj['joint2'][i] = (0.3 * np.sin(0.4 * i + 0.1), 0.0, 0)

        self.timeopt.set_trajectory(traj)
        self.timeopt.preprocess(0.1)
        self.timeopt.update()
        opt_traj = self.timeopt.get_optimal_trajectory()
        score = self.count_score(opt_traj)
        assert_greater_equal(score, 0.9)

    def test_normal_case3(self):
        u"""Tests using long orbit.

        Passed if Score exceeded 0.9
        """
        _POINTS = 500

        traj = TrajectoryDict(_POINTS)
        for name in self.target.names:
            traj.append(name, NaturalCubicSplineTrajectory)

        for i in range(_POINTS + 1):
            traj['joint0'][i] = (np.sin(0.1 * i), 0.0, 0)
            traj['joint1'][i] = (np.cos(0.1 * i), 0.0, 0)
            traj['joint2'][i] = (0.3 * np.sin(0.4 * i + 0.1), 0.0, 0)

        self.timeopt.set_trajectory(traj)
        self.timeopt.preprocess(0.1)
        self.timeopt.update()
        opt_traj = self.timeopt.get_optimal_trajectory()
        score = self.count_score(opt_traj)
        assert_greater_equal(score, 0.9)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_total.py',
                    TotalTestCase)
