#!/usr/bin/env python
# Copyright (c) 2026 TOYOTA MOTOR CORPORATION
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted (subject to the limitations in the disclaimer
# below) provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of the copyright holder nor the names of its contributors may be used
#   to endorse or promote products derived from this software without specific
#   prior written permission.
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
# LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
# OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.
# -*- coding: utf-8 -*-
u"""Unit test for the Kinematics class."""

from math import cos
from math import sin

import unittest

from tmc_timeopt.kinematics import Kinematics
from tmc_timeopt.target import Target
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt.trajectory import TrajectoryDict


class DummyTarget(Target):
    def __init__(self):
        self.names = ('JOINT1', 'JOINT2', 'JOINT3')

    def set_test_case(self, name):
        self.case_name = name

    def update_dynamics(self):
        pass


class KinematicsTestCase(unittest.TestCase):

    def setUp(self):
        # Test logic only with a dummy target.
        self.target = DummyTarget()
        self.kinematics = Kinematics(self.target)

        self.kinematics.set_limit('JOINT1', 'velocity', (-2, 2))
        self.kinematics.set_limit('JOINT2', 'velocity', (-2, 2))
        self.kinematics.set_limit('JOINT3', 'velocity', (-2, 2))

        self.kinematics.set_limit('JOINT1', 'acceleration', (-2, 2))
        self.kinematics.set_limit('JOINT2', 'acceleration', (-2, 2))
        self.kinematics.set_limit('JOINT3', 'acceleration', (-2, 2))

        self.points = 10

        self.traj = TrajectoryDict(self.points)
        for name in self.target.names:
            self.traj.append(name, NaturalCubicSplineTrajectory)

        for i in range(self.points + 1):
            self.traj['JOINT1'][i] = (sin(0.1 * i), 0.0, 0.0)
            self.traj['JOINT2'][i] = (cos(0.1 * i), 0.0, 0.0)
            self.traj['JOINT3'][i] = (sin(0.4 * i + 1.0), 0.0, 0.0)
        self.kinematics.set_trajectory(self.traj)

    def test_set_limit(self):
        ans = {('JOINT1', 'velocity'): (-2, 2),
               ('JOINT2', 'velocity'): (-2, 2),
               ('JOINT3', 'velocity'): (-2, 2),
               ('JOINT1', 'acceleration'): (-2, 2),
               ('JOINT2', 'acceleration'): (-2, 2),
               ('JOINT3', 'acceleration'): (-2, 2)}
        self.assertEqual(self.kinematics.limits, ans)

    def test_get_current_point(self):
        self.kinematics.update(0.0)
        current = self.kinematics.get_current_point()
        self.assertAlmostEqual(0.0, current['JOINT1'][0])
        self.assertAlmostEqual(1.0, current['JOINT2'][0])
        self.assertAlmostEqual(sin(1.0), current['JOINT3'][0])

        self.kinematics.update(10.0)
        current = self.kinematics.get_current_point()
        self.assertAlmostEqual(sin(1.0), current['JOINT1'][0])
        self.assertAlmostEqual(cos(1.0), current['JOINT2'][0])
        self.assertAlmostEqual(sin(5.0), current['JOINT3'][0])

    def test_get_vlc(self):
        self.kinematics.update(0.0)
        # Obtain the first derivative with respect to s of the trajectory as it is difficult to predict.
        current = self.kinematics.get_current_point()
        fds = (current['JOINT1'][1], current['JOINT2'][1],
               current['JOINT3'][1])
        vlc = self.kinematics.get_vlc()
        # JOINT3 is the most constrained, so vlc should correspond to it.
        self.assertAlmostEqual(2.0 / fds[2], vlc)

    # Almost a reversal of reality, making it a test with little meaning.
    def test_get_state(self):
        # This is not originally necessary to call, but it is called once for testing purposes.
        self.kinematics.update(5.0)
        current = self.kinematics.get_current_point()
        f = (current['JOINT1'][0], current['JOINT2'][0],
             current['JOINT3'][0])
        fds = (current['JOINT1'][1], current['JOINT2'][1],
               current['JOINT3'][1])
        fdds = (current['JOINT1'][2], current['JOINT2'][2],
                current['JOINT3'][2])
        state = self.kinematics.get_state(5.0, 1.0, 1.0)
        self.assertAlmostEqual(state['JOINT1'][0], f[0])
        self.assertAlmostEqual(state['JOINT1'][1], fds[0] * 1.0)
        self.assertAlmostEqual(state['JOINT1'][2], fds[0] * 1.0 + fdds[0] * 1.0)

        self.assertAlmostEqual(state['JOINT2'][0], f[1])
        self.assertAlmostEqual(state['JOINT2'][1], fds[1] * 1.0)
        self.assertAlmostEqual(state['JOINT2'][2], fds[1] * 1.0 + fdds[1] * 1.0)

        self.assertAlmostEqual(state['JOINT3'][0], f[2])
        self.assertAlmostEqual(state['JOINT3'][1], fds[2] * 1.0)
        self.assertAlmostEqual(state['JOINT3'][2], fds[2] * 1.0 + fdds[2] * 1.0)

    # A test just in case, as prediction is difficult.
    def test_calc_accel_limit(self):
        self.kinematics.update(5.0)
        (sa_min, sa_max) = self.kinematics.calc_accel_limit(1.0)
        self.assertGreater(sa_max, sa_min)

    def test_set_limit_invalid_name(self):
        with self.assertRaises(ValueError):
            self.kinematics.set_limit('JOINT_NONE', 'velocity', (-1, 1))

    def test_set_limit_invalid(self):
        with self.assertRaises(TypeError):
            self.kinematics.set_limit('JOINT1', 'none', 1)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_kinematics_py',
                    KinematicsTestCase)
