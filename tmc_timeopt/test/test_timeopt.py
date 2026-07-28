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
u"""Unit test for the Timeopt class."""

from __future__ import print_function

from math import cos
from math import sin

import unittest

from unittest.mock import MagicMock

from tmc_timeopt.target import Target
from tmc_timeopt.timeopt import Timeopt
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt.trajectory import TrajectoryDict


class DummyTarget(Target):
    u"""Dummy target class."""

    test_case = {'CASE_A': {'JOINT1': (1, 1, 1, 1),
                            'JOINT2': (2, 1, 1, 1),
                            'JOINT3': (4, 1, 1, 1)},
                 'CASE_B': {'JOINT1': (-1, 1, 1, 1),
                            'JOINT2': (-2, 1, 1, 1),
                            'JOINT3': (-4, 1, 1, 1)},
                 'CASE_C': {'JOINT1': (1, 1, 1, 1),
                            'JOINT2': (-1, 1, 1, 1),
                            'JOINT3': (1, 1, 1, 1)},
                 'CASE_D': {'JOINT1': (1, 1, 1, 1),
                            'JOINT2': (-1, 1, 1, 1),
                            'JOINT3': (0, 1, 2, 1)},
                 'CASE_E': {'JOINT1': (1, 0, 0, 100),
                            'JOINT2': (-1, 0, 0, 100),
                            'JOINT3': (1, 0, 0, 100)}}

    def __init__(self):
        self.names = ['JOINT1',
                      'JOINT2',
                      'JOINT3']

    def set_test_case(self, name):
        self.case_name = name

    def update_dynamics(self):
        pass

    def get_dynamics(self):
        a = {(name, 'effort'): self.test_case[self.case_name][name][0]
             for name in self.test_case[self.case_name]}
        b = {(name, 'effort'): self.test_case[self.case_name][name][1]
             for name in self.test_case[self.case_name]}
        c = {(name, 'effort'): self.test_case[self.case_name][name][2]
             for name in self.test_case[self.case_name]}
        d = {(name, 'effort'): self.test_case[self.case_name][name][3]
             for name in self.test_case[self.case_name]}
        return a, b, c, d


class TestTimeopt(unittest.TestCase):
    def setUp(self):
        self.target = DummyTarget()
        self.timeopt = Timeopt(self.target)

        self.points = 5

        self._traj = TrajectoryDict(self.points)
        for name in self.target.names:
            self._traj.append(name, NaturalCubicSplineTrajectory)

        for i in range(self.points + 1):
            self._traj['JOINT1'][i] = (sin(0.1 * i), 0.0, 0.0)
            self._traj['JOINT2'][i] = (cos(0.1 * i), 0.0, 0.0)
            self._traj['JOINT3'][i] = (sin(0.4 * i + 1.0), 0.0, 0.0)
        self.timeopt.set_trajectory(self._traj)

    def test_preprocess(self):
        u"""Perform the following checks.

        1. Check if MVC and VLC are stored correctly.
        2. Test if dynamics are calculated correctly.
        """
        # Prepare mock values for checking MVC and VLC.
        VLC_CHECK = 11.0
        MVC_CHECK = 22.0
        SA_CHECK = 33.0

        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = VLC_CHECK
        self.timeopt._dynamics.get_mvc.return_value = MVC_CHECK
        self.timeopt._dynamics.calc_accel_limit.return_value = (SA_CHECK, SA_CHECK)

        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)

        size = self.timeopt._size
        self.assertEqual(0.1, self.timeopt._ds)
        self.assertEqual(41, size)
        self.assertEqual(size, len(self.timeopt._sd))
        self.assertEqual(size, len(self.timeopt._sv))
        self.assertEqual(size, len(self.timeopt._sa))
        self.assertEqual([VLC_CHECK] * size, self.timeopt._vlc)
        self.assertEqual([MVC_CHECK] * size, self.timeopt._mvc)
        self.assertEqual([SA_CHECK] * size, self.timeopt._sa_mvc)
        self.assertAlmostEqual(4.0, self.timeopt._sd[-1])
        # Check a, b, c, d.
        test_abcd = self.target.test_case['CASE_A']
        for i in range(size):
            for pair in self.timeopt._dynamics.limits:
                self.assertEqual([test_abcd[pair[0]][0]] * size, self.timeopt._a_buff[pair])
                self.assertEqual([test_abcd[pair[0]][1]] * size, self.timeopt._b_buff[pair])
                self.assertEqual([test_abcd[pair[0]][2]] * size, self.timeopt._c_buff[pair])
                self.assertEqual([test_abcd[pair[0]][3]] * size, self.timeopt._d_buff[pair])

    def test_update_error(self):
        with self.assertRaises(RuntimeError):
            self.timeopt._Timeopt__integrate_backward_segment = MagicMock('_Timeopt__integrate_backward_segment')
            self.timeopt._Timeopt__integrate_forward_segment = MagicMock('_Timeopt__integrate_forward_segment')
            self.timeopt._Timeopt__integrate_forward_segment.return_value = ('OK', 1)
            self.timeopt._Timeopt__search_switching_point = MagicMock('_Timeopt__search_switching_point')
            self.timeopt._Timeopt__search_switching_point.return_value = ('NG', 1)
            # Process.
            self.target.set_test_case('CASE_A')
            self.timeopt.preprocess(0.1)
            self.timeopt.update()

    def test_integrate_forward_segment_cross(self):
        u"""Check if intersection detection works correctly."""
        self.timeopt._Timeopt__integrate_forward_adaptive = \
            MagicMock('_Timeopt__integrate_forward_adaptive')
        # In this case, values should be assigned to sv[1], sv[2], sa[0], sa[1].
        self.timeopt._Timeopt__integrate_forward_adaptive.side_effect = \
            [('OK', 1, 1, 1), ('OK', 2, 2, 2), ('OK', 3, 3, 3),
             ('OK', 4, 4, 4), ('OK', 5, 5, 5), ('OK', 6, 6, 6)]

        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        # Set values considering backward integration for intersection detection.
        self.timeopt._sv = [4.5] * self.timeopt._size

        result = self.timeopt._Timeopt__integrate_forward_segment(0)
        self.assertEqual(('END', 4), result)
        self.assertEqual(1, self.timeopt._sv[1])
        self.assertEqual(2, self.timeopt._sv[2])
        self.assertEqual(3, self.timeopt._sv[3])
        self.assertEqual(4, self.timeopt._sv[4])
        self.assertEqual(4.5, self.timeopt._sv[5])

        self.assertEqual(1, self.timeopt._sa[0])
        self.assertEqual(2, self.timeopt._sa[1])
        self.assertEqual(3, self.timeopt._sa[2])
        self.assertEqual(4, self.timeopt._sa[3])
        self.assertEqual(5, self.timeopt._sa[4])

    def test_integrate_forward_segment_stop(self):
        u"""Check if the buffer for sv and sa is updated correctly upon stopping."""
        self.timeopt._Timeopt__integrate_forward_adaptive = MagicMock('_Timeopt__integrate_forward_adaptive')
        # In this case, values should be assigned to sv[1], sv[2], sa[0], sa[1].
        self.timeopt._Timeopt__integrate_forward_adaptive.side_effect = \
            [('OK', 1, 1, 1), ('OK', 2, 2, 2), ('MVC', 3, 3, 3)]

        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)

        result = self.timeopt._Timeopt__integrate_forward_segment(0)
        self.assertEqual(('MVC', 2), result)
        self.assertEqual(1, self.timeopt._sv[1])
        self.assertEqual(2, self.timeopt._sv[2])
        self.assertEqual(float('inf'), self.timeopt._sv[3])

        self.assertEqual(1, self.timeopt._sa[0])
        self.assertEqual(2, self.timeopt._sa[1])
        self.assertEqual(0, self.timeopt._sa[2])

    def test_integrate_forward_adaptive(self):
        u"""Check if it is divided into multiple parts."""
        self.timeopt._Timeopt__integrate_forward_divide = MagicMock('_Timeopt__integrate_forward_divide')
        self.timeopt._Timeopt__integrate_forward_divide.return_value = ('MVC', 1, 1, 0.1), 0, (1, 1)
        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        self.timeopt._Timeopt__integrate_forward_adaptive(1, 1, 0.1)
        self.assertEqual(self.timeopt._INTEGRATE_DIV_NUM, self.timeopt._Timeopt__integrate_forward_divide.call_count)
        call_args_list = self.timeopt._Timeopt__integrate_forward_divide.call_args_list
        self.assertEqual(1, call_args_list[0][0][3])
        self.assertEqual(4, call_args_list[1][0][3])
        self.assertEqual(16, call_args_list[2][0][3])
        self.assertEqual(64, call_args_list[3][0][3])

    def test_integrate_forward_divide_ok(self):
        u"""Check if the division is correct."""
        self.timeopt._Timeopt__integrate_forward_step = MagicMock('_Timeopt__integrate_forward_step')
        self.timeopt._Timeopt__integrate_forward_step.return_value = ('OK', 1, 2, 3)
        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        # If singular is set, integrate with singular for only 1 step.
        self.timeopt._fwd_singular = True
        result, _, _ = self.timeopt._Timeopt__integrate_forward_divide(0, 1, 0.1, 4)
        self.assertEqual(('OK', 1, 2, 3), result)
        self.assertEqual(4, self.timeopt._Timeopt__integrate_forward_step.call_count)
        args = self.timeopt._Timeopt__integrate_forward_step.call_args_list
        self.assertTrue(args[0][0][3])

    def test_integrate_forward_divide_stop(self):
        u"""Check if it is divided and investigated."""
        self.timeopt._Timeopt__integrate_forward_step = MagicMock('_Timeopt__integrate_forward_step')
        self.timeopt._Timeopt__integrate_forward_step.side_effect = \
            [('OK', 1, 2, 3), ('OK', 1, 2, 3),
             ('MVC', 1, 2, 3), ('OK', 1, 2, 3)]
        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        # If singular is set, integrate with singular for only 1 step.
        self._fwd_singular = True
        result, _, _ = self.timeopt._Timeopt__integrate_forward_divide(
            0, 1, 0.1, 4)
        self.assertEqual(('MVC', 1, 2, 3), result)
        self.assertEqual(3, self.timeopt._Timeopt__integrate_forward_step.call_count)

    def test_integrate_forward_step_mvc1(self):
        u"""When the constraints are strict."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 100.0
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (-10, -10)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_next, sv_next, sa_max) = \
            self.timeopt._Timeopt__integrate_forward_step(
                1.0, 0.0001, 0.1, False)
        self.assertEqual('MVC', r)

    def test_integrate_forward_step_mvc2(self):
        u"""When it applies to MVC."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 100.0
        self.timeopt._dynamics.get_mvc.return_value = 1.05
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_next, sv_next, sa_max) = self.timeopt._Timeopt__integrate_forward_step(1.0, 1.0, 0.1, False)
        dt_max = self.timeopt._Timeopt__calc_dt(1.0, sa_max, 0.1)
        self.assertEqual('MVC', r)
        self.assertAlmostEqual(1.1, sd_next)
        self.assertAlmostEqual(1.0 + sa_max * dt_max, sv_next)

    def test_integrate_forward_step_vlc1(self):
        u"""When it can be shaped into VLC."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 1.05
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_next, sv_next, sa_max) = self.timeopt._Timeopt__integrate_forward_step(1.0, 1.0, 0.1, False)
        self.timeopt._Timeopt__calc_dt(1.0, sa_max, 0.1)
        self.assertEqual('OK', r)
        self.assertAlmostEqual(1.1, sd_next)
        self.assertAlmostEqual(1.05, sv_next)

    def test_integrate_forward_step_vlc2(self):
        u"""When it cannot be shaped into VLC."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 1.05
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (0.99, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_next, sv_next, sa_max) = self.timeopt._Timeopt__integrate_forward_step(1.0, 1.0, 0.1, False)
        self.timeopt._Timeopt__calc_dt(1.0, sa_max, 0.1)
        self.assertEqual('VLC', r)
        self.assertAlmostEqual(1.1, sd_next)

    def test_integrate_forward_step_ok(self):
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 100.0
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_next, sv_next, sa_max) = self.timeopt._Timeopt__integrate_forward_step(1.0, 1.0, 0.1, False)
        dt_max = self.timeopt._Timeopt__calc_dt(1.0, sa_max, 0.1)
        self.assertEqual('OK', r)
        self.assertAlmostEqual(1.1, sd_next)
        self.assertAlmostEqual(1.0 + sa_max * dt_max, sv_next)

    def test_integrate_backward_segment_stop(self):
        u"""Check if the buffer for sv and sa is updated correctly upon stopping."""
        self.timeopt._Timeopt__integrate_backward_adaptive = MagicMock('_Timeopt__integrate_backward_adaptive')
        # In this case, values should be assigned to sv[9], sv[8], sa[10], sa[9].
        self.timeopt._Timeopt__integrate_backward_adaptive.side_effect = \
            [('OK', 9, 9, 9), ('OK', 8, 8, 8), ('MVC', 7, 7, 7)]

        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)

        result = self.timeopt._Timeopt__integrate_backward_segment(10)
        self.assertEqual(('MVC', 8), result)
        self.assertEqual(9, self.timeopt._sv[9])
        self.assertEqual(8, self.timeopt._sv[8])
        self.assertEqual(float('inf'), self.timeopt._sv[7])

        self.assertEqual(9, self.timeopt._sa[9])
        self.assertEqual(8, self.timeopt._sa[8])
        self.assertEqual(0, self.timeopt._sa[7])

    def test_integrate_backward_adaptive(self):
        u"""Check if it is divided into multiple parts."""
        self.timeopt._Timeopt__integrate_backward_divide = MagicMock('_Timeopt__integrate_backward_divide')
        self.timeopt._Timeopt__integrate_backward_divide.return_value = ('MVC', 1, 1, 0.1), 0, (1, 1)
        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        self.timeopt._Timeopt__integrate_backward_adaptive(10, 1, 0.1)
        self.assertEqual(self.timeopt._INTEGRATE_DIV_NUM,
                         self.timeopt._Timeopt__integrate_backward_divide.call_count)
        call_args_list = self.timeopt._Timeopt__integrate_backward_divide.call_args_list
        self.assertEqual(1, call_args_list[0][0][3])
        self.assertEqual(4, call_args_list[1][0][3])
        self.assertEqual(16, call_args_list[2][0][3])
        self.assertEqual(64, call_args_list[3][0][3])

    def test_integrate_backward_divide_ok(self):
        u"""Check if the division is correct."""
        self.timeopt._Timeopt__integrate_backward_step = MagicMock('_Timeopt__integrate_backward_step')
        self.timeopt._Timeopt__integrate_backward_step.return_value = ('OK', 9, 2, 3)
        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        # If singular is set, integrate with singular for only 1 step.
        self.timeopt._bkw_singular = True
        result, _, _ = self.timeopt._Timeopt__integrate_backward_divide(10, 1, 0.1, 4)
        self.assertEqual(('OK', 9, 2, 3), result)
        self.assertEqual(4, self.timeopt._Timeopt__integrate_backward_step.call_count)
        args = self.timeopt._Timeopt__integrate_backward_step.call_args_list
        self.assertTrue(args[0][0][3])

    def test_integrate_backward_divide_stop(self):
        u"""Check if it is divided and investigated."""
        self.timeopt._Timeopt__integrate_backward_step = MagicMock('_Timeopt__integrate_backward_step')
        self.timeopt._Timeopt__integrate_backward_step.side_effect = \
            [('OK', 9, 2, 3), ('OK', 9, 2, 3),
             ('MVC', 9, 2, 3), ('OK', 9, 2, 3)]
        # Process.
        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        # If singular is set, integrate with singular for only 1 step.
        self._fwd_singular = True
        result, _, _ = self.timeopt._Timeopt__integrate_backward_divide(10, 1, 0.1, 4)
        self.assertEqual(('MVC', 9, 2, 3), result)
        self.assertEqual(3, self.timeopt._Timeopt__integrate_backward_step.call_count)

    def test_integrate_backward_step_mvc1(self):
        u"""When the constraints are strict."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 100.0
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (2, 2)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_next, sv_next, sa_min) = self.timeopt._Timeopt__integrate_backward_step(1.0, 0.001, 0.1, False)
        self.assertEqual('MVC', r)

    def test_integrate_backward_step_mvc2(self):
        u"""When it applies to MVC."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 100.0
        self.timeopt._dynamics.get_mvc.return_value = 1.05
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_prev, sv_prev, sa_min) = self.timeopt._Timeopt__integrate_backward_step(1.0, 1.0, 0.1, False)
        self.timeopt._Timeopt__calc_dt(1.0, sa_min, 0.1)
        self.assertEqual('MVC', r)
        self.assertAlmostEqual(0.9, sd_prev)

    def test_integrate_backward_step_vlc1(self):
        u"""When it can be shaped into VLC."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 1.05
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_prev, sv_prev, sa_min) = self.timeopt._Timeopt__integrate_backward_step(1.0, 1.0, 0.1, False)
        self.timeopt._Timeopt__calc_dt(1.0, sa_min, 0.1)
        self.assertEqual('OK', r)
        self.assertAlmostEqual(0.9, sd_prev)
        self.assertAlmostEqual(1.05, sv_prev)

    def test_integrate_backward_step_vlc2(self):
        u"""When it cannot be shaped into VLC."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 1.05
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, -0.99)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_prev, sv_prev, sa_min) = self.timeopt._Timeopt__integrate_backward_step(1.0, 1.0, 0.1, False)
        self.timeopt._Timeopt__calc_dt(1.0, sa_min, 0.1)
        self.assertEqual('VLC', r)
        self.assertAlmostEqual(0.9, sd_prev)

    def test_integrate_backward_step_ok(self):
        u"""When integration is possible."""
        self.timeopt._kinematics.get_vlc = MagicMock('get_vlc')
        self.timeopt._dynamics.get_mvc = MagicMock('get_mvc')
        self.timeopt._dynamics.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt._kinematics.get_vlc.return_value = 100.0
        self.timeopt._dynamics.get_mvc.return_value = 100.0
        self.timeopt._dynamics.calc_accel_limit.return_value = (-1, 1)

        self.target.set_test_case('CASE_A')
        self.timeopt.preprocess(0.1)
        (r, sd_prev, sv_prev, sa_min) = self.timeopt._Timeopt__integrate_backward_step(1.0, 1.0, 0.1, False)
        dt_min = self.timeopt._Timeopt__calc_dt_back(1.0, sa_min, 0.1)
        self.assertEqual('OK', r)
        self.assertAlmostEqual(0.9, sd_prev)
        self.assertAlmostEqual(1.0 - sa_min * dt_min, sv_prev)

    def test_calc_dt(self):
        u"""Verify if uniform acceleration motion follows ds = 0.5 * sa * dt^2 + sv * dt."""
        def calc_ds(sa, sv, dt):
            return 0.5 * sa * dt ** 2 + sv * dt
        ds = 0.1
        self.timeopt._ds = ds

        # sa is positive.
        (sv, sa) = (1, 2)
        dt = self.timeopt._Timeopt__calc_dt(sv, sa, ds)
        self.assertAlmostEqual(calc_ds(sa, sv, dt), ds)

        # sa is negative.
        (sv, sa) = (1, -2)
        dt = self.timeopt._Timeopt__calc_dt(sv, sa, ds)
        self.assertAlmostEqual(calc_ds(sa, sv, dt), ds)

        # sa is zero.
        (sv, sa) = (1, 0)
        dt = self.timeopt._Timeopt__calc_dt(sv, sa, ds)
        self.assertAlmostEqual(calc_ds(sa, sv, dt), ds)

        # No solution.
        (sv, sa) = (0, -1)
        dt = self.timeopt._Timeopt__calc_dt(sv, sa, ds)
        self.assertAlmostEqual(self.timeopt._MINIMUM_DT, dt)

        # Inf
        (sv, sa) = (float('inf'), -float('inf'))
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(self.timeopt._MINIMUM_DT, dt)

        # sa and sv are zero.
        (sv, sa) = (0, 0)
        dt = self.timeopt._Timeopt__calc_dt(sv, sa, ds)
        self.assertAlmostEqual(self.timeopt._MINIMUM_DT, dt)

    def test_calc_dt_back(self):
        u"""Verify if uniform acceleration motion follows ds = - 0.5 * sa * dt^2 + sv * dt."""
        def calc_ds(sa, sv, dt):
            return - 0.5 * sa * dt ** 2 + sv * dt
        ds = 0.1
        self.timeopt._ds = ds

        # sa is positive.
        (sv, sa) = (1, 1)
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(calc_ds(sa, sv, dt), ds)

        # sa is negative.
        (sv, sa) = (1, -2)
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(calc_ds(sa, sv, dt), ds)

        # sa is zero.
        (sv, sa) = (1, 0)
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(calc_ds(sa, sv, dt), ds)

        # No solution.
        (sv, sa) = (0, 1)
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(self.timeopt._MINIMUM_DT, dt)

        # Inf
        (sv, sa) = (float('inf'), -float('inf'))
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(self.timeopt._MINIMUM_DT, dt)

        # sa and sv are zero.
        (sv, sa) = (0, 0)
        dt = self.timeopt._Timeopt__calc_dt_back(sv, sa, ds)
        self.assertAlmostEqual(self.timeopt._MINIMUM_DT, dt)

    def test_recalc_trajectory(self):
        u"""Check if time is calculated using s, sv, and sa."""
        self.timeopt._sd = [0, 1, 2, 3, 4]
        self.timeopt._sv = [0, 0.1, 0.2, 0.3, 0.4]
        self.timeopt._sa = [0.1, 0.1, 0.1, 0.1, 0.1]
        self.timeopt._size = 5
        self.timeopt._ds = 1.0
        self.timeopt._Timeopt__recalc_trajectory()
        self.assertAlmostEqual(0, self.timeopt._time[0], places=3)
        self.assertAlmostEqual(4.4721359, self.timeopt._time[1], places=3)
        self.assertAlmostEqual(8.0547116, self.timeopt._time[2], places=3)
        self.assertAlmostEqual(10.9536911, self.timeopt._time[3], places=3)
        self.assertAlmostEqual(13.3388559, self.timeopt._time[4], places=3)

    def test_tangent_point_found(self):
        u"""Check if the Tangent point can be found."""
        curr = 0
        self.timeopt._ds = 1.0
        self.timeopt._sd = [0, 1, 2]
        self.timeopt._mvc = [10.25, 10.25, 12.25]
        self.timeopt._sa_mvc = [1.0, -1.0, -3.0]

        self.timeopt._Timeopt__iterate_tangent_sp = MagicMock('iterate_tangent_sp')
        self.timeopt._Timeopt__iterate_tangent_sp.return_value = (0.5, 1, 0.5, 1)

        sp = self.timeopt._Timeopt__check_tangent_point(curr)
        self.assertAlmostEqual(0.5, sp[0])
        self.assertAlmostEqual(0.95, sp[1])
        self.assertAlmostEqual(0.5, sp[2])
        self.assertAlmostEqual(0.95, sp[3])

    def test_check_trap_point(self):
        u"""Example of a Trap point."""
        self.timeopt._kinematics.update = MagicMock()
        self.timeopt._dynamics.update = MagicMock()
        self.timeopt.calc_accel_limit = MagicMock('calc_accel_limit')
        self.timeopt.calc_accel_limit.return_value = (-0.1, 0.1)
        self.timeopt._vlc = [1, 2]
        self.timeopt._mvc = [2, 3]
        self.timeopt._sd = [0, 0.1]
        self.timeopt._sv = [1, 1]
        self.timeopt._ds = 0.1
        sp = self.timeopt._Timeopt__check_trap_point(0)
        self.assertAlmostEqual(0.0, sp[0])
        self.assertAlmostEqual(1.0, sp[1])
        self.assertAlmostEqual(0.1, sp[2])
        self.assertAlmostEqual(2.0, sp[3])

    def test_check_zero_inertia_point(self):
        self.timeopt._dynamics.set_limit('joint1', 'effort', (-0.1, 0.1))
        pair = ('joint1', 'effort')

        # When there is no zero_inertia_point.
        self.timeopt._a_buff[pair] = [0, 1, 1]
        res = self.timeopt._Timeopt__check_zero_inertia_point(1)
        self.assertEqual([], res)

        # When there is a zero_inertia_point (case 1).
        self.timeopt._a_buff[pair] = [0, 0, 1]
        res = self.timeopt._Timeopt__check_zero_inertia_point(1)
        self.assertEqual([pair], res)

        # When there is a zero_inertia_point (case 2).
        self.timeopt._a_buff[pair] = [0, 1, 0]
        res = self.timeopt._Timeopt__check_zero_inertia_point(1)
        self.assertEqual([pair], res)

        # When there is a zero_inertia_point (case 3).
        self.timeopt._a_buff[pair] = [0, 1, -1]
        res = self.timeopt._Timeopt__check_zero_inertia_point(1)
        self.assertEqual([pair], res)

    def test_integrate_from_switiching_point(self):
        self.timeopt._Timeopt__integrate_forward_adaptive = MagicMock('fwa')
        self.timeopt._Timeopt__integrate_backward_adaptive = MagicMock('bwa')
        self.timeopt._Timeopt__integrate_backward_segment = MagicMock('bws')

        # Preconditions.
        self.timeopt._sd = [1, 2, 3, 4]
        self.timeopt._sv = [1, 1, 1, 1]
        sp = (2.4, 1, 2.6, 1)

        # Return value.
        self.timeopt._Timeopt__integrate_forward_adaptive.return_value = ('MVC', 0, 0, 0)
        self.timeopt._Timeopt__integrate_backward_adaptive.return_value = ('OK', 0, 0, 0)
        self.timeopt._Timeopt__integrate_backward_segment.return_value = ('MVC', 0)

        # Check.
        res = self.timeopt._Timeopt__integrate_from_switching_point(2, sp)
        self.assertFalse(res)

        # Return value.
        self.timeopt._Timeopt__integrate_forward_adaptive.return_value = ('OK', 0, 0, 0)
        self.timeopt._Timeopt__integrate_backward_adaptive.return_value = ('MVC', 0, 0, 0)
        self.timeopt._Timeopt__integrate_backward_segment.return_value = ('MVC', 0)

        # Check.
        res = self.timeopt._Timeopt__integrate_from_switching_point(2, sp)
        self.assertFalse(res)

        # Return value.
        self.timeopt._Timeopt__integrate_forward_adaptive.return_value = ('OK', 0, 0, 0)
        self.timeopt._Timeopt__integrate_backward_adaptive.return_value = ('OK', 0, 0, 0)
        self.timeopt._Timeopt__integrate_backward_segment.return_value = ('MVC', 0)

        # Check.
        res = self.timeopt._Timeopt__integrate_from_switching_point(2, sp)
        self.assertFalse(res)

        # Return value.
        self.timeopt._Timeopt__integrate_forward_adaptive.return_value = ('OK', 3, 2, 0)
        self.timeopt._Timeopt__integrate_backward_adaptive.return_value = ('OK', 2, 5, 0)
        self.timeopt._Timeopt__integrate_backward_segment.return_value = ('END', 0)

        # Check.
        res = self.timeopt._Timeopt__integrate_from_switching_point(2, sp)
        self.assertTrue(res)
        # Check the update of the s trajectory.
        self.assertEqual(3, self.timeopt._sd[3])
        self.assertEqual(2, self.timeopt._sv[3])
        self.assertEqual(2, self.timeopt._sd[2])
        self.assertEqual(5, self.timeopt._sv[2])


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_timeopt_py',
                    TestTimeopt)
