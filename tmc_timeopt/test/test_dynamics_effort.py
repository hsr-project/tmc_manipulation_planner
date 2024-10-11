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
u"""Dynamics class single test."""

from math import sqrt

import unittest

from nose.tools import assert_almost_equal, eq_, raises
from tmc_timeopt.dynamics import Dynamics
from tmc_timeopt.target import Target


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


class DynamicsTestCase(unittest.TestCase):

    def setUp(self):
        # Test only logic with a dummy target
        self.target = DummyTarget()
        self.dynamics = Dynamics(self.target)

        self.dynamics.set_limit('JOINT1', 'effort', (-2, 2))
        self.dynamics.set_limit('JOINT2', 'effort', (-2, 2))
        self.dynamics.set_limit('JOINT3', 'effort', (-2, 2))

    def test_set_limit(self):
        ans = {('JOINT1', 'effort'): (-2, 2),
               ('JOINT2', 'effort'): (-2, 2),
               ('JOINT3', 'effort'): (-2, 2)}
        eq_(self.dynamics.limits, ans)

    def test_calc_accel_limit_sv_inf(self):
        # CASE A
        self.target.set_test_case('CASE_A')
        self.dynamics.update()
        self.dynamics.calc_accel_limit(1e+300)

    def test_calc_accel_limit(self):
        # CASE A
        self.target.set_test_case('CASE_A')
        self.dynamics.update()
        (sa_min, sa_max) = self.dynamics.calc_accel_limit(0.0)
        assert_almost_equal((-3.0 / 4.0, 1.0 / 4.0), (sa_min, sa_max))

        # CASE B
        self.target.set_test_case('CASE_B')
        self.dynamics.update()
        (sa_min, sa_max) = self.dynamics.calc_accel_limit(0.0)
        assert_almost_equal((-1.0 / 4.0, 3.0 / 4.0), (sa_min, sa_max))

    def test_get_mvc(self):
        # CASE C
        self.target.set_test_case('CASE_C')
        self.dynamics.update()
        mvc = self.dynamics.get_mvc()
        assert_almost_equal((sqrt(5) - 1) / 2, mvc)

        # CASE D
        self.target.set_test_case('CASE_D')
        self.dynamics.update()
        mvc = self.dynamics.get_mvc()
        assert_almost_equal(sqrt(2) - 1, mvc)

    @raises(ValueError)
    def test_get_mvc_exception(self):
        self.target.set_test_case('CASE_E')
        self.dynamics.update()
        self.dynamics.get_mvc()

    def test_zero_inertia_sv(self):
        self.target.set_test_case('CASE_D')
        self.dynamics.update()
        sv_max = self.dynamics.calc_zero_inertia_sv(('JOINT3', 'effort'))
        assert_almost_equal(-1.0 + sqrt(2.0), sv_max)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_dynamics_py',
                    DynamicsTestCase)
