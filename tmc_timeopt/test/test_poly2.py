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
u"""Solve_ineQulisty's single test."""

import unittest

from nose.tools import eq_
from tmc_timeopt.poly2 import solve_inequality


class SolvePoly2TestCase(unittest.TestCase):
    def test_inequality_normal(self):
        a = (2.0, -3.0, 1.0)
        solution1 = solve_inequality(a, '>')
        eq_([[-float('inf'), 1.0], [2.0, float('inf')]], solution1)
        solution2 = solve_inequality(a, '<')
        eq_([1.0, 2.0], solution2)

        b = (-2.0, 3.0, -1.0)
        solution3 = solve_inequality(b, '>')
        eq_([1.0, 2.0], solution3)
        solution4 = solve_inequality(b, '<')
        eq_([[-float('inf'), 1.0], [2.0, float('inf')]], solution4)

        c = (2.0, 0.1, 1.0)
        solution5 = solve_inequality(c, '>')
        eq_([-float('inf'), float('inf')], solution5)

    def test_nipequelity_no_var(self):
        a = (1.0, 0.0, 0.0)
        solution1 = solve_inequality(a, '>')
        eq_([-float('inf'), float('inf')], solution1)

        solution2 = solve_inequality(a, '<')
        eq_([], solution2)

    def test_inequelity_1(self):
        a = (1.0, 1.0, 0.0)
        solution1 = solve_inequality(a, '>')
        eq_([-1.0, float('inf')], solution1)

        solution2 = solve_inequality(a, '<')
        eq_([-float('inf'), -1.0], solution2)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_poly2_py',
                    SolvePoly2TestCase)
