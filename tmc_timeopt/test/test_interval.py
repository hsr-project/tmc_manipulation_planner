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
u"""Solve_ineQulisty single test"""

import unittest

from nose.tools import eq_
from tmc_timeopt.interval import Interval


class IntervalTest(unittest.TestCase):
    def test_add1(self):
        int1 = Interval(lst=[1, 3])
        int2 = Interval(lst=[2, 4])
        int1_plus_int2 = int1 + int2
        eq_([[1, 4]], int1_plus_int2.get_list())

    def test_add2(self):
        int1 = Interval(lst=[1, 2])
        int2 = Interval(lst=[3, 4])
        int1_plus_int2 = int1 + int2
        eq_([[1, 2], [3, 4]], int1_plus_int2.get_list())

    def test_add3(self):
        int1 = Interval(lst=[1, 5])
        int2 = Interval(lst=[3, 4])
        int1_plus_int2 = int1 + int2
        eq_([[1, 5]], int1_plus_int2.get_list())

    def test_add_no1(self):
        int1 = Interval(lst=[])
        int2 = Interval(lst=[3, 4])
        int1_plus_int2 = int1 + int2
        eq_([[3, 4]], int1_plus_int2.get_list())

    def test_add_no2(self):
        int1 = Interval(lst=[1, 2])
        int2 = Interval(lst=[])
        int1_plus_int2 = int1 + int2
        eq_([[1, 2]], int1_plus_int2.get_list())

    def test_mul(self):
        int1 = Interval(lst=[1, 3])
        int2 = Interval(lst=[2, 4])
        int1_mul_int2 = int1 * int2
        eq_([[2, 3]], int1_mul_int2.get_list())

    def test_mul_no(self):
        int1 = Interval(lst=[1, 3])
        int2 = Interval(lst=[])
        int1_mul_int2 = int1 * int2
        eq_([], int1_mul_int2)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_interval_py',
                    IntervalTest)
