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
u"""NSPLINE single test."""

import unittest

from nose.tools import assert_almost_equal, ok_

import numpy as np

from tmc_timeopt.nspline import NaturalCubicSpline


class NaturalCubicSplineTestCase(unittest.TestCase):
    def setUp(self):
        self.x = np.array([
            0.9, 1.3, 1.9, 2.1, 2.6,
            3.0, 3.9, 4.4, 4.7, 5.0,
            6.0, 7.0, 8.0, 9.2])
        self.f = np.array([
            1.3, 1.5, 1.85, 2.1, 2.6,
            2.7, 2.4, 2.15, 2.05, 2.1,
            2.25, 2.3, 2.25, 1.95])
        self.spline = NaturalCubicSpline(self.x, self.f)

    def test_nspline(self):
        u"""Check with simple examples."""
        # Confirm the coefficient
        ok_(np.allclose(
            self.f,
            self.spline._a[0],
            atol=1e-3),
            'a is {0}'.format(self.spline._a[0]))

        ok_(np.allclose(np.array(
            [0.540, 0.42, 1.09, 1.29, 0.59,
             -0.02, -0.50, -0.48, -0.07, 0.26,
             0.08, 0.02, -0.15]),
            self.spline._a[1][:-1],
            atol=1e-2),
            'b is {0}'.format(self.spline._a[1][:-1]))

        ok_(np.allclose(np.array(
            [0.00, -0.30, 1.41, -0.37, -1.04,
             -0.50, -0.03, 0.08, 1.27, -0.16,
             -0.03, -0.04, -0.13]),
            self.spline._a[2][:-1],
            atol=1e-2,
            rtol=0.0),
            'c is {0}'.format(self.spline._a[2][:-1]))

        ok_(np.allclose(np.array(
            [-0.24, 0.95, -2.96, -0.45, 0.45,
             0.17, 0.08, 1.31, -1.58, 0.04,
             0.00, -0.03, 0.04]),
            self.spline._a[3][:-1],
            atol=1e-2),
            'd is {0}'.format(self.spline._a[3][:-1]))

    def test_call(self):
        u"""Call check."""
        assert_almost_equal(None, self.spline(0.0))
        assert_almost_equal(self.f[0], self.spline(self.x[0]))
        assert_almost_equal(1.35, self.spline(1.0), places=2)
        assert_almost_equal(self.f[-1], self.spline(self.x[-1]))
        assert_almost_equal(None, self.spline(9.3))

        assert_almost_equal(0.53, self.spline(1.0, der=1), places=2)

        # Since it is a natural sprine, the acceleration is 0 at the end
        assert_almost_equal(0.0, self.spline(self.x[0], der=2))
        assert_almost_equal(-0.149, self.spline(1.0, der=2), places=2)
        assert_almost_equal(0.0, self.spline(self.x[-1], der=2))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('tmc_timeopt', 'test_nspline_py',
                    NaturalCubicSplineTestCase)
