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
u"""Dynamics interface module required for Topp."""

import itertools
import math
import sys

from tmc_timeopt.interval import Interval
import tmc_timeopt.poly2 as poly2


class Dynamics(object):
    u"""Bass class that defines Dynamics required for Topp."""

    def __init__(self, target):
        u"""Initialize this class.

        Args:
            target (Target): Targeting dynamics
        """
        self.target = target
        self.limits = {}
        self.get_mvc = self.get_mvc_accel_effort
        # dict of dynamics parameters
        (self.a, self.b, self.c, self.d) = ({}, {}, {}, {})

    def set_limit(self, name, limit_type, limit):
        u"""Set constraints such as torque.

        Args:
            name (str): Name of output variable
            limit_type (str): Limit type ('Effort', 'ZMP', etc)
            limit (tuple): (lower limit, upper limit)
        """
        self.limits[name, limit_type] = limit

    def update(self):
        u"""Acquired the dynamic parameter (A, B, C, D) of Target and update the member variable."""
        self.target.update_dynamics()
        (self.a, self.b, self.c, self.d) = self.target.get_dynamics()

    def calc_accel_limit(self, sv):
        u"""Calculate the upper and lower limit of orbit acceleration under Dynamics constraints.

        Args:
            sv (Float): Speed ​​of S in orbit
        Return:
            (l, u): Acceleration (lower, upper limit)
        """
        (sa_min, sa_max) = ({}, {})

        # If the SV is a very large value, if you do not use INF, you will get an exception with SV ** 2.
        try:
            sv ** 2
        except OverflowError:
            sv = float('inf')

        effort_limits = [pair for pair in self.limits
                         if pair[1] == 'effort' or pair[1] == 'acceleration']
        for pair in effort_limits:
            limit = self.limits[pair]
            (sa_min[pair], sa_max[pair]) = (float('-inf'), float('inf'))
            if self.a[pair] > sys.float_info.epsilon:
                sa_min[pair] = (limit[0] - self.b[pair] * sv ** 2 - self.c[
                    pair] * sv - self.d[pair]) / self.a[pair]
                sa_max[pair] = (limit[1] - self.b[pair] * sv ** 2 - self.c[
                    pair] * sv - self.d[pair]) / self.a[pair]
            elif self.a[pair] < - sys.float_info.epsilon:
                sa_min[pair] = (limit[1] - self.b[pair] * sv ** 2 - self.c[
                    pair] * sv - self.d[pair]) / self.a[pair]
                sa_max[pair] = (limit[0] - self.b[pair] * sv ** 2 - self.c[
                    pair] * sv - self.d[pair]) / self.a[pair]
        # Returns a set of up and down limit
        (lst, u) = (float('-inf'), float('inf'))
        for pair in self.limits:
            lst = max(lst, sa_min[pair])
            u = min(u, sa_max[pair])
        return (lst, u)

    def get_mvc_accel(self):
        u"""Returns the point on the MVC (Maximum Velocity Curve) calculated from the acceleration restriction.

        It is necessary to call Update () and set a point.

        The restrictions on the original get_mvc_accel_effort are limited to acceleration constraints.
        In acceleration control, it can be calculated at high speed because it becomes CI == 0, DI == 0.

        Return:
            mvc (float): speed of S at MVC points (SV)
        """
        (u, l) = ({}, {})
        cond = [pair for pair in self.limits
                if pair[1] == 'acceleration']
        non_zero = []  # List of conditions of A! = 0
        rlst = []

        # Check the sign of A under each condition and sort
        for pair in cond:
            limit = self.limits[pair]
            if self.a[pair] > sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[0], limit[1])
                non_zero.append(pair)
            elif self.a[pair] < - sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[1], limit[0])
                non_zero.append(pair)
            # In the case of a == 0, add it to the ZERO list and exclude it from COND.
            else:
                l[pair] = limit[0]
                u[pair] = limit[1]
                r = [-self.d[pair] + u[pair], -self.c[pair], -self.b[pair]]
                rlst.append(r)
                r = [self.d[pair] - l[pair], self.c[pair], self.b[pair]]
                rlst.append(r)

        # Calculate the secondary inequality parameters
        p = {pair: [(l[pair] - self.d[pair]) / self.a[pair],
                    -self.c[pair] / self.a[pair],
                    -self.b[pair] / self.a[pair]]
             for pair in non_zero}
        q = {pair: [(u[pair] - self.d[pair]) / self.a[pair],
                    -self.c[pair] / self.a[pair],
                    -self.b[pair] / self.a[pair]]
             for pair in non_zero}
        # Find a solution for all combinations
        # It also depends on the positional relationship (Q-P-P-Q) of the two conditions, so it becomes Permutations.
        for pair_1, pair_2 in itertools.permutations(non_zero, 2):
            r = [q[pair_1][i] - p[pair_2][i] for i in range(3)]
            rlst.append(r)

        amin = float('Inf')
        for r in rlst:
            if r[1] != 0:
                raise ValueError('Maybe not acceleration constraint')
            if r[2] == 0:
                if r[0] < 0:
                    raise ValueError('Too tight constraint')
            else:
                det = -4 * r[2] * r[0]
                if det < 0:
                    if r[2] < 0:
                        raise ValueError('Too tight constraint')
                else:
                    amin = min(amin, abs(math.sqrt(det) / (2 * r[2])))

        self.mvc = amin
        return self.mvc

    def get_mvc_accel_effort(self):
        u"""Returns the point on the MVC (Maximum Velocity Curve) calculated from the torque restriction.

        It is necessary to call Update () and set a point.

        when ai > 0
        sa_min_i < (l_limit_i - bi * sv**2 - ci * sv - di) / ai
        sa_max_i < (u_limit_i - bi * sv**2 - ci * sv - di) / ai

        when ai < 0
        sa_min_i < (u_limit_i - bi * sv**2 - ci * sv - di) / ai
        sa_max_i < (l_limit_i - bi * sv**2 - ci * sv - di) / ai

        However, sa_min_i <= sa_max_i
        I saw everything,

        when ai == 0
        l_limit_i < bi * sv**2 + ci * sv + di
        u_limit_i > bi * sv**2 + ci * sv + di

        The maximum value of the first area of ​​the SV is MVC
        It may be defined defined.

        Return:
            mvc (float): speed of S at MVC points (SV)
        """
        (u, l, p, q) = ({}, {}, {}, {})
        cond = [pair for pair in self.limits
                if pair[1] == 'effort' or pair[1] == 'acceleration']
        non_zero = []  # List of conditions of A! = 0
        zero = []  # List of conditions of a == 0
        ans = {}
        # Check the sign of A under each condition and sort
        for pair in cond:
            limit = self.limits[pair]
            if self.a[pair] > sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[0], limit[1])
                non_zero.append(pair)
            elif self.a[pair] < - sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[1], limit[0])
                non_zero.append(pair)
            # In the case of a == 0, add it to the ZERO list and exclude it from COND.
            else:
                l[pair] = limit[0]
                u[pair] = limit[1]
                zero.append(pair)
        # Calculate the secondary inequality parameters
        for pair in non_zero:
            (p[pair], q[pair]) = ([0] * 3, [0] * 3)
            p[pair][0] = (l[pair] - self.d[pair]) / self.a[pair]
            p[pair][1] = -self.c[pair] / self.a[pair]
            p[pair][2] = -self.b[pair] / self.a[pair]
            q[pair][0] = (u[pair] - self.d[pair]) / self.a[pair]
            q[pair][1] = -self.c[pair] / self.a[pair]
            q[pair][2] = -self.b[pair] / self.a[pair]
        # Find a solution for all combinations
        # It also depends on the positional relationship (Q-P-P-Q) of the two conditions, so it becomes Permutations.
        for pair_1, pair_2 in itertools.permutations(non_zero, 2):
            # Set the secondary inequality and solve it
            r = [0] * 3
            for i in range(3):
                r[i] = q[pair_1][i] - p[pair_2][i]
            ans[pair_1, pair_2] = Interval()
            ans_of_inequality = poly2.solve_inequality(r, '>')
            # If there is no solution restriction
            if not ans_of_inequality:
                raise ValueError('Too tight constraint %s, %s' % pair)
            ans[pair_1, pair_2].set_list(ans_of_inequality)
        # In the case of a == 0, the problem to solve is different (candidate for Zero-Inertia Switching Point)
        ans_zero = {}
        for pair in zero:
            r = [0] * 3
            r[0] = self.d[pair] - u[pair]
            r[1] = self.c[pair]
            r[2] = self.b[pair]
            tmp_u = Interval()
            tmp_u.set_list(poly2.solve_inequality(r, '<'))
            r[0] = self.d[pair] - l[pair]
            tmp_l = Interval()
            tmp_l.set_list(poly2.solve_inequality(r, '>'))
            ans_zero[pair] = tmp_l * tmp_u
        # Take a set of solutions
        final = Interval([float('-inf'), float('inf')])
        for pair_1 in non_zero:
            for pair_2 in non_zero:
                if pair_1 == pair_2:
                    continue
                final = final * ans[pair_1, pair_2]
        # Accumulation of the solution space when a == 0
        for pair in zero:
            final = final * ans_zero[pair]
        lst = final.get_list()
        self.mvc = lst[0][1]
        return self.mvc

    def calc_zero_inertia_sv(self, pair):
        u"""uReturns the restricted SV assuming that the current status (SD, SV) that was updated n with pdate () is ZERO INERTIA SP.

        Args:
            pair (Tuple): Constation pair (name, type)
        Return:
            sv (Float): Maximum value of SV that can be taken
        """
        r = [self.d[pair] - self.limits[pair][1], self.c[pair], self.b[pair]]
        tmp_u = Interval()
        tmp_u.set_list(poly2.solve_inequality(r, '<'))
        r[0] = self.d[pair] - self.limits[pair][0]
        tmp_l = Interval()
        tmp_l.set_list(poly2.solve_inequality(r, '>'))
        ans_zero = tmp_l * tmp_u
        ans = ans_zero.get_list()
        return ans[0][1]
