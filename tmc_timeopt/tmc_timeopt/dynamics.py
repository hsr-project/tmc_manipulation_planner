# !/usr/bin/env python
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
u"""Interface module for the dynamics required by TOPP."""

import itertools
import math
import sys

from tmc_timeopt.interval import Interval
import tmc_timeopt.poly2 as poly2


class Dynamics(object):
    u"""Base class defining the dynamics required by TOPP."""

    def __init__(self, target):
        u"""Initialize this class.

        Args:
            target(Target): Target of dynamics.
        """
        self.target = target
        self.limits = {}
        self.get_mvc = self.get_mvc_accel_effort
        # dict of dynamics parameters
        (self.a, self.b, self.c, self.d) = ({}, {}, {}, {})

    def set_limit(self, name, limit_type, limit):
        u"""Set constraints such as torque.

        Args:
            name (str): Name of the output variable.
            limit_type (str): Type of constraint ('effort', 'ZMP', etc).
            limit (tuple): (lower bound, upper bound).
        """
        self.limits[name, limit_type] = limit

    def update(self):
        u"""Retrieve the dynamics parameters (a, b, c, d) of Target and update member variables."""
        self.target.update_dynamics()
        (self.a, self.b, self.c, self.d) = self.target.get_dynamics()

    def calc_accel_limit(self, sv):
        u"""Calculate the upper and lower bounds of trajectory acceleration under dynamics constraints.

        Args:
            sv (float): Velocity of s on the trajectory.
        Return:
            (l, u): (lower bound, upper bound) of acceleration.
        """
        (sa_min, sa_max) = ({}, {})

        # If sv is an extremely large value, set it to inf to avoid exceptions with sv**2.
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
        # Return the intersection of the upper and lower bounds.
        (lst, u) = (float('-inf'), float('inf'))
        for pair in self.limits:
            lst = max(lst, sa_min[pair])
            u = min(u, sa_max[pair])
        return (lst, u)

    def get_mvc_accel(self):
        u"""Return a point on the MVC (Maximum Velocity Curve) calculated from acceleration constraints.

        update() must be called to set the point beforehand.

        Limited the original get_mvc_accel_effort constraint to acceleration constraints.
        In acceleration control, ci == 0 and di == 0, so it can be calculated quickly.

        Return:
            mvc (float): Velocity of s (sv) at a point on the MVC.
        """
        (u, l) = ({}, {})
        cond = [pair for pair in self.limits
                if pair[1] == 'acceleration']
        non_zero = []  # List of conditions where a != 0.
        rlst = []

        # Check the sign of a for each condition and classify.
        for pair in cond:
            limit = self.limits[pair]
            if self.a[pair] > sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[0], limit[1])
                non_zero.append(pair)
            elif self.a[pair] < - sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[1], limit[0])
                non_zero.append(pair)
            # If a == 0, add to the zero list and exclude from cond.
            else:
                l[pair] = limit[0]
                u[pair] = limit[1]
                r = [-self.d[pair] + u[pair], -self.c[pair], -self.b[pair]]
                rlst.append(r)
                r = [self.d[pair] - l[pair], self.c[pair], self.b[pair]]
                rlst.append(r)

        # Calculate the parameters of the quadratic inequality.
        p = {pair: [(l[pair] - self.d[pair]) / self.a[pair],
                    -self.c[pair] / self.a[pair],
                    -self.b[pair] / self.a[pair]]
             for pair in non_zero}
        q = {pair: [(u[pair] - self.d[pair]) / self.a[pair],
                    -self.c[pair] / self.a[pair],
                    -self.b[pair] / self.a[pair]]
             for pair in non_zero}
        # Determine the solution space for all combinations.
        # Depends on the positional relationship of two conditions (q-p or p-q), so it becomes permutations.
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
        u"""Return a point on the MVC (Maximum Velocity Curve) calculated from torque constraints.

        update() must be called to set the point beforehand.

        when ai > 0
        sa_min_i < (l_limit_i - bi * sv**2 - ci * sv - di) / ai
        sa_max_i < (u_limit_i - bi * sv**2 - ci * sv - di) / ai

        when ai < 0
        sa_min_i < (u_limit_i - bi * sv**2 - ci * sv - di) / ai
        sa_max_i < (l_limit_i - bi * sv**2 - ci * sv - di) / ai

        However, sa_min_i <= sa_max_i.
        satisfying all of them,

        when ai == 0
        l_limit_i < bi * sv**2 + ci * sv + di
        u_limit_i > bi * sv**2 + ci * sv + di

        the maximum value of the first region of sv is the MVC.
        By definition, it can also become Inf.

        Return:
            mvc (float): Velocity of s (sv) at a point on the MVC.
        """
        (u, l, p, q) = ({}, {}, {}, {})
        cond = [pair for pair in self.limits
                if pair[1] == 'effort' or pair[1] == 'acceleration']
        non_zero = []  # List of conditions where a != 0.
        zero = []  # List of conditions where a == 0.
        ans = {}
        # Check the sign of a for each condition and classify.
        for pair in cond:
            limit = self.limits[pair]
            if self.a[pair] > sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[0], limit[1])
                non_zero.append(pair)
            elif self.a[pair] < - sys.float_info.epsilon:
                (l[pair], u[pair]) = (limit[1], limit[0])
                non_zero.append(pair)
            # If a == 0, add to the zero list and exclude from cond.
            else:
                l[pair] = limit[0]
                u[pair] = limit[1]
                zero.append(pair)
        # Calculate the parameters of the quadratic inequality.
        for pair in non_zero:
            (p[pair], q[pair]) = ([0] * 3, [0] * 3)
            p[pair][0] = (l[pair] - self.d[pair]) / self.a[pair]
            p[pair][1] = -self.c[pair] / self.a[pair]
            p[pair][2] = -self.b[pair] / self.a[pair]
            q[pair][0] = (u[pair] - self.d[pair]) / self.a[pair]
            q[pair][1] = -self.c[pair] / self.a[pair]
            q[pair][2] = -self.b[pair] / self.a[pair]
        # Determine the solution space for all combinations.
        # Depends on the positional relationship of two conditions (q-p or p-q), so it becomes permutations.
        for pair_1, pair_2 in itertools.permutations(non_zero, 2):
            # Set and solve the quadratic inequality.
            r = [0] * 3
            for i in range(3):
                r[i] = q[pair_1][i] - p[pair_2][i]
            ans[pair_1, pair_2] = Interval()
            ans_of_inequality = poly2.solve_inequality(r, '>')
            # If there are constraints with no solution.
            if not ans_of_inequality:
                raise ValueError('Too tight constraint %s, %s' % pair)
            ans[pair_1, pair_2].set_list(ans_of_inequality)
        # If a == 0, the problem to solve is different (candidate for zero-inertia switching point).
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
        # Take the intersection of the solution spaces.
        final = Interval([float('-inf'), float('inf')])
        for pair_1 in non_zero:
            for pair_2 in non_zero:
                if pair_1 == pair_2:
                    continue
                final = final * ans[pair_1, pair_2]
        # Intersection of solution spaces when a == 0.
        for pair in zero:
            final = final * ans_zero[pair]
        lst = final.get_list()
        self.mvc = lst[0][1]
        return self.mvc

    def calc_zero_inertia_sv(self, pair):
        u"""Assuming the current state (sd, sv) updated by update() is a zero inertia sp, return the sv of the constraint.

        Args:
            pair (tuple): Pair of constraints (name, type).
        Return:
            sv (float): Maximum possible value of sv.
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
