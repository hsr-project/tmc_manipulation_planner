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
u"""A function to solve the secondary inequality."""

from math import sqrt


def solve_equality(a):
    u"""Secondary equation a [0]+a [1] x+a [2] x^2 == 0 and return it.

    The solution is [0, 2], etc.If there is no solution, return []
    Args:
        a [Double]: Cot coefficient of secondary equation
    Return:
        List: List of solutions
    """
    u = a[1] * a[1] - 4 * a[2] * a[0]
    if u < 0:
        return []
    x = [(-a[1] - sqrt(u)) / (2 * a[2]), (-a[1] + sqrt(u)) / (2 * a[2])]
    x.sort()
    return x


def solve_inequality(a, s):
    u"""Secondary inequality a [0]+a [1] x+a [2] x^2 <> 0.

    S is inequal issue '>' or '<'.The solution is [0, 2] (0, 2), [[-inf, 0], [2, inf]] (x <0, x> 2).
    If there is no solution, return []
    Args:
        a [Double]: Cot coefficient of secondary equation
        s Sting: '>' OR '<'
    Return:
        List that represents the List section
    """
    if a[2] == 0:
        # In the case without a variable
        if a[1] == 0:
            if (s == '>' and a[0] > 0) or (s == '<' and a[0] < 0):
                return [float('-inf'), float('inf')]
            else:
                return []
        # In the case of primary inhabitants
        if (s == '>' and a[1] > 0) or (s == '<' and a[1] < 0):
            return [-a[0] / a[1], float('inf')]
        if (s == '<' and a[1] > 0) or (s == '>' and a[1] < 0):
            return [float('-inf'), -a[0] / a[1]]

    x = solve_equality(a)
    if not x:
        if (s == '>' and a[2] > 0) or (s == '<' and a[2] < 0):
            return [float('-inf'), float('inf')]
        else:
            return []
    if (s == '>' and a[2] > 0) or (s == '<' and a[2] < 0):
        return [[float('-inf'), x[0]], [x[1], float('inf')]]
    else:
        return [x[0], x[1]]
