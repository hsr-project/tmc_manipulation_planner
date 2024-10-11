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
#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""Natural Cubic Spline."""

import numpy as np


class NaturalCubicSpline(object):
    def __init__(self, x, y):
        self._x = x
        self._y = y

        n = len(x) - 1

        x_diff = np.diff(x)
        y_diff = np.diff(y)

        h = x_diff
        alpha = 3.0 * (y_diff[1:]) / h[1:] - 3.0 * y_diff[:-1] / h[:-1]
        alpha = np.insert(alpha, 0, 0)

        lst = np.ones(n + 1)
        u = np.zeros(n + 1)
        z = np.zeros(n + 1)

        for i in range(1, n):
            lst[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * u[i - 1]
            u[i] = h[i] / lst[i]
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / lst[i]

        b = np.zeros(n + 1)
        c = np.zeros(n + 1)
        d = np.zeros(n + 1)

        for i in range(n - 1, -1, -1):
            c[i] = z[i] - u[i] * c[i + 1]
            b[i] = y_diff[i] / h[i] - h[i] * (c[i + 1] + 2.0 * c[i]) / 3.0
            d[i] = (c[i + 1] - c[i]) / (3.0 * h[i])
        self._a = [y, b, c, d]

    def __call__(self, x, der=0):
        if not (self._x[0] - 1e-5 <= x <= self._x[-1] + 1e-5):
            return None
        index = np.where(self._x <= x)[0][-1]
        if index == len(self._x) - 1:
            index = len(self._x) - 2
        xi = x - self._x[index]
        if der == 0:
            return sum([self._a[i][index] * (xi ** i) for i in range(4)])
        elif der == 1:
            return sum([(i + 1) * self._a[i + 1][index] * (xi ** i) for i in range(3)])
        elif der == 2:
            return sum([(i + 1) * (i + 2) * self._a[i + 2][index] * (xi ** i) for i in range(2)])
