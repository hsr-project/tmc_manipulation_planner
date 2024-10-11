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
"""Target Interface."""

import matplotlib
# Change AGG to TKAGG when Debug
matplotlib.use('Agg')

# Subsequent IMPORT is written after matplotlib.use, so pass through with noqa.
import matplotlib.pyplot as plt  # noqa


class Target(object):
    """Target Interface class."""

    def update_kinematics(self, point):
        u"""Update Kinematics."""
        self.point = point

    def update_point(self, point):
        u"""Update Target kinematics using the calculated point."""
        self.point = point

    def update_dynamics(self):
        u"""Calculate the dynamic spalameter (A, B, C, D)."""
        pass

    def get_dynamics(self):
        u"""Return the dynamic spalameter (A, B, C, D).

        (a, b, c, d), each is a key ('variable name', 'constraintic type'), respectively, and the value is Value.
        """
        pass

    def plot_trajectory(self, traj, step=0.01):
        u"""Plot orbit."""
        x = [i * step for i in range(int(traj.length / step) + 1)]

        (y0, y1, y2) = ({}, {}, {})
        for name in traj:
            (y0[name], y1[name], y2[name]) = ([], [], [])
        for s in x:
            point = traj(s)
            self.update(point)
            for name in traj:
                y0[name].append(point[name][0])
                y1[name].append(point[name][1])
                y2[name].append(point[name][2])

        for name in traj:
            plt.title(name)
            plt.plot(x, y0[name], '-', label='%s[0]' % name)
            plt.plot(x, y1[name], '-', label='%s[1]' % name)
            plt.plot(x, y2[name], '-', label='%s[2]' % name)
            plt.legend()
            plt.show()
