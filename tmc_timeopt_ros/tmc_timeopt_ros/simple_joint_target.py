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
from tmc_timeopt.target import Target


class SimpleJointTarget(Target):
    u"""Target for joint orbital filter only for acceleration level restraint"""

    def __init__(self, joint_names=[]):
        self.names = joint_names
        self.point = {}

    def update_kinematics(self, point):
        u"""Update Kinematics.

        Args:
            point: DICT of state amount
        """
        self.point = point

    def update_dynamics(self):
        u"""I don't do anything because it is an acceleration level restraint."""
        pass

    def get_dynamics(self):
        u"""Return the dynamic spalameter (A, B, C, D).

        Returns:
            (a, b, c, d): ('variables' name', 'restriction type') is key and the value is value.
        """
        (a, b, c, d) = ({}, {}, {}, {})
        for name in self.names:
            a[name, 'acceleration'] = self.point[name][1]
            b[name, 'acceleration'] = self.point[name][2]
            c[name, 'acceleration'] = 0
            d[name, 'acceleration'] = 0
        return a, b, c, d
