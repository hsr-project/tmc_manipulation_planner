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
u"""TOPP class sample."""

import time

import matplotlib.pyplot as plt

import numpy as np

from numpy.random import random

from tmc_timeopt.target import Target
from tmc_timeopt.timeopt import Timeopt
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt.trajectory import TrajectoryDict


class MyTarget(Target):
    u"""My Target class."""

    def __init__(self):
        self.names = ['joint0',
                      'joint1',
                      'joint2']
        self.point = {}

    def update_kinematics(self, point):
        u"""Update TARGET athletic.

        Args:
            Point: DICT of state amount
        """
        self.point = point

    def update_dynamics(self):
        u"""I don't do anything because it is an acceleration level restraint."""
        pass

    def get_dynamics(self):
        u"""Return the dynamic spalameter (A, B, C, D).

        Each (a, b, c, d) is a key ('variable name', 'restriction type'), respectively, and the value is Value.
        """
        (a, b, c, d) = ({}, {}, {}, {})
        for name in self.names:
            a[name, 'acceleration'] = self.point[name][1]
            b[name, 'acceleration'] = self.point[name][2]
            c[name, 'acceleration'] = 0
            d[name, 'acceleration'] = 0
        return a, b, c, d


if __name__ == "__main__":
    VEL_LIMIT = {'joint0': 0.1, 'joint1': 0.2, 'joint2': 0.3}
    ACC_LIMIT = {'joint0': 0.1, 'joint1': 0.1, 'joint2': 0.1}

    target = MyTarget()
    timeopt = Timeopt(target)
    kinematics = timeopt.kinematics()
    dynamics = timeopt.dynamics()
    joint_names = target.names

    for joint in joint_names:
        dynamics.set_limit(joint, 'acceleration',
                           (-ACC_LIMIT[joint], ACC_LIMIT[joint]))
        kinematics.set_limit(joint,
                             'velocity',
                             (-VEL_LIMIT[joint],
                              VEL_LIMIT[joint]))
    _POINTS = 30
    traj = TrajectoryDict(_POINTS)
    for name in target.names:
        traj.append(name, NaturalCubicSplineTrajectory)

    rand = random()
    for i in range(_POINTS + 1):
        traj['joint1'][i] = (np.sin(0.1 * i), 0.0, 0)
        traj['joint2'][i] = (np.cos(0.1 * i), 0.0, 0)
        traj['joint3'][i] = (0.3 * np.sin(0.4 * i + rand), 0.0, 0)

    timeopt.set_trajectory(traj)
    start = time.time()
    timeopt.preprocess(0.1)
    timeopt.update()
    elapsed_time = time.time() - start
    print("elapsed_time:{0}".format(elapsed_time) + "[sec]")
    timeopt.plot_s_sdot_result()

    opt_traj = timeopt.get_optimal_trajectory()
    joint_names = target.names

    times = np.array([point[0] for point in opt_traj])

    fig, (fig1, fig2, fig3) = plt.subplots(nrows=3, figsize=(10, 4))
    for joint in joint_names:
        pos = np.array([point[1][joint][0] for point in opt_traj])
        fig1.plot(times, pos, label=joint)
    fig1.grid(True)
    fig1.legend()

    for joint in joint_names:
        vel = np.array([point[1][joint][1] for point in opt_traj])
        fig2.plot(times, vel, label=joint)
    fig2.grid(True)
    fig2.legend()

    for joint in joint_names:
        acc = np.array([point[1][joint][2] for point in opt_traj])
        fig3.plot(times, acc, label=joint)

    fig3.grid(True)
    fig3.legend()

    plt.xlabel('Time')
    plt.show()
