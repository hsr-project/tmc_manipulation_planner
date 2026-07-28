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
u"""Geometry calculation module."""


class Kinematics(object):
    u"""Base class for handling kinematics constrained to an orbit."""

    def __init__(self, target):
        u"""Initialize with a given target.

        Args:
            target Calculation target
        """
        self.target = target
        self.traj = {}
        self.traj_memo = {}
        self.limits = {}
        self.curr = {}

    def set_limit(self, name, limit_type, limit):
        u"""Set kinematic constraints.

        Args:
            name (str): Name of the variable
            limit_type (str): Type of constraint
            limit (tuple): (Lower limit, Upper limit)
        """
        if not isinstance(limit, tuple):
            raise TypeError('limit is not tuple')
        if name not in self.target.names:
            raise ValueError('No input name variables %s.' % name)
        self.limits[name, limit_type] = limit

    def set_trajectory(self, traj):
        u"""Set the command space trajectory.

        Args:
            traj (dict of Trajectory): Command space trajectory
        """
        self.traj = traj
        self.traj_memo = {}
        # Dictionary of current state
        self.curr = {}
        for name in self.traj.keys():
            self.curr[name] = (0, 0, 0)

    def get_current_point(self):
        u"""Return the current state point.

        Return:
            curr (Point): Returns the current state of each joint
        """
        return self.curr

    def update(self, sd):
        u"""Update kinematics at point sd on the trajectory. The current state of the target is updated.

        Args:
            sd(float): Position on the trajectory (specified by parameter s)
        """
        # Due to calculation errors, sd > self.traj.length may occur
        sd = min(sd, self.traj.length)

        # Retrieve interpolation point on the trajectory
        if sd in self.traj_memo:
            self.curr = self.traj_memo[sd]
            self.target.update_point(self.traj_memo[sd])
        else:
            self.curr = {name: self.traj[name](sd) for name in self.traj}
            self.target.update_kinematics(self.curr)
            self.traj_memo[sd] = {name: self.curr[name][:]
                                  for name in self.traj}

    def pre_calc_traj(self, sd_seq, step):
        u"""Pre-calculation of traj"""
        temp = [self.traj[name].calc(sd_seq, step) for name in self.traj]
        self.traj_memo = {x[0]: {y[0]: y[1] for y in zip(self.traj, x[1:])}
                          for x in zip(sd_seq, *temp)}
        for x in sd_seq:
            self.curr = self.traj_memo[x]
            self.target.update_kinematics(self.curr)

    def get_vlc(self):
        u"""Return a point on the curve VLC (Velocity Limit Curve) constrained by velocity. Call update first.

        Return sv_max(float): s velocity of the point on VLC
        """
        pairs = [pair for pair in self.limits if pair[1] == 'velocity']
        sv_max = float('inf')
        for pair in pairs:
            limit = self.limits[pair]
            if self.curr[pair[0]][1] > 0:
                sv_max = min(sv_max, limit[1] / self.curr[pair[0]][1])
            elif self.curr[pair[0]][1] < 0:
                sv_max = min(sv_max, limit[0] / self.curr[pair[0]][1])
        return sv_max

    def get_state(self, sd, sv, sa):
        u"""Return the current state point.

        Args:
            sd (float): s on the trajectory
            sv (float): s velocity on the trajectory
            sa (float): s acceleration on the trajectory
        Return:
            state (dict): [Position, Velocity, Acceleration] of each joint. Derivatives are time derivatives.
        """
        state = {}
        self.update(sd)
        for name, point in self.curr.items():
            state[name] = [0, 0, 0]
            state[name][0] = point[0]
            state[name][1] = point[1] * sv
            state[name][2] = point[1] * sa + point[2] * sv ** 2
        return state

    def calc_accel_limit(self, sv):
        u"""Calculate the upper and lower limits of trajectory acceleration under constraints.

        Args:
            sv (float): s velocity of the point to calculate
        Return:
            [sa_min, sa_max] : Lower and upper limits of s acceleration
        """
        (sa_min, sa_max) = (float('-inf'), float('inf'))
        accel_limits = [
            pair for pair in self.limits if pair[1] == 'acceleration']
        for (name, limit_type) in accel_limits:
            limit = self.limits[name, limit_type]
            if self.curr[name][1] > 0:
                sa_min = max(
                    (limit[0] - self.curr[name][2] * sv ** 2) / self.curr[name][1], sa_min)
                sa_max = min(
                    (limit[1] - self.curr[name][2] * sv ** 2) / self.curr[name][1], sa_max)
            elif self.curr[name][1] < 0:
                sa_min = max(
                    (limit[1] - self.curr[name][2] * sv ** 2) / self.curr[name][1], sa_min)
                sa_max = min(
                    (limit[0] - self.curr[name][2] * sv ** 2) / self.curr[name][1], sa_max)
        return [sa_min, sa_max]
