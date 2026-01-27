# !/usr/bin/env python
# Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
u"""Module to calculate the minimum time control problem (TOPP)."""

from __future__ import print_function

from math import isinf
from math import sqrt
import os

import matplotlib
# Change Agg to tkAgg for debugging
matplotlib.use('Agg')

# Subsequent imports are written after matplotlib.use, so pass with noqa
import matplotlib.pyplot as plt  # noqa

from tmc_timeopt.dynamics import Dynamics  # noqa
from tmc_timeopt.kinematics import Kinematics  # noqa
from tmc_timeopt.trajectory import TrajectoryDict, _NEW_SCIPY  # noqa


def _plot_step(func):
    u"""Decorator to plot the processing of one-step integration.

    For debugging.
    """
    def wrapper(*args, **kwds):
        self = args[0]
        (sd0, sv0, ds) = args[1:4]
        result = func(*args, **kwds)
        print("%s%s: %s" % (func.__name__, args[1:], result))
        (r, sd1, sv1, sa0) = result
        plt.clf()
        # Plot the results of preprocessing
        plt.plot(self._sd, self._vlc, '-', label="vlc")
        plt.plot(self._sd, self._mvc, '-', label="mvc")
        plt.quiver(self._sd, self._mvc,
                   [self._ds] * self._size, self._mvc_sa_min,
                   scale_units='xy', angles='xy', scale=1, width=0.006)
        plt.quiver(self._sd, self._mvc,
                   [self._ds] * self._size, self._mvc_sa_max,
                   scale_units='xy', angles='xy', scale=1, width=0.006)
        # Plot the results
        plt.plot(self._sd, self._sv, '-', label="sv")
        plt.plot([sd0], [sv0], 'o', label="curr")
        xdir = ds
        ydir = self._sa_min * self._dt_min
        plt.quiver(sd0, sv0, xdir, ydir, scale_units='xy', angles='xy', scale=1, width=0.006)
        ydir = self._sa_max * self._dt_max
        plt.quiver(sd0, sv0, xdir, ydir, scale_units='xy', angles='xy', scale=1, width=0.006)
        # plt.xlim(sd0 - 0.3, sd0 + 0.3)
        plt.xlim(0, self._traj.length)
        plt.ylim(max(0, sv0 - 0.3), sv0 + 0.3)
        # plt.ylim(0, 30)
        plt.ylim(0, 20.0)
        plt.legend()
        # plt.draw()
        plt.pause(0.01)
        # plt.show()
        return result
    return wrapper


class Timeopt(object):
    u"""Class to calculate the minimum time control problem (TOPP)."""

    # Maximum number of divisions for divided integration
    _INTEGRATE_DIV_NUM = 4
    # Lower sensitivity for tangent point as calculations are critical
    _LOW_SENSITIVITY_COEFF = 0.95
    _VLC_MERGIN = 0.001
    _MVC_MERGIN = 0.001
    _CROSS_MERGIN = 0.0

    # Minimum time integration width
    _MINIMUM_DT = 0.001

    # Precision of switching point
    _SP_ACCURACY = 1e-4

    # Threshold for double solutions
    _DOUBLE_EPS = 1e-3

    def __init__(self, target):
        u"""Initialize the class by providing the target.

        Args:
            target: Target to be considered
            step: Step width of trajectory parameter s
        """
        self._target = target
        self._kinematics = Kinematics(self._target)
        self._dynamics = Dynamics(self._target)
        # Array size of the entire trajectory
        self._size = 0
        # Buffer for parameters a, b, c, d
        (self._a_buff, self._b_buff, self._c_buff, self._d_buff) = (
            {}, {}, {}, {})
        # sa_min, sa_max
        self._sa_min = 0
        self._sa_max = 0
        self._dt_min = 0
        self._dt_max = 0
        self._fwd_singular = False
        self._bkw_singular = False

    def kinematics(self):
        return self._kinematics

    def dynamics(self):
        return self._dynamics

    def set_trajectory(self, trajectory):
        u"""Set the spatial trajectory.

        Args:
            trajectory (TrajectoryDict): Spatial trajectory
        """
        if not isinstance(trajectory, TrajectoryDict):
            raise TypeError('TrajectoryDict is needed.')
        self._traj = trajectory
        self._kinematics.set_trajectory(trajectory)

    def preprocess(self, step=0.1):
        u"""Perform preprocessing for minimum time control.

           1. Reserve the area
           2. Calculate MVC and VLC
           3. Calculate parameters of dynamics
        Args:
            step (float): Step width of s used for exploration
        Note:
            It is efficient & simple to calculate MVC for all discrete points first
        """
        self._ds = step
        # Get the length of the trajectory
        self._size = int((self._traj.length - 1) / float(step)) + 1
        # Calculate self._sd
        self._sd = [float(i) * self._ds for i in range(self._size)]
        # Pre-calculate traj
        if _NEW_SCIPY:
            # It's faster not to pre-calculate when not using scipy
            self._kinematics.pre_calc_traj(self._sd, step)
        # Reserve area for processing
        (self._mvc, self._sa_mvc, self._vlc) = ([0] * self._size, [0] * self._size, [0] * self._size)
        self._mvc_dt = [0] * self._size
        self._mvc_sa_min = [0] * self._size
        self._mvc_sa_max = [0] * self._size
        # List of actual solutions
        self._sv = [float('inf')] * self._size
        self._sv[0] = self._sv[-1] = 0
        self._sa = [0] * self._size
        self._fw = [True] * self._size

        # Reserve area for parameters
        for pair in self._dynamics.limits:
            self._a_buff[pair] = [0] * self._size
            self._b_buff[pair] = [0] * self._size
            self._c_buff[pair] = [0] * self._size
            self._d_buff[pair] = [0] * self._size

        # Find MVC and VLC for the entire section
        for i, sd in enumerate(self._sd):
            # Update kinematics, dynamics
            self._kinematics.update(sd)
            self._dynamics.update()

            # Calculate MVC
            self._mvc[i] = self._dynamics.get_mvc()
            # Find upper and lower limits of acceleration on MVC
            (self._mvc_sa_min[i], self._mvc_sa_max[i]) = self._dynamics.calc_accel_limit(self._mvc[i])
            self._sa_mvc[i] = self._mvc_sa_min[i]
            dt = self.__calc_dt(self._mvc[i], self._mvc_sa_min[i], self._ds)
            self._mvc_sa_min[i] *= dt
            dt = self.__calc_dt(self._mvc[i], self._mvc_sa_max[i], self._ds)
            self._mvc_sa_max[i] *= dt

            # Calculate VLC
            self._vlc[i] = self._kinematics.get_vlc()
            # Save constraint parameters (a, b, c, d)
            for pair in self._dynamics.limits:
                self._a_buff[pair][i] = self._dynamics.a[pair]
                self._b_buff[pair][i] = self._dynamics.b[pair]
                self._c_buff[pair][i] = self._dynamics.c[pair]
                self._d_buff[pair][i] = self._dynamics.d[pair]

    def update(self):
        u"""Solve the TOPP algorithm to generate the shortest velocity trajectory."""
        # Integrate backward from the endpoint
        self.__integrate_backward_segment(self._size - 1)

        # Integrate forward from the starting point
        curr = 0
        while curr < self._size:
            # Forward integration until it stops
            result = self.__integrate_forward_segment(curr)
            (r, index) = result
            if r == 'END':
                break
            # Search for switching points forward
            result = self.__search_switching_point(index)
            (r, curr) = result
            # When there are no valid switching points
            if r == 'NG':
                raise RuntimeError('No valid switching point found.')
        # Recalculate the trajectory and finish
        self.__recalc_trajectory()

    def get_optimal_trajectory(self):
        u"""Get the optimal trajectory after calculation with update.

        Return:
           trajectory: list of (time, state)
        """
        trajectory = []
        for n in range(0, self._size):
            (sd, sv, sa) = (self._sd[n], self._sv[n], self._sa[n])
            time = self._time[n]
            state = self._kinematics.get_state(sd, sv, sa)
            trajectory.append((time, state))
        return trajectory

    def __integrate_forward_segment(self, curr):
        u"""Integrate forward as much as possible from sd[curr] and return the stop factor and stop position.

        Args:
            curr (int): Index of the starting position of integration
        Retrun:
            tuple: Returns a tuple of (stop factor, stop index).
            'END': Integration completed to the end
            'MVC': Stopped exceeding MVC
            'VLC': Stopped failing to meet VLC
        """
        while curr < self._size - 1:
            (sd0, sv0) = (self._sd[curr], self._sv[curr])
            # Integrate while changing the integration width
            result = self.__integrate_forward_adaptive(sd0, sv0, self._ds)
            (r, sd1, sv1, sa0) = result
            # Integration stopped
            if r != 'OK':
                return (r, curr)
            # Intersection judgment with the backward trajectory
            if curr + 1 < self._size:
                if sv1 >= self._sv[curr + 1] - self._CROSS_MERGIN:
                    # It seems to be a bug that sa is not updated here in the code of the research center
                    self._sa[curr] = sa0
                    break
            self._sa[curr] = sa0
            curr += 1
            self._sv[curr] = sv1
        return ('END', curr)

    def __integrate_forward_adaptive(self, sd0, sv0, step):
        u"""Integrate forward by step width from (sd0, sv0).

        If integration fails, try while dividing the integration width.
        Args:
            sd0 (float): s of the integration start point
            sv0 (float): Velocity of s of the integration start point
        Return:
            tuple: Returns a tuple of (stop factor, stop state).
            'OK': Integration successful
            'MVC': Stopped exceeding MVC
            'VLC': Stopped failing to meet VLC
        """
        nxt = 0
        sa0_sv = None
        for div_num in [4 ** n for n in range(self._INTEGRATE_DIV_NUM)]:
            result, nxt, prev = self.__integrate_forward_divide(sd0, sv0, step, div_num, nxt)
            (r, sd1, sv1, sa0) = result
            if sa0 is None:
                sa0 = sa0_sv
                result = (r, sd1, sv1, sa0)
            else:
                sa0_sv = sa0
            if r == 'OK':
                break
            if nxt != 0:
                nxt = (nxt - 1) * 4
                sd0, sv0 = prev
        return result

    def __integrate_forward_divide(self, sd0, sv0, step, div_num, start=0):
        u"""Integrate forward by width divided by div_num from (sd0, sv0).

        Args:
            sd0 (float): s of the integration start point
            sv0 (float): Velocity of s of the integration start point
        Return:
            tuple: Returns a tuple of (stop factor, stop state).
            'OK': Integration successful
            'MVC': Stopped exceeding MVC
            'VLC': Stopped failing to meet VLC
        """
        ds = step / div_num
        sa0 = None
        if start == 0:
            result = self.__integrate_forward_step(
                sd0, sv0, ds, self._fwd_singular)
            (r, sd, sv, sa0) = result
            if r in ['MVC', 'VLC']:
                return result, 0, (sd0, sv0)
            prev = (sd0, sv0)
            start = 1
        else:
            prev = (sd0, sv0)
            sd, sv = prev
        for i in range(start, div_num):
            result = self.__integrate_forward_step(sd, sv, ds)
            (r, sd, sv, sa) = result
            if r in ['MVC', 'VLC']:
                return result, i, prev
            prev = (sd, sv)
        return ('OK', sd, sv, sa0), None, prev

    # @_plot_step
    def __integrate_forward_step(self, sd, sv, step, singular_flag=False):
        u"""Integrate forward by step width for one step from (sd, sv).

        Args:
            sd (float): s of the integration start point
            sv (float): Velocity of s of the integration start point
            step (float): Integration width
            singular_flag (bool): Set to True when integrating from zero-inertia switching point
        Return:
            tuple: Returns a tuple of (stop factor, stop state).
            'OK': Integration successful
            'MVC': Failed exceeding MVC
            'VLC': Failed failing to meet VLC
        """
        # Update dynamics
        self._kinematics.update(sd)
        self._dynamics.update()
        # Calculate current acceleration limits
        if singular_flag:
            [self._sa_min, self._sa_max] = [0, 0]
        else:
            [self._sa_min, self._sa_max] = self.__calc_accel_limit(sv)
        # Calculate integration time
        self._dt_min = self.__calc_dt(sv, self._sa_min, step)
        self._dt_max = self.__calc_dt(sv, self._sa_max, step)
        # Update state
        sd_next = sd + sv * self._dt_max + 0.5 * self._sa_max * self._dt_max ** 2
        sv_next = sv + self._sa_max * self._dt_max
        # Constraints are too tight to be feasible
        if sv_next < 0:
            return ('MVC', sd_next, sv_next, self._sa_max)
        # Update dynamics at candidate points
        mvc, vlc = self.__get_mvc_vlc(sd_next, update_flg=True)
        # Determine if it exceeded mvc
        if sv_next > mvc + self._MVC_MERGIN:
            return ('MVC', sd_next, sv_next, self._sa_max)
        # Shape to not exceed VLC
        if sv_next > vlc:
            dt = 2.0 * step / (vlc + sv)
            sa = (vlc - sv) / dt
            sv_next = vlc

            # Cannot follow VLC
            if sa < self._sa_min:
                return ('VLC', sd_next, sv_next, self._sa_min)
            else:
                self._sa_max = sa
        return ('OK', sd_next, sv_next, self._sa_max)

    def __integrate_backward_segment(self, curr):
        u"""Integrate backward as much as possible from sd[curr] and return the stop factor and stop position.

        Args:
            curr (int): Index of the starting position of integration
        Retrun:
            tuple: Returns a tuple of (stop factor, stop index).
            'END': Integration completed to the end
            'MVC': Stopped exceeding MVC
            'VLC': Stopped failing to meet VLC
        """
        while curr > 0:
            (sd1, sv1) = (self._sd[curr], self._sv[curr])
            result = self.__integrate_backward_adaptive(sd1, sv1, self._ds)
            (r, sd0, sv0, sa1) = result
            # Integration stopped
            if r != 'OK':
                return (r, curr)
            # Intersection judgment with the forward trajectory
            if sv0 >= self._sv[curr - 1] - self._CROSS_MERGIN:
                if float('inf') in self._sv:
                    if self._sv.index(float('inf')) >= curr - 1:
                        self._sa[curr - 1] = sa1
                        break
                else:
                    self._sa[curr - 1] = sa1
                    break
            self._sa[curr - 1] = sa1
            curr -= 1
            self._sv[curr] = sv0
        return ('END', curr)

    def __integrate_backward_adaptive(self, sd1, sv1, step):
        u"""Integrate backward by step width from (sd1, sv1).

        If integration fails, try while dividing the integration width.
        Args:
            sd1 (float): s of the integration start point
            sv1 (float): Velocity of s of the integration start point
        Return:
            tuple: Returns a tuple of (stop factor, stop state).
            'OK': Integration successful
            'MVC': Stopped exceeding MVC
            'VLC': Stopped failing to meet VLC
        """
        nxt = 0
        sa1_sv = None
        for div_num in [4 ** n for n in range(self._INTEGRATE_DIV_NUM)]:
            result, nxt, prev = self.__integrate_backward_divide(sd1, sv1, step, div_num, nxt)
            (r, sd0, sv0, sa1) = result
            if sa1 is None:
                sa1 = sa1_sv
                result = (r, sd0, sv0, sa1)
            else:
                sa1_sv = sa1
            if r == 'OK':
                break
            if nxt != 0:
                nxt = (nxt - 1) * 4
                sd1, sv1 = prev
        return result

    def __integrate_backward_divide(self, sd1, sv1, step, div_num, start=0):
        u"""Integrate backward by width divided by div_num from (sd1, sv1).

        Args:
            sd1 (float): s of the integration start point
            sv1 (float): Velocity of s of the integration start point
        Return:
            tuple: Returns a tuple of (stop factor, stop state).
            'OK': Integration successful
            'MVC': Stopped exceeding MVC
            'VLC': Stopped failing to meet VLC
        """
        ds = step / div_num
        sa1 = None
        if start == 0:
            result = self.__integrate_backward_step(sd1, sv1, ds, self._bkw_singular)
            (r, sd, sv, sa1) = result
            if r in ['MVC', 'VLC']:
                return result, 0, (sd1, sv1)
            prev = (sd1, sv1)
            start = 1
        else:
            prev = (sd1, sv1)
            sd, sv = prev
        for i in range(start, div_num):
            result = self.__integrate_backward_step(sd, sv, ds)
            (r, sd, sv, sa) = result
            if r in ['MVC', 'VLC']:
                return result, i, prev
            prev = (sd, sv)
        return ('OK', sd, sv, sa1), None, prev

    # @_plot_step
    def __integrate_backward_step(self, sd, sv, step, singular_flag=False):
        u"""Integrate backward by step width for one step from (sd, sv).

        Args:
            sd (float): s of the integration start point
            sv (float): Velocity of s of the integration start point
            step (float): Integration width
            singular_flag (bool): Set to True when integrating from zero-inertia switching point
        Return:
            tuple: Returns a tuple of (stop factor, stop state).
            'OK': Integration successful
            'MVC': Failed exceeding MVC
            'VLC': Failed failing to meet VLC
        """
        # Update dynamics
        self._kinematics.update(sd)
        self._dynamics.update()
        # Calculate current acceleration limits
        if singular_flag:
            [self._sa_min, self._sa_max] = [0, 0]
        else:
            [self._sa_min, self._sa_max] = self.__calc_accel_limit(sv)
        # Backward integration
        self._dt_min = self.__calc_dt_back(sv, self._sa_min, step)
        self._dt_max = self.__calc_dt_back(sv, self._sa_max, step)
        # Update state
        sd_prev = sd - sv * self._dt_min + 0.5 * self._sa_min * self._dt_min ** 2
        sv_prev = sv - self._sa_min * self._dt_min
        # Constraints are too tight to be feasible
        if sv_prev < 0:
            return ('MVC', sd_prev, sv_prev, self._sa_min)
        # Update dynamics at candidate points
        mvc, vlc = self.__get_mvc_vlc(sd_prev, update_flg=True)
        # Determine if it exceeded mvc
        if sv_prev > mvc + self._MVC_MERGIN:
            return ('MVC', sd_prev, sv_prev, self._sa_min)
        # Shape to not exceed VLC
        if sv_prev > vlc:
            dt = 2.0 * step / (vlc + sv)
            sa = (sv - vlc) / dt
            sv_prev = vlc

            # Cannot follow VLC
            if sa > self._sa_max:
                return ('VLC', sd_prev, sv_prev, self._sa_max)
            else:
                self._sa_min = sa
        return ('OK', sd_prev, sv_prev, self._sa_min)

    def __calc_dt(self, sv, sa, ds):
        u"""Calculate integration time width dt for integration width ds (>0) from sv and sa.

        Args:
            sv (float): Velocity of s
            sa (float): Acceleration of s
            ds (float): Integration value of s
        Retrun:
            float: Integration time width dt
        """
        minimum_dt = self._MINIMUM_DT * ds / self._ds
        disciminant = sv ** 2.0 + 2.0 * sa * ds
        if sa == float('inf') or sa == float('-inf'):
            return minimum_dt
        if abs(sa) < self._DOUBLE_EPS:
            if sv == 0:
                return minimum_dt
            return ds / sv
        # When there is no solution
        if (disciminant) < 0:
            if sv > self._DOUBLE_EPS:
                # Return as a double solution as a numerical calculation error if small
                return -sv / sa
            else:
                return minimum_dt
        dt = [(-sv - sqrt(disciminant)) / sa,
              (-sv + sqrt(disciminant)) / sa]
        dt = [t for t in dt if t > 0]
        if dt == []:
            return minimum_dt
        else:
            dt = min(dt)
        return max(dt, minimum_dt)

    def __calc_dt_back(self, sv, sa, ds):
        u"""Calculate integration time width dt for integration width ds (>0) from sv and sa.

           During backward integration
        Args:
            sv (float): Velocity of s
            sa (float): Acceleration of s
            ds (float): Integration value of s
        Retrun:
            float: Integration time width dt
        """
        minimum_dt = self._MINIMUM_DT * ds / self._ds
        disciminant = sv ** 2.0 - 2.0 * sa * ds
        if sa == float('inf') or sa == float('-inf'):
            return minimum_dt
        if abs(sa) < self._DOUBLE_EPS:
            if sv == 0:
                return minimum_dt
            return ds / sv
        # When there is no solution
        if (disciminant) < 0:
            if sv > self._DOUBLE_EPS:
                # Return as a double solution as a numerical calculation error if small
                return sv / sa
            else:
                return minimum_dt
        dt = [(sv - sqrt(disciminant)) / sa, (sv + sqrt(disciminant)) / sa]
        dt = [t for t in dt if t > 0]
        if dt == []:
            return minimum_dt
        else:
            dt = min(dt)
        return max(dt, minimum_dt)

    def __recalc_trajectory(self):
        u"""Integrate from s=0 to the endpoint and update the final state."""
        (self._time, self._span) = ([0] * self._size, [0] * self._size)
        tick = 0
        for n in range(self._size):
            (sv, sa) = (self._sv[n], self._sa[n])
            self._time[n] = tick
            span = self.__calc_dt(sv, sa, self._ds)
            self._span[n] = span
            tick += span

    def __search_switching_point(self, curr):
        u"""Search forward from position curr until a Switching Point is found.

        Args:
            curr (int): Index of the starting position of the search
        Retrun:
            tuple: Returns a tuple of (stop factor, stop position).
            'OK': Found a switching point
            'NG': Switching point not found (usually impossible)
        """
        while curr < self._size - 1:
            # Check Zero-Inertia SP
            for name in self.__check_zero_inertia_point(curr):
                sp = self.__calc_zero_inertia_point(name, curr)
                if sp:
                    (self._fwd_singular, self._bkw_singular) = (True, True)
                    if self.__integrate_from_switching_point(curr, sp):
                        (self._fwd_singular, self._bkw_singular) = (False, False)
                        return ('OK', curr + 1)
                    (self._fwd_singular, self._bkw_singular) = (False, False)

            # Check Trap point
            sp = self.__check_trap_point(curr)
            if sp:
                # Confirm if integration can continue from SP
                res = self.__integrate_from_switching_point(curr, sp)
                if res:
                    return ('OK', curr + 1)

            # Check Tangent point
            sp = self.__check_tangent_point(curr)
            if sp:
                # Confirm if integration can continue from SP
                res = self.__integrate_from_switching_point(curr, sp)
                if res:
                    return ('OK', curr + 1)
            # Check the next point
            curr += 1
        return ('NG', curr)

    def __check_tangent_point(self, curr):
        u"""Check if there is a tangent switching point between curr and curr+1.

        If so, return the exact position using bisection method
        Args:
            curr (int): Index of the position to check
        Retrun:
            tuple: Two points surrounding SP (sd1, sv1, sd2, sv2)
            None: No SP found
        """
        # Calculate acceleration limits
        sa_curr = self._sa_mvc[curr]
        sa_next = self._sa_mvc[curr + 1]
        # Calculate integration time
        dt_curr = self.__calc_dt(self._mvc[curr], sa_curr, self._ds)
        dt_next = self.__calc_dt(self._mvc[curr + 1], sa_next, self._ds)
        if (curr + 3) > len(self._mvc):
            return None
        # In case of sink->source
        if (self._mvc[curr] + sa_curr * dt_curr > self._mvc[curr + 1]) and \
           (self._mvc[curr + 1] + sa_next * dt_next < self._mvc[curr + 2]):
            # Find and return the exact tangent sp
            (sd1, sv1, sd2, sv2) = self.__search_precise_tangent_sp(curr)
            # self._sv[curr+1] = self._mvc[curr]+sa_limit*dt
            return (sd1, sv1 * self._LOW_SENSITIVITY_COEFF,
                    sd2, sv2 * self._LOW_SENSITIVITY_COEFF)
        return None

    def __search_precise_tangent_sp(self, curr):
        u"""Calculate the exact position of tangent sp between curr and curr+1."""
        (sd1, sd2) = (self._sd[curr], self._sd[curr + 1])
        (sv1, sv2) = (self._mvc[curr], self._mvc[curr + 1])
        for i in range(100):
            (sd1, sv1, sd2, sv2) = self.__iterate_tangent_sp(
                sd1, sv1, sd2, sv2)
            if (sd2 - sd1) < self._SP_ACCURACY:
                break
        return (sd1, sv1, sd2, sv2)

    def __iterate_tangent_sp(self, sd1, sv1, sd2, sv2):
        u"""Search for tangent switching point between (sd1, sv1) and (sd2, sv1) using bisection method.

        Args:
            sd1, sv1 (float): Left point
            sd2, sv2 (float): Right point
        Retrun:
            tuple: Two points surrounding SP (sd1, sv1, sd2, sv2)
        """
        sd = (sd1 + sd2) / 2.0
        # Calculate MVC
        self._kinematics.update(sd)
        self._dynamics.update()
        sv, _ = self.__get_mvc_vlc(sd)
        (sa_min, sa_max) = self._dynamics.calc_accel_limit(sv)
        # Calculate integration time
        dt = self.__calc_dt(sv, sa_min, sd2 - sd)
        if sv + sa_max * dt < sv2:
            (sd2, sv2) = (sd, sv)
        else:
            (sd1, sv1) = (sd, sv)
        return (sd1, sv1, sd2, sv2)

    def __check_trap_point(self, curr):
        u"""Check if there is a trap point between curr and curr+1.

        Args:
            curr (int): Index of the position to check
        Retrun:
            tuple: Two points surrounding SP (sd1, sv1, sd2, sv2)
            None: No SP found
        """
        # Determine if it is a point that can merge with VLC at the lower limit of acceleration
        self._kinematics.update(self._sd[curr])
        self._dynamics.update()
        if (self._vlc[curr] > self._mvc[curr]
                or self._vlc[curr + 1] > self._mvc[curr + 1]):
            return None
        [sa_min, sa_max] = self.__calc_accel_limit(self._vlc[curr])
        dt = self.__calc_dt(self._vlc[curr], sa_min, self._ds)
        if self._vlc[curr] + sa_min * dt < self._vlc[curr + 1]:
            self._sv[curr] = self._vlc[curr]
            return (self._sd[curr], self._vlc[curr],
                    self._sd[curr + 1], self._vlc[curr + 1])
        return None

    def __check_zero_inertia_point(self, curr):
        u"""Check if there is a zero inertia switching point between curr and curr+1.

        Specifically, check the sign reversal of a(s).
        If so, return the list of relevant constraints.

        Args:
            curr (int): Index of the position to check
        Retrun:
            list: List of constraints ('name', 'type') where the sign of a reverses
        """
        zero_list = []
        # Ignore start and end points
        if curr == 0 or curr == self._size - 1:
            return zero_list
        # Check a==0 for all constraints
        for pair in self._dynamics.limits:
            # Confirm if there is a point where a==0 in [curr, curr+1)
            if (self._a_buff[pair][curr] == 0
               or self._a_buff[pair][curr] * self._a_buff[pair][curr + 1] <= 0
               or self._a_buff[pair][curr + 1] == 0):
                zero_list.append(pair)
        return zero_list

    def __calc_zero_inertia_point(self, name, curr):
        u"""Calculate the exact position of zero-inertia point in [curr, curr+1).

        Args:
            name (tuple): Constraint ('name', 'type') where the sign of a reverses
            curr (int): Index of the position to check
        Retrun:
            tuple: Two points surrounding SP (sd1, sv1, sd2, sv2)
            None: No SP found
        """
        (sd1, sd2) = (self._sd[curr], self._sd[curr + 1])
        (a1, a2) = (self._a_buff[name][curr], self._a_buff[name][curr + 1])
        # Determine the exact position using bisection method
        if a1 != 0:
            for i in range(100):
                result = self.__iterate_zero_inertia_sp(name, sd1, a1, sd2, a2)
                (sd1, a1, sd2, a2) = result
                if (sd2 - sd1) < self._SP_ACCURACY:
                    break
        # Calculate Zero inertia point (sd1, sv1, sd2, sv2)
        self._kinematics.update(sd1)
        self._dynamics.update()
        sv1 = self._dynamics.calc_zero_inertia_sv(name)
        mvc1, vlc1 = self.__get_mvc_vlc(sd1)
        self._kinematics.update(sd2)
        self._dynamics.update()
        sv2 = self._dynamics.calc_zero_inertia_sv(name)
        mvc2, vlc2 = self.__get_mvc_vlc(sd2)
        # Ignore if above MVC
        if sv1 > mvc1 and sv2 > mvc2:
            return None
        # Ignore if above VLC
        if sv1 > vlc1 and sv2 > vlc2:
            return None
        return (sd1, sv1, sd2, sv2)

    def __iterate_zero_inertia_sp(self, name, sd1, param_a1, sd2, param_a2):
        u"""Zero-inertia switching point (a=0) between point 1 and point 2.

        Improve accuracy using bisection method
        Args:
            name (tuple): Constraint ('name', 'type') to be considered
            sd1 (float): s of the left point
            param_a1 (float): a(s) of the left point
            sd2 (float): s of the right point
            param_a2 (float): a(s) of the right point
        Retrun:
            tuple: Improved two points and parameter a (sd1, param_a1, sd2, param_a2)
        """
        sd = (sd1 + sd2) / 2.0
        if sd in self._kinematics.traj_memo:
            param_a = self._kinematics.traj_memo[sd][name[0]][1]
        else:
            self._kinematics.update(sd)
            self._dynamics.update()
            param_a = self._dynamics.a[name]
        # Search for point where param_a==0
        if param_a * param_a1 < 0:
            (sd2, param_a2) = (sd, param_a)
        else:
            (sd1, param_a1) = (sd, param_a)
        return (sd1, param_a1, sd2, param_a2)

    def __integrate_from_switching_point(self, curr, sp):
        u"""Check if integration can continue from the Switching point forward to curr+1 and backward to curr.

        Args:
            curr (int): Index of the position to check
            sp (tuple): Two points surrounding switching point (sd1, sv1, sd2, sv2)
        Retrun:
            bool: True: Integration successful False: Integration failed
        """
        (sd1, sv1, sd2, sv2) = sp
        # Confirm if forward integration connects to curr+1
        step = self._sd[curr + 1] - sd2
        if step == 0:
            (self._sd[curr + 1], self._sv[curr + 1]) = (sd2, sv2)
            self._fwd_singular = False
        else:
            result = self.__integrate_forward_adaptive(sd2, sv2, step)
            self._fwd_singular = False
            (r, sd, sv, sa) = result
            if r in ['MVC', 'VLC']:
                return False
            (self._sd[curr + 1], self._sv[curr + 1]) = (sd, sv)
        # Confirm if backward integration connects to curr
        step = sd1 - self._sd[curr]
        if step == 0:
            (self._sd[curr], self._sv[curr]) = (sd1, sv1)
            self._bkw_singular = False
        else:
            result = self.__integrate_backward_adaptive(sd1, sv1, step)
            self._bkw_singular = False
            (r, sd, sv, sa) = result
            if r in ['MVC', 'VLC']:
                return False
            (self._sd[curr], self._sv[curr]) = (sd, sv)
        # Confirm if backward integration connects to existing trajectory
        result = self.__integrate_backward_segment(curr)
        (r, index) = result
        if r != 'END':
            return False
        return True

    def __calc_accel_limit(self, sv):
        u"""Acceleration constraints of kinematics and torque constraints of dynamics.

        Find acceleration constraints of s considering both
        Args:
            sv(float):
                Velocity of s
        Return:
            [sa_min, sa_max](list):
                Lower and upper limits of acceleration of s
        """
        [sa_min_kin, sa_max_kin] = self._kinematics.calc_accel_limit(sv)
        [sa_min_dyn, sa_max_dyn] = self._dynamics.calc_accel_limit(sv)
        [sa_min, sa_max] = [
            max(sa_min_kin, sa_min_dyn), min(sa_max_kin, sa_max_dyn)]
        return [sa_min, sa_max]

    def validate(self):
        u"""Check if the calculated sv and time are valid results.

        Retrun:
            bool: True: Valid, False: Invalid result
        """
        for sv in self._sv:
            if (sv < 0) or isinf(sv):
                return False
        if self._time[-1] <= 0:
            return False
        return True

    def update_limited_value(self):
        u"""Reverse calculate actual results for constraints from (a, b, c, d) and save to self._value."""
        self._value = {}

        # Constraints of Dynamics
        for pair in self._dynamics.limits.keys():
            self._value[pair] = [0] * self._size
            for i in range(self._size):
                (sv, sa) = (self._sv[i], self._sa[i])
                self._value[pair][i] = (self._a_buff[pair][i] * sa + self._b_buff[pair][i] * sv ** 2
                                        + self._c_buff[pair][i] * sv + self._d_buff[pair][i])

        # Constraints of velocity
        vel_limits = [
            pair for pair in self._kinematics.limits if pair[1] == 'velocity']
        for pair in vel_limits:
            self._value[pair] = [0] * self._size
        for i in range(self._size):
            (sd, sv, sa) = (self._sd[i], self._sv[i], self._sa[i])
            self._kinematics.update(sd)
            point = self._kinematics.get_current_point()
            for name, limit_type in vel_limits:
                self._value[name, limit_type][i] = sv * point[name][1]

        # Constraints of acceleration
        acc_limits = [pair for pair in
                      self._kinematics.limits if pair[1] == 'acceleration']
        for pair in acc_limits:
            self._value[pair] = [0] * self._size
        for i in range(self._size):
            (sd, sv, sa) = (self._sd[i], self._sv[i], self._sa[i])
            self._kinematics.update(sd)
            point = self._kinematics.get_current_point()
            for name, limit_type in acc_limits:
                self._value[name, limit_type][i] = point[name][1] * sa + point[name][2] * sv ** 2

    def __get_mvc_vlc(self, sd, update_flg=False):
        u"""Determine if pre-calculated mvc and vlc can be used

        It's fine if sd is a multiple of ds, so judge by whether mod is 0
        Trick to absorb float error
        """
        mod_sd = int(sd * 1000000 + 0.1)
        mod_ds = int(self._ds * 1000000 + 0.1)
        if ((mod_sd) % (mod_ds)) == 0:
            # Can use pre-calculated results, so call from array
            already_index = int(mod_sd / mod_ds)
            mvc = self._mvc[already_index]
            vlc = self._vlc[already_index]
        else:
            # Recalculate as it's a number not calculated due to bisection method
            if update_flg:
                self._kinematics.update(sd)
                self._dynamics.update()
            mvc = self._dynamics.get_mvc()
            vlc = self._kinematics.get_vlc()
        # Return mvc and vlc
        return mvc, vlc

    def plot_limited_value(self):
        u"""Reverse calculate actual results for constraints from (a, b, c, d) and plot everything."""
        self.update_limited_value()
        for pair in self._dynamics.limits.keys():
            (name, limit_type) = pair
            plt.clf()
            plt.title(name)
            plt.plot(self._time, self._value[pair], '-o', label='%s,%s' % pair)
            plt.plot(
                self._time,
                [self._dynamics.limits[pair][0]] * self._size, label='min')
            plt.plot(
                self._time,
                [self._dynamics.limits[pair][1]] * self._size, label='max')
            plt.legend(loc='best')
            plt.show()

        for pair in self._kinematics.limits:
            (name, limit_type) = pair
            plt.clf()
            plt.title(name)
            plt.plot(self._time, self._value[pair], '-o', label='%s,%s' % pair)
            plt.plot(
                self._time,
                [self._kinematics.limits[pair][0]] * self._size, label='min')
            plt.plot(
                self._time,
                [self._kinematics.limits[pair][1]] * self._size, label='max')
            plt.legend()
            plt.show()

    def plot_preprocess_result(self):
        u"""Plot MVC and VLC."""
        plt.title('Preprocess result')
        plt.xlabel("s")
        plt.ylabel("sdot")
        plt.plot(self._sd, self._mvc, '-o', label="mvc")
        plt.plot(self._sd, self._vlc, '-o', label="vlc")
        plt.legend()
        plt.show()

    def plot_precise_curve(self, step=0.001):
        u"""Plot MVC and VLC curves specifying step width."""
        s = 0.0
        sd = []
        mvc = []
        vlc = []
        while s <= self._traj.length:
            self._kinematics.update(s)
            self._dynamics.update()
            sd.append(s)
            mvc.append(self._dynamics.get_mvc())
            vlc.append(self._kinematics.get_vlc())
            s += step
        plt.title('MVC and VLC result')
        plt.xlabel("s")
        plt.ylabel("sdot")
        plt.plot(sd, mvc, '-', label="mvc")
        plt.plot(sd, vlc, '-', label="vlc")
        plt.plot(self._sd, self._mvc, 'o', label="mvc")
        plt.plot(self._sd, self._vlc, 'o', label="vlc")
        plt.legend()
        plt.show()

    def plot_s_sdot_result(self):
        u"""Plot the final s-sdot trajectory."""
        plt.clf()
        plt.xlabel("$s$", fontsize=20)
        plt.ylabel("$dot{s}$", fontsize=20)
        plt.ylim(0, max(self._sv) * 2)
        plt.plot(self._sd, self._vlc, '-.', label="VLC", color='black', lw=2)
        plt.plot(self._sd, self._mvc, '--', label="MVC", color='black', lw=2)
        plt.plot(self._sd, self._sv, '-', label="Result", color='black', lw=2)
        plt.legend()
        plt.pause(1.0)
        # plt.show()

    def output_limited_value(self, cond_name, offset=1.0):
        self.update_limited_value()
        dir_name = "/home/tajima/tmp/%s" % cond_name
        if not os.path.exists(dir_name):
            os.makedirs(dir_name)
        for pair in self._dynamics.limits.keys():
            data_file = open(dir_name + '/%s_%s.txt' % pair, 'w')
            for i in range(self._size):
                print(self._time[i], self._value[pair][i], file=data_file)

        for pair in self._kinematics.limits:
            data_file = open(dir_name + '/%s_%s.txt' % pair, 'w')
            for i in range(self._size):
                print(self._time[i], self._value[pair][i], file=data_file)
