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
u"""Module that calculates the shortest time control problem (TOPP)."""

from math import isinf
from math import sqrt
import os

import matplotlib
# Change AGG to TKAGG when Debug
matplotlib.use('Agg')

# Subsequent IMPORT is written after matplotlib.use, so pass through with noqa.
import matplotlib.pyplot as plt  # noqa

from tmc_timeopt.dynamics import Dynamics  # noqa
from tmc_timeopt.kinematics import Kinematics  # noqa
from tmc_timeopt.trajectory import TrajectoryDict, _NEW_SCIPY  # noqa


def _plot_step(func):
    u"""A decorator that plots the integration of one step.

    For debugging.
    """
    def wrapper(*args, **kwds):
        self = args[0]
        (sd0, sv0, ds) = args[1:4]
        result = func(*args, **kwds)
        print("%s%s: %s" % (func.__name__, args[1:], result))
        (r, sd1, sv1, sa0) = result
        plt.clf()
        # Plot the results of pretreatment
        plt.plot(self._sd, self._vlc, '-', label="vlc")
        plt.plot(self._sd, self._mvc, '-', label="mvc")
        plt.quiver(self._sd, self._mvc,
                   [self._ds] * self._size, self._mvc_sa_min,
                   scale_units='xy', angles='xy', scale=1, width=0.006)
        plt.quiver(self._sd, self._mvc,
                   [self._ds] * self._size, self._mvc_sa_max,
                   scale_units='xy', angles='xy', scale=1, width=0.006)
        # Plot the result
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
    u"""Class to calculate the shortest time control problem (TOPP)."""

    # Maximum number of division integration
    _INTEGRATE_DIV_NUM = 4
    # TANGENT POINT has a severe calculation, so it reduces sensitivity
    _LOW_SENSITIVITY_COEFF = 0.95
    _VLC_MERGIN = 0.001
    _MVC_MERGIN = 0.001
    _CROSS_MERGIN = 0.0

    # Minimum time integral range
    _MINIMUM_DT = 0.001

    # Switching point accuracy
    _SP_ACCURACY = 1e-4

    # Double solution threshold
    _DOUBLE_EPS = 1e-3

    def __init__(self, target):
        u"""Initialize the class by giving Target.

        Args:
            target: Target
        """
        self._target = target
        self._kinematics = Kinematics(self._target)
        self._dynamics = Dynamics(self._target)
        # Arrangement size of the entire track
        self._size = 0
        # A, B, C, D parameter buffer
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
        u"""Set the spatial orbital.

        Args:
            trajectory (TrajectoryDict): Space orbit
        """
        if not isinstance(trajectory, TrajectoryDict):
            raise TypeError('TrajectoryDict is needed.')
        self._traj = trajectory
        self._kinematics.set_trajectory(trajectory)

    def preprocess(self, step=0.1):
        u"""Perform preprocessing for the shortest time control.

           1. Ensuring the area
           2. Calculation of MVC and VLC
           3. Calculate Dynamics parameters
        Args:
            step (float): Chopped width of S used for search
        Note:
            MVC is more efficient and simple to calculate all discrete points first
        """
        self._ds = step
        # Get the length of the orbit
        self._size = int((self._traj.length - 1) / float(step)) + 1
        # Calf._sd
        self._sd = [float(i) * self._ds for i in range(self._size)]
        # Calculate Trajj
        if _NEW_SCIPY:
            # It is faster to calculate in advance when not using Scipy
            self._kinematics.pre_calc_traj(self._sd, step)
        # Ensuring the area for processing
        (self._mvc, self._sa_mvc, self._vlc) = ([0] * self._size, [0] * self._size, [0] * self._size)
        self._mvc_dt = [0] * self._size
        self._mvc_sa_min = [0] * self._size
        self._mvc_sa_max = [0] * self._size
        # List of actual solutions
        self._sv = [float('inf')] * self._size
        self._sv[0] = self._sv[-1] = 0
        self._sa = [0] * self._size
        self._fw = [True] * self._size

        # Ensuring the area for parameters
        for pair in self._dynamics.limits:
            self._a_buff[pair] = [0] * self._size
            self._b_buff[pair] = [0] * self._size
            self._c_buff[pair] = [0] * self._size
            self._d_buff[pair] = [0] * self._size

        # Finding MVC and VLC for all sections
        for i, sd in enumerate(self._sd):
            # Update Kinematics, Dynamics
            self._kinematics.update(sd)
            self._dynamics.update()

            # Calculate MVC
            self._mvc[i] = self._dynamics.get_mvc()
            # Find the upper and lower limit of acceleration on MVC
            (self._mvc_sa_min[i], self._mvc_sa_max[i]) = self._dynamics.calc_accel_limit(self._mvc[i])
            self._sa_mvc[i] = self._mvc_sa_min[i]
            dt = self.__calc_dt(self._mvc[i], self._mvc_sa_min[i], self._ds)
            self._mvc_sa_min[i] *= dt
            dt = self.__calc_dt(self._mvc[i], self._mvc_sa_max[i], self._ds)
            self._mvc_sa_max[i] *= dt

            # Calculate VLC
            self._vlc[i] = self._kinematics.get_vlc()
            # Save restriction parameters (a, b, c, d)
            for pair in self._dynamics.limits:
                self._a_buff[pair][i] = self._dynamics.a[pair]
                self._b_buff[pair][i] = self._dynamics.b[pair]
                self._c_buff[pair][i] = self._dynamics.c[pair]
                self._d_buff[pair][i] = self._dynamics.d[pair]

    def update(self):
        u"""Solve the TOPP algorithm to generate the shortest speed orbit."""
        # Intide from the end point to the backwards
        self.__integrate_backward_segment(self._size - 1)

        # Integrate positively from the starting point
        curr = 0
        while curr < self._size:
            # Inner progression until it is stopped
            result = self.__integrate_forward_segment(curr)
            (r, index) = result
            if r == 'END':
                break
            # Find a switching point forward
            result = self.__search_switching_point(index)
            (r, curr) = result
            # If there is no valid switching point
            if r == 'NG':
                raise RuntimeError('No valid switching point found.')
        # Restorate the orbit and finish
        self.__recalc_trajectory()

    def get_optimal_trajectory(self):
        u"""uGet the optimal orbital after calculating with Update.

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
        u"""From the SD [CURR], integrate as positively as possible and return the stop factor and stop position.

        Args:
            curr (int): Index of the starting position of the integral
        Retrun:
            Tuple: Returns tuple of (stop factor, stop index).
            'End': The integral has ended to the end
            'MVC': Stop beyond MVC
            'VLC': Stop without being attached to VLC
        """
        while curr < self._size - 1:
            (sd0, sv0) = (self._sd[curr], self._sv[curr])
            # Integration while changing the integration
            result = self.__integrate_forward_adaptive(sd0, sv0, self._ds)
            (r, sd1, sv1, sa0) = result
            # The integral has stopped
            if r != 'OK':
                return (r, curr)
            # Insolation judgment with retreating orbital
            if curr + 1 < self._size:
                if sv1 >= self._sv[curr + 1] - self._CROSS_MERGIN:
                    # It seems that it is a bug that does not update SA here with Chuken's code
                    self._sa[curr] = sa0
                    break
            self._sa[curr] = sa0
            curr += 1
            self._sv[curr] = sv1
        return ('END', curr)

    def __integrate_forward_adaptive(self, sd0, sv0, step):
        u"""(SD0, SV0) integrate only the STEP width positively.

        If the integral fails, try it while dividing the integration width.
        Args:
            sd0 (FLOAT): Points start point S
            sv0 (Float): Speed ​​S at the integration starting point
        Return:
            Tuple: Returns tuple (stopped factor, stopped state).
            'OK': Successful points
            'MVC': Stop beyond MVC
            'VLC': Stop without being attached to VLC
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
        u"""From (sd0, sv0), integrate the step with Div_num split.

        Args:
            sd0 (FLOAT): Points start point S
            sv0 (Float): Speed ​​S at the integration starting point
        Return:
            Tuple: Returns tuple (stopped factor, stopped state).
            'OK': Successful points
            'MVC': Stop beyond MVC
            'VLC': Stop without being attached to VLC
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
        u"""(SD, SV) to STEP width 1 step, integrate positively.

        Args:
            sd (float): S at the integral starting point
            sv (float): speed of S at the integration starting point
            step (float):
            singular_flag (bool): Turue when integration from Zero-Inertia Switching Point
        Return:
            Tuple: Returns tuple (stopped factor, stopped state).
            'OK': Successful points
            'MVC': Failed beyond MVC
            'VLC': Failed because it does not accompany VLC
        """
        # Update dynamics
        self._kinematics.update(sd)
        self._dynamics.update()
        # Calculate the current acceleration upper and lower limit
        if singular_flag:
            [self._sa_min, self._sa_max] = [0, 0]
        else:
            [self._sa_min, self._sa_max] = self.__calc_accel_limit(sv)
        # Calculate the integration time
        self._dt_min = self.__calc_dt(sv, self._sa_min, step)
        self._dt_max = self.__calc_dt(sv, self._sa_max, step)
        # Update the state
        sd_next = sd + sv * self._dt_max + 0.5 * self._sa_max * self._dt_max ** 2
        sv_next = sv + self._sa_max * self._dt_max
        # When the constraints are too tight and cannot be realized
        if sv_next < 0:
            return ('MVC', sd_next, sv_next, self._sa_max)
        # Update dynamics at the candidate
        mvc, vlc = self.__get_mvc_vlc(sd_next, update_flg=True)
        # Judge whether it has exceeded MVC
        if sv_next > mvc + self._MVC_MERGIN:
            return ('MVC', sd_next, sv_next, self._sa_max)
        # Shaping so that it does not exceed VLC
        if sv_next > vlc:
            dt = 2.0 * step / (vlc + sv)
            sa = (vlc - sv) / dt
            sv_next = vlc

            # If you can't follow VLC
            if sa < self._sa_min:
                return ('VLC', sd_next, sv_next, self._sa_min)
            else:
                self._sa_max = sa
        return ('OK', sd_next, sv_next, self._sa_max)

    def __integrate_backward_segment(self, curr):
        u"""From SD [CURR], integrate as much as possible and return the stop factor and stop position.

        Args:
            curr (int): Index of the starting position of the integral
        Retrun:
            Tuple: Returns tuple of (stop factor, stop index).
            'End': The integral has ended to the end
            'MVC': Stop beyond MVC
            'VLC': Stop without being attached to VLC
        """
        while curr > 0:
            (sd1, sv1) = (self._sd[curr], self._sv[curr])
            result = self.__integrate_backward_adaptive(sd1, sv1, self._ds)
            (r, sd0, sv0, sa1) = result
            # Called
            if r != 'OK':
                return(r, curr)
            # Insolation judgment with the advanced orbit
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
        u"""(SD1, SV1) is integrated only in the STEP width.

        If the integral fails, try it while dividing the integration width.
        Args:
            sd1 (float): S at the integration start point
            sv1 (float): speed of S at the integration starting point
        Return:
            Tuple: Returns tuple (stopped factor, stopped state).
            'OK': Successful points
            'MVC': Stop beyond MVC
            'VLC': Stop without being attached to VLC
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
        u"""(SD1, SV1) is integrated with Div_num divided in the width of Div_num.

        Args:
            sd1 (float): S at the integration start point
            sv1 (float): speed of S at the integration starting point
        Return:
            Tuple: Returns tuple (stopped factor, stopped state).
            'OK': Successful points
            'MVC': Stop beyond MVC
            'VLC': Stop without being attached to VLC
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
        u"""From (SD, SV) to STEP width 1 step, integrate backwards.

        Args:
            sd (float): S at the integral starting point
            sv (float): speed of S at the integration starting point
            step (float):
            singular_flag (bool): Turue when integration from Zero-Inertia Switching Point
        Return:
            Tuple: Returns tuple (stopped factor, stopped state).
            'OK': Successful points
            'MVC': Failed beyond MVC
            'VLC': Failed because it does not accompany VLC
        """
        # Update dynamics
        self._kinematics.update(sd)
        self._dynamics.update()
        # Calculate the current acceleration upper and lower limit
        if singular_flag:
            [self._sa_min, self._sa_max] = [0, 0]
        else:
            [self._sa_min, self._sa_max] = self.__calc_accel_limit(sv)
        # Rear points
        self._dt_min = self.__calc_dt_back(sv, self._sa_min, step)
        self._dt_max = self.__calc_dt_back(sv, self._sa_max, step)
        # Update the state
        sd_prev = sd - sv * self._dt_min + 0.5 * self._sa_min * self._dt_min ** 2
        sv_prev = sv - self._sa_min * self._dt_min
        # When the constraints are too tight and cannot be realized
        if sv_prev < 0:
            return ('MVC', sd_prev, sv_prev, self._sa_min)
        # Update dynamics at the candidate
        mvc, vlc = self.__get_mvc_vlc(sd_prev, update_flg=True)
        # Judge whether it has exceeded MVC
        if sv_prev > mvc + self._MVC_MERGIN:
            return ('MVC', sd_prev, sv_prev, self._sa_min)
        # Shaping so that it does not exceed VLC
        if sv_prev > vlc:
            dt = 2.0 * step / (vlc + sv)
            sa = (sv - vlc) / dt
            sv_prev = vlc

            # If you can't follow VLC
            if sa > self._sa_max:
                return ('VLC', sd_prev, sv_prev, self._sa_max)
            else:
                self._sa_min = sa
        return ('OK', sd_prev, sv_prev, self._sa_min)

    def __calc_dt(self, sv, sa, ds):
        u"""Calculate the integral time DT of the width DS (> 0) when integrating from SV and SA.

        Args:
            sv (float): Sp speed
            sa (float): acceleration of S
            ds (float): S integral value
        Retrun:
            Float: Points DT DT
        """
        minimum_dt = self._MINIMUM_DT * ds / self._ds
        disciminant = sv ** 2.0 + 2.0 * sa * ds
        if sa == float('inf') or sa == float('-inf'):
            return minimum_dt
        if abs(sa) < self._DOUBLE_EPS:
            if sv == 0:
                return minimum_dt
            return ds / sv
        # If there is no solution
        if (disciminant) < 0:
            if sv > self._DOUBLE_EPS:
                # If it is small, it will be returned in a seriously solution as an error in the numerical calculation.
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
        u"""Calculate the integral time DT of the width DS (> 0) when integrating from SV and SA.

           Resignation
        Args:
            sv (float): Sp speed
            sa (float): acceleration of S
            ds (float): S integral value
        Retrun:
            Float: Points DT DT
        """
        minimum_dt = self._MINIMUM_DT * ds / self._ds
        disciminant = sv ** 2.0 - 2.0 * sa * ds
        if sa == float('inf') or sa == float('-inf'):
            return minimum_dt
        if abs(sa) < self._DOUBLE_EPS:
            if sv == 0:
                return minimum_dt
            return ds / sv
        # If there is no solution
        if (disciminant) < 0:
            if sv > self._DOUBLE_EPS:
                # If it is small, it will be returned in a seriously solution as an error in the numerical calculation.
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
        u"""We integrate from s = 0 to the end point and update the final state."""
        (self._time, self._span) = ([0] * self._size, [0] * self._size)
        tick = 0
        for n in range(self._size):
            (sv, sa) = (self._sv[n], self._sa[n])
            self._time[n] = tick
            span = self.__calc_dt(sv, sa, self._ds)
            self._span[n] = span
            tick += span

    def __search_switching_point(self, curr):
        u"""Search positively from the location CURR until Switching Point is found.

        Args:
            curr (int): Index at the start of search
        Retrun:
            Tuple: Returns the tuple in (stop factor, stopped position).
            'OK': I found a switching point
            'NG': I can't find a switching point (normally impossible)
        """
        while curr < self._size - 1:
            # Check for Zero-Inertia SP
            for name in self.__check_zero_inertia_point(curr):
                sp = self.__calc_zero_inertia_point(name, curr)
                if sp:
                    (self._fwd_singular, self._bkw_singular) = (True, True)
                    if self.__integrate_from_switching_point(curr, sp):
                        (self._fwd_singular, self._bkw_singular) = (False, False)
                        return ('OK', curr + 1)
                    (self._fwd_singular, self._bkw_singular) = (False, False)

            # Check of Trap Point
            sp = self.__check_trap_point(curr)
            if sp:
                # Check if you can continue integration from SP
                res = self.__integrate_from_switching_point(curr, sp)
                if res:
                    return ('OK', curr + 1)

            # Check for tangent point
            sp = self.__check_tangent_point(curr)
            if sp:
                # Check if you can continue integration from SP
                res = self.__integrate_from_switching_point(curr, sp)
                if res:
                    return ('OK', curr + 1)
            # Investigate the following points
            curr += 1
        return ('NG', curr)

    def __check_tangent_point(self, curr):
        u"""Check if there is a tangent Switching point between Curr and CURR+1.

        If so, return the accurate position in the 2nd method
        Args:
            curr (int): Checking the index
        Retrun:
            Tuple: (SD1, SV1, SD2, SV2) 2 points sandwiched
            None: There was no SP
        """
        # Calculate acceleration restrictions
        sa_curr = self._sa_mvc[curr]
        sa_next = self._sa_mvc[curr + 1]
        # Calculate the integration time
        dt_curr = self.__calc_dt(self._mvc[curr], sa_curr, self._ds)
        dt_next = self.__calc_dt(self._mvc[curr + 1], sa_next, self._ds)
        if (curr + 3) > len(self._mvc):
            return None
        # Sink-> Source
        if (self._mvc[curr] + sa_curr * dt_curr > self._mvc[curr + 1]) and \
           (self._mvc[curr + 1] + sa_next * dt_next < self._mvc[curr + 2]):
            # Return in search of accurate tangent SP
            (sd1, sv1, sd2, sv2) = self.__search_precise_tangent_sp(curr)
            # self._sv[curr+1] = self._mvc[curr]+sa_limit*dt
            return (sd1, sv1 * self._LOW_SENSITIVITY_COEFF,
                    sd2, sv2 * self._LOW_SENSITIVITY_COEFF)
        return None

    def __search_precise_tangent_sp(self, curr):
        u"""Calculate the exact location of Tangent SP between CURR and CURR+1."""
        (sd1, sd2) = (self._sd[curr], self._sd[curr + 1])
        (sv1, sv2) = (self._mvc[curr], self._mvc[curr + 1])
        for i in range(100):
            (sd1, sv1, sd2, sv2) = self.__iterate_tangent_sp(
                sd1, sv1, sd2, sv2)
            if (sd2 - sd1) < self._SP_ACCURACY:
                break
        return (sd1, sv1, sd2, sv2)

    def __iterate_tangent_sp(self, sd1, sv1, sd2, sv2):
        u"""Explore Tangent Switching Point between (SD1, SV1) and (SD2, SV1) in a two -part method.

        Args:
            sd1, sv1 (float): Points on the left
            sd2, sv2 (float): Points on the right
        Retrun:
            Tuple: (SD1, SV1, SD2, SV2) 2 points sandwiched
        """
        sd = (sd1 + sd2) / 2.0
        # Calculate MVC
        self._kinematics.update(sd)
        self._dynamics.update()
        sv, _ = self.__get_mvc_vlc(sd)
        (sa_min, sa_max) = self._dynamics.calc_accel_limit(sv)
        # Calculate the integration time
        dt = self.__calc_dt(sv, sa_min, sd2 - sd)
        if sv + sa_max * dt < sv2:
            (sd2, sv2) = (sd, sv)
        else:
            (sd1, sv1) = (sd, sv)
        return (sd1, sv1, sd2, sv2)

    def __check_trap_point(self, curr):
        u"""Check if there is a Trap Point between CURR and CURR+1.

        Args:
            curr (int): Checking the index
        Retrun:
            Tuple: (SD1, SV1, SD2, SV2) 2 points sandwiched
            None: There was no SP
        """
        # Judge whether VLC can join at the lower limit of acceleration
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
        u"""Check if there is a ZERO INERTIA Switching Point between CURR and CURR+1.

        Specifically, check the reversal of the sign of A (s).
        If so, return the list of relevant restraint conditions.

        Args:
            curr (int): Checking the index
        Retrun:
            List: List of restraint ('name', 'type') in which the sign of A reverses
        """
        zero_list = []
        # Ignore the starting point and the end point
        if curr == 0 or curr == self._size - 1:
            return zero_list
        # Check A == 0 for all restrictions
        for pair in self._dynamics.limits:
            # Check if there is a point of A == 0 of [Curr, Curr+1)
            if (self._a_buff[pair][curr] == 0
               or self._a_buff[pair][curr] * self._a_buff[pair][curr + 1] <= 0
               or self._a_buff[pair][curr + 1] == 0):
                zero_list.append(pair)
        return zero_list

    def __calc_zero_inertia_point(self, name, curr):
        u"""Calculate the accurate position of ZERO-Inertia Point of [Curr, Currr+1).

        Args:
            name (tuple): Contribution where A is inverted ('name', 'type')
            curr (int): Checking the index
        Retrun:
            Tuple: (SD1, SV1, SD2, SV2) 2 points sandwiched
            None: There was no SP
        """
        (sd1, sd2) = (self._sd[curr], self._sd[curr + 1])
        (a1, a2) = (self._a_buff[name][curr], self._a_buff[name][curr + 1])
        # Give accurate positions by 2 minutes
        if a1 != 0:
            for i in range(100):
                result = self.__iterate_zero_inertia_sp(name, sd1, a1, sd2, a2)
                (sd1, a1, sd2, a2) = result
                if (sd2 - sd1) < self._SP_ACCURACY:
                    break
        # Calculate ZERO INERTIA POINT (SD1, SV1, SD2, SV2)
        self._kinematics.update(sd1)
        self._dynamics.update()
        sv1 = self._dynamics.calc_zero_inertia_sv(name)
        mvc1, vlc1 = self.__get_mvc_vlc(sd1)
        self._kinematics.update(sd2)
        self._dynamics.update()
        sv2 = self._dynamics.calc_zero_inertia_sv(name)
        mvc2, vlc2 = self.__get_mvc_vlc(sd2)
        # Ignore it on MVC
        if sv1 > mvc1 and sv2 > mvc2:
            return None
        # Ignore it on VLC
        if sv1 > vlc1 and sv2 > vlc2:
            return None
        return (sd1, sv1, sd2, sv2)

    def __iterate_zero_inertia_sp(self, name, sd1, param_a1, sd2, param_a2):
        u"""Zero-Inertia Switching Point between point 1 and point 2 (point is a = 0).

        Improve accuracy in a 2 -minute method
        Args:
            name (tuple): Target constraints ('name', 'type')
            sd1 (float): S at the left point
            param_a1 (float): A (S) at the left point
            sd2 (float): S at the right point
            param_a2 (float): A (s) at the right point
        Retrun:
            Tuple: (SD1, Param_a1, SD2, Param_a2) 2 improved points and parameters A
        """
        sd = (sd1 + sd2) / 2.0
        if sd in self._kinematics.traj_memo:
            param_a = self._kinematics.traj_memo[sd][name[0]][1]
        else:
            self._kinematics.update(sd)
            self._dynamics.update()
            param_a = self._dynamics.a[name]
        # Find a point of param_a == 0
        if param_a * param_a1 < 0:
            (sd2, param_a2) = (sd, param_a)
        else:
            (sd1, param_a1) = (sd, param_a)
        return (sd1, param_a1, sd2, param_a2)

    def __integrate_from_switching_point(self, curr, sp):
        u"""Check if you can continue to integrate from Switching Point to CURR+1 and later CURR.

        Args:
            curr (int): Checking the index
            sp (tuple): 2 points that sandwiches Switching Point (SD1, SV1, SD2, SV2)
        Retrun:
            BOOL: TRUE: Successful integral FALSE: Points failed
        """
        (sd1, sv1, sd2, sv2) = sp
        # Confirm that the progress is connected to CURR+1
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
        # Confirm whether it will be connected to CURR by retreatment
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
        # Is it a retreat and connected to the existing orbit?
        result = self.__integrate_backward_segment(curr)
        (r, index) = result
        if r != 'END':
            return False
        return True

    def __calc_accel_limit(self, sv):
        u"""Kinematics acceleration constraints and dynamic torque restrictions.

        Find the constraints of S acceleration in consideration of both
        Args:
            sv(float):
                S speed
        Return:
            [sa_min, sa_max](list):
                Lower, upper limit of S acceleration
        """
        [sa_min_kin, sa_max_kin] = self._kinematics.calc_accel_limit(sv)
        [sa_min_dyn, sa_max_dyn] = self._dynamics.calc_accel_limit(sv)
        [sa_min, sa_max] = [
            max(sa_min_kin, sa_min_dyn), min(sa_max_kin, sa_max_dyn)]
        return [sa_min, sa_max]

    def validate(self):
        u"""Check if the calculated SV and time are valid.

        Retrun:
            Bool: True: Value, False: Funny results
        """
        for sv in self._sv:
            if (sv < 0) or isinf(sv):
                return False
        if self._time[-1] <= 0:
            return False
        return True

    def update_limited_value(self):
        u"""Calculate the actual result for constraints from (a, b, c, d) and save it in Self._value."""
        self._value = {}

        # Dynamics constraints
        for pair in self._dynamics.limits.keys():
            self._value[pair] = [0] * self._size
            for i in range(self._size):
                (sv, sa) = (self._sv[i], self._sa[i])
                self._value[pair][i] = (self._a_buff[pair][i] * sa + self._b_buff[pair][i] * sv ** 2
                                        + self._c_buff[pair][i] * sv + self._d_buff[pair][i])

        # Speed ​​constraints
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

        # Constation of acceleration
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
        u"""Judgment whether the calculated MVC and VLC can be used in the preserv

        If the SD is a multiple of the DS, it is determined by whether the mod is 0.
        Small work to absorb Float errors
        """
        mod_sd = int(sd * 1000000 + 0.1)
        mod_ds = int(self._ds * 1000000 + 0.1)
        if ((mod_sd) % (mod_ds)) == 0:
            # Call from array because you can use the calculated result
            already_index = int(mod_sd / mod_ds)
            mvc = self._mvc[already_index]
            vlc = self._vlc[already_index]
        else:
            # Since it is a number that is not calculated due to the influence of the 2 -minute method, it will be recalculated.
            if(update_flg):
                self._kinematics.update(sd)
                self._dynamics.update()
            mvc = self._dynamics.get_mvc()
            vlc = self._kinematics.get_vlc()
        # Return MVC and VLC
        return mvc, vlc

    def plot_limited_value(self):
        u"""Plot all of the actual results for constraints from (a, b, c, d) backwards."""
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
        u"""Specify the step width of the curve of MVC and VLC."""
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
        u"""Plot the final S-SDOT orbit."""
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
