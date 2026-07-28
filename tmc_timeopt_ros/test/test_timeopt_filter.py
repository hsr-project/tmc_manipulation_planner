#!/usr/bin/env python
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
import unittest

from rclpy.duration import Duration
from sensor_msgs.msg import JointState
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt_ros.timeopt_filter import (
    _ros_trajectory_from_timeopt,
    _timeopt_trajectory_from_ros,
)
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)


class ActiveCasterTestCase(unittest.TestCase):
    def test_ros_traj_from_timeopt_normal(self):
        u"""Test if basic conversion is performed"""
        timeopt_traj = [(0.0, {'joint1': (0, 0, 0), 'joint2': (0, 0, 0)}),
                        (1.0, {'joint1': (1, 1, 1), 'joint2': (2, 2, 2)}),
                        (2.0, {'joint1': (2, 2, 2), 'joint2': (3, 3, 3)})]

        ros_traj = _ros_trajectory_from_timeopt(
            timeopt_traj, ['joint1', 'joint2'], None, {'joint1': None, 'joint2': None})
        self.assertEqual(ros_traj.joint_names, ['joint1', 'joint2'])
        self.assertEqual(len(ros_traj.points), 3)
        for i in range(3):
            point = ros_traj.points[i]
            self.assertAlmostEqual(Duration(seconds=timeopt_traj[i][0]), Duration.from_msg(point.time_from_start))
            self.assertAlmostEqual(point.positions[0], timeopt_traj[i][1]['joint1'][0])
            self.assertAlmostEqual(point.velocities[0], timeopt_traj[i][1]['joint1'][1])
            self.assertAlmostEqual(point.accelerations[0], timeopt_traj[i][1]['joint1'][2])

            self.assertAlmostEqual(point.positions[1], timeopt_traj[i][1]['joint2'][0])
            self.assertAlmostEqual(point.velocities[1], timeopt_traj[i][1]['joint2'][1])
            self.assertAlmostEqual(point.accelerations[1], timeopt_traj[i][1]['joint2'][2])

    def test_ros_traj_from_timeopt_skip(self):
        u"""Test skipping trajectory with min_step"""
        timeopt_traj = [(0.0, {'joint1': (0, 0, 0), 'joint2': (0, 0, 0)}),
                        (1.0, {'joint1': (1, 1, 1), 'joint2': (2, 2, 2)}),
                        (1.005, {'joint1': (2, 2, 2), 'joint2': (3, 3, 3)})]

        ros_traj = _ros_trajectory_from_timeopt(
            timeopt_traj, ['joint1', 'joint2'], None, {'joint1': None, 'joint2': None})
        self.assertEqual(len(ros_traj.points), 2)

    def test_ros_traj_from_timeopt_offset(self):
        u"""Test offset"""
        timeopt_traj = [(0.0, {'joint1': (0, 0, 0), 'joint2': (0, 0, 0)}),
                        (1.0, {'joint1': (1, 1, 1), 'joint2': (2, 2, 2)}),
                        (2.0, {'joint1': (2, 2, 2), 'joint2': (3, 3, 3)})]

        ros_traj = _ros_trajectory_from_timeopt(
            timeopt_traj, ['joint1', 'joint2'], None, {'joint1': None, 'joint2': None}, offset=0.5)
        for i in range(3):
            point = ros_traj.points[i]
            self.assertAlmostEqual(Duration.from_msg(point.time_from_start),
                                   Duration(seconds=timeopt_traj[i][0] + 0.5))

    def test_timeopt_traj_from_ros_normal(self):
        u"""Simple conversion test"""
        start_state = JointState()
        start_state.name = ['joint1', 'joint2']
        start_state.position = [0.0, 0.0]
        trajectory_msgs = JointTrajectory()
        trajectory_msgs.joint_names = ['joint1', 'joint2']
        trajectory_msgs.points = [JointTrajectoryPoint(positions=point) for point in [[1, 3], [2, 4]]]

        timeopt_traj = _timeopt_trajectory_from_ros(
            start_state, trajectory_msgs,
            ['joint1', 'joint2', 'joint3'])
        self.assertEqual(timeopt_traj.length, 3)
        self.assertAlmostEqual(timeopt_traj['joint1'][0][0], 0.0)
        self.assertAlmostEqual(timeopt_traj['joint1'][1][0], 1.0)
        self.assertAlmostEqual(timeopt_traj['joint1'][2][0], 2.0)
        self.assertAlmostEqual(timeopt_traj['joint2'][0][0], 0.0)
        self.assertAlmostEqual(timeopt_traj['joint2'][1][0], 3.0)
        self.assertAlmostEqual(timeopt_traj['joint2'][2][0], 4.0)
        self.assertAlmostEqual(timeopt_traj['joint3'][0][0], 0.0)
        self.assertIsInstance(timeopt_traj['joint1'], NaturalCubicSplineTrajectory)
        self.assertIsInstance(timeopt_traj['joint2'], NaturalCubicSplineTrajectory)
        self.assertIsInstance(timeopt_traj['joint3'], NaturalCubicSplineTrajectory)

    def test_timeopt_traj_from_ros_decimate(self):
        u"""Test thinning"""
        start_state = JointState()
        start_state.name = ['joint1']
        start_state.position = [0.0]
        trajectory_msgs = JointTrajectory()
        trajectory_msgs.joint_names = ['joint1']
        trajectory_msgs.points = [JointTrajectoryPoint(positions=point) for point in [[0.00001], [2]]]

        timeopt_traj = _timeopt_trajectory_from_ros(
            start_state, trajectory_msgs, ['joint1'])
        self.assertEqual(timeopt_traj.length, 2)

    def test_timeopt_traj_from_ros_decimate_no_point(self):
        u"""Thinning results in no points"""
        start_state = JointState()
        start_state.name = ['joint1']
        start_state.position = [0.0]
        trajectory_msgs = JointTrajectory()
        trajectory_msgs.joint_names = ['joint1']
        trajectory_msgs.points = [JointTrajectoryPoint(positions=point) for point in [[0.00001]]]

        timeopt_traj = _timeopt_trajectory_from_ros(
            start_state, trajectory_msgs, ['joint1'])
        self.assertIsNone(timeopt_traj)
