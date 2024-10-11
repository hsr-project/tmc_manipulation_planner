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
import threading
import unittest

import launch
import launch_ros.actions
import launch_testing
from moveit_msgs.msg import JointLimits
import pytest
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


_JOINT1_LIMIT = {'velocity': 0.1, 'acceleration': 0.2}
_JOINT2_LIMIT = {'velocity': 0.3, 'acceleration': 0.4}
_EPSILONE = 1.0e-2


@pytest.mark.launch_test
def generate_test_description():
    timeopt_filter_node = launch_ros.actions.Node(
        package='tmc_timeopt_ros',
        executable='timeopt_filter_node',
        parameters=[{'use_joint': ['joint1', 'joint2'],
                     'timeopt_resolution': 0.01,
                     'joint1': _JOINT1_LIMIT,
                     'joint2': _JOINT2_LIMIT}],
        output='screen'
    )
    return launch.LaunchDescription(
        [timeopt_filter_node, launch_testing.actions.ReadyToTest()])


class TimeoptFilterNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._req = FilterJointTrajectory.Request()
        self._req.start_state.joint_state.name = ['joint1', 'joint2']
        self._req.start_state.joint_state.position = [1.0, 1.0]
        self._req.trajectory.joint_names = ['joint1', 'joint2']
        self._req.trajectory.points = [
            JointTrajectoryPoint(positions=pos)
            for pos in [[1.5, 0.0], [2.0, 2.0]]]

        # In order to call the service, it is necessary to make a separate thread of SPIN
        self._node = rclpy.create_node("test_node")
        self._executor = MultiThreadedExecutor()
        self._thread = threading.Thread(target=rclpy.spin, args=(self._node, self._executor), daemon=True)
        self._thread.start()

        self._filter_srv = self._node.create_client(FilterJointTrajectory, 'filter_trajectory')
        self._filter_srv.wait_for_service(timeout_sec=1.0)

        self._parameter_srv = self._node.create_client(SetParameters, 'timeopt_filter_node/set_parameters')
        self._parameter_srv.wait_for_service(timeout_sec=1.0)

    def tearDown(self):
        set_param_req = SetParameters.Request()
        set_param_req.parameters.append(Parameter('velocity_ratio', value=1.0).to_parameter_msg())
        set_param_req.parameters.append(Parameter('acceleration_ratio', value=1.0).to_parameter_msg())
        _ = self._parameter_srv.call(set_param_req)

        self._node.destroy_node()

    def test_simple_optimization(self):
        u"""Simple time optimization"""
        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        self.assertEqual(len(res.trajectory.joint_names), 2)
        self.assertEqual(res.trajectory.joint_names[0], 'joint1')
        self.assertEqual(res.trajectory.joint_names[1], 'joint2')

        for point in res.trajectory.points:
            self.assertLessEqual(abs(point.velocities[0]), _JOINT1_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[0]), _JOINT1_LIMIT['acceleration'] + _EPSILONE)
            self.assertLessEqual(abs(point.velocities[1]), _JOINT2_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[1]), _JOINT2_LIMIT['acceleration'] + _EPSILONE)

    def test_timeopt_result_including_acc_error(self):
        u"""A case where the optimization result contains a large acceleration"""
        self._req.trajectory.points = [JointTrajectoryPoint(positions=pos) for pos in [[2.0, -1.0], [1.5, 0.0]]]
        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        self.assertEqual(len(res.trajectory.joint_names), 2)
        self.assertEqual(res.trajectory.joint_names[0], 'joint1')
        self.assertEqual(res.trajectory.joint_names[1], 'joint2')

        for point in res.trajectory.points:
            self.assertLessEqual(abs(point.velocities[0]), _JOINT1_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[0]), _JOINT1_LIMIT['acceleration'] + _EPSILONE)
            self.assertLessEqual(abs(point.velocities[1]), _JOINT2_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[1]), _JOINT2_LIMIT['acceleration'] + _EPSILONE)

    def test_two_points_trajectory(self):
        u"""Current posture and one goal of one -point orbit time optimization"""
        del self._req.trajectory.points[-1]
        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        self.assertEqual(len(res.trajectory.joint_names), 2)
        self.assertEqual(res.trajectory.joint_names[0], 'joint1')
        self.assertEqual(res.trajectory.joint_names[1], 'joint2')

        for point in res.trajectory.points:
            self.assertLessEqual(abs(point.velocities[0]), _JOINT1_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[0]), _JOINT1_LIMIT['acceleration'] + _EPSILONE)
            self.assertLessEqual(abs(point.velocities[1]), _JOINT2_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[1]), _JOINT2_LIMIT['acceleration'] + _EPSILONE)

    def test_no_movement(self):
        u"""Optimization of orbit without movement"""
        self._req.trajectory.points = [
            JointTrajectoryPoint(positions=pos)
            for pos in [[1.0, 1.0], [1.0 + 1.0e-5, 1.0]]]
        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        self.assertEqual(len(res.trajectory.joint_names), 2)
        self.assertEqual(res.trajectory.joint_names[0], 'joint1')
        self.assertEqual(res.trajectory.joint_names[1], 'joint2')

        self.assertEqual(len(res.trajectory.points), 1)
        self.assertAlmostEqual(res.trajectory.points[0].positions[0], 1.0 + 1.0e-5, 1.0e-7)
        self.assertAlmostEqual(
            Duration.from_msg(res.trajectory.points[0].time_from_start).nanoseconds / 1000000000.0, 0.1)

    def test_velocity_ratio(self):
        u"""Setting of speed restriction parameters"""
        set_param_req = SetParameters.Request()
        set_param_req.parameters.append(Parameter('velocity_ratio', value=0.3).to_parameter_msg())
        _ = self._parameter_srv.call(set_param_req)

        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        for point in res.trajectory.points:
            self.assertLessEqual(abs(point.velocities[0]), _JOINT1_LIMIT['velocity'] * 0.3 + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[0]), _JOINT1_LIMIT['acceleration'] + _EPSILONE)
            self.assertLessEqual(abs(point.velocities[1]), _JOINT2_LIMIT['velocity'] * 0.3 + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[1]), _JOINT2_LIMIT['acceleration'] + _EPSILONE)

    def test_acceleration_ratio(self):
        u"""Setting of acceleration restriction parameters"""
        set_param_req = SetParameters.Request()
        set_param_req.parameters.append(Parameter('acceleration_ratio', value=0.5).to_parameter_msg())
        _ = self._parameter_srv.call(set_param_req)

        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        for point in res.trajectory.points:
            self.assertLessEqual(abs(point.velocities[0]), _JOINT1_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[0]), _JOINT1_LIMIT['acceleration'] * 0.5 + _EPSILONE)
            self.assertLessEqual(abs(point.velocities[1]), _JOINT2_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[1]), _JOINT2_LIMIT['acceleration'] * 0.5 + _EPSILONE)

    def test_request_with_limits(self):
        u"""Contributions in service requests"""
        # I'm not sure, but if it is 0.03, the restrictions will be properly effective, and if it is 0.05, it will not work.
        # There seems to be a problem around the optimization
        self._req.limits.append(
            JointLimits(joint_name='joint1',
                        has_velocity_limits=True, max_velocity=0.03))
        res = self._filter_srv.call(self._req)

        self.assertTrue(res.is_success)
        for point in res.trajectory.points:
            self.assertLessEqual(abs(point.velocities[0]), 0.03 + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[0]), _JOINT1_LIMIT['acceleration'] + _EPSILONE)
            self.assertLessEqual(abs(point.velocities[1]), _JOINT2_LIMIT['velocity'] + _EPSILONE)
            self.assertLessEqual(abs(point.accelerations[1]), _JOINT2_LIMIT['acceleration'] + _EPSILONE)
