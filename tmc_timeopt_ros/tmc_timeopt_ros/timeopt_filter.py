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
import sys

import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from tmc_timeopt.timeopt import Timeopt
from tmc_timeopt.trajectory import LinearTrajectory
from tmc_timeopt.trajectory import NaturalCubicSplineTrajectory
from tmc_timeopt.trajectory import TrajectoryDict
from tmc_timeopt_ros.simple_joint_target import SimpleJointTarget
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


# Experimental determined, even if you repeat Move_TO_GO and Move_TO_NEUTRAL in HSR, 1.0E-3 often gets caught.
# More loose feeling
_EPSILON = 1.0e-2


def _extract_acc_limit_dict(joint_names, limits):
    acc_limit_dict = {}
    for name in joint_names:
        acc_limit_dict[name] = limits.get((name, 'acceleration'), None)
    return acc_limit_dict


def _is_in_acc_limit(joint_names, accelerations, acc_limit_dict):
    for name, acc in zip(joint_names, accelerations):
        if acc_limit_dict[name] is None:
            continue
        if (acc < acc_limit_dict[name][0] - _EPSILON) or (acc > acc_limit_dict[name][1] + _EPSILON):
            print(name, acc, acc_limit_dict[name])
            return False
    return True


def _ros_trajectory_from_timeopt(
        trajectory, joint_names, logger, acc_limit_dict, min_step=0.01, offset=0.0):
    u"""Convert the orbit that can be taken with get_optimal_trajectory of Timeopt to ROS

    Args:
        trajectory: timeopt trajectory (list  (time, state) )
        joint_names: list of output joints (list (str) )
        min_step: minumum time between points[s] (float)
        offset: time offset of first point in trajectory[s] (float)
    Return:
        ros_trajectory: ROS trajectory_msg (trajectory_msg.JointTrajectory)
    """
    msg = JointTrajectory()
    trajectory_points = []
    last_time = -float('inf')
    for point in trajectory:
        if (point[0] - last_time) < min_step:
            continue
        joint_point = JointTrajectoryPoint(
            positions=[point[1][name][0] for name in joint_names],
            velocities=[point[1][name][1] for name in joint_names],
            accelerations=[point[1][name][2] for name in joint_names],
            effort=[],
            time_from_start=Duration(seconds=point[0] + offset).to_msg())
        if _is_in_acc_limit(joint_names, joint_point.accelerations, acc_limit_dict):
            trajectory_points.append(joint_point)
            last_time = point[0]
        else:
            logger.error('Acc error')
    msg = JointTrajectory(joint_names=joint_names,
                          points=trajectory_points)
    return msg


def _timeopt_trajectory_from_ros(start_state,
                                 trajectory_msg,
                                 target_joint_names,
                                 decimate_threshold=1e-3):
    u"""Convert the ROS orbit so that it can be handled with Timeopt

    Args:
        start_state: Initial state.
        trajectory_msg: ROS joint orbit (Trajectory_msg/JointTrajecotry)
        target_joint_names: Orbit optimized with Timeopt (List (str))
        decimate_threshold: Skills that are close to 2 points.
    Return:
        Timeopt_trajectory: Trajectory (TrajectoryDict) used in Timeopt
    Note:
        If there is no point that changes from the initial state, return None
    """
    points = trajectory_msg.points
    joint_names = trajectory_msg.joint_names

    # If there is almost the same point, the calculation will be unstable.
    # The same point is unnecessary because of the shortest time control characteristics
    decimate_point = []
    prev_point = [start_state.position[start_state.name.index(joint)]
                  for joint in joint_names]
    for point in points:
        if np.linalg.norm(
                np.array(prev_point) - np.array(point.positions)) > decimate_threshold:
            decimate_point.append(
                JointTrajectoryPoint(positions=point.positions))
            prev_point = point.positions
    points = decimate_point
    point_num = len(points) + 1

    # If there is no score
    if (point_num == 1):
        return None

    traj = TrajectoryDict(point_num)
    if (point_num == 2):
        # If there are 2 points, use a straight orbital.
        for name in target_joint_names:
            traj.append(name, LinearTrajectory)
    else:
        # Usually interpolate with a tertiary natural sprine
        for name in target_joint_names:
            traj.append(name, NaturalCubicSplineTrajectory)

    # Enter the point of Start_state as the starting point
    for index, name in enumerate(start_state.name):
        if name in traj:
            traj[name][0] = (start_state.position[index], 0, 0)
    for name in set(target_joint_names) - set(start_state.name):
        traj[name][0] = (0, 0, 0)

    # Settings via points
    for index, name in enumerate(joint_names):
        for i in range(point_num - 1):
            traj[name][i + 1] = (points[i].positions[index], 0, 0)
    for name in set(target_joint_names) - set(joint_names):
        for i in range(point_num - 1):
            traj[name][i + 1] = (0, 0, 0)
    return traj


class TimeoptFilterNode(Node):
    u"""Node that converts a spatial command orbital into a time orbit"""

    _DEFAULT_ACCELERATION_LIMIT = 1.0
    _DEFAULT_VELOCITY_LIMIT = 1.0

    _DECIMATE_THRESHOLD = 1e-3
    _TIMEOPT_RESOLUTION_S = 0.2
    _MINIMUM_DT = 0.1

    def __init__(self, srv_name='filter_trajectory', default_joint=None):
        u"""Initialization"""
        super().__init__('timeopt_filter_node')

        self._use_joint = self._get_param('use_joint', default_joint)
        self._target = SimpleJointTarget(self._use_joint)

        self._timeopt = Timeopt(self._target)
        self._kinematics = self._timeopt.kinematics()
        self._dynamics = self._timeopt.dynamics()

        self.add_on_set_parameters_callback(self._parameters_callback)
        self.declare_parameter('velocity_ratio', 1.0)
        self.declare_parameter('acceleration_ratio', 1.0)

        self._vel_limit = {joint: self._DEFAULT_VELOCITY_LIMIT for joint in self._use_joint}
        self._acc_limit = {joint: self._DEFAULT_ACCELERATION_LIMIT for joint in self._use_joint}
        for joint_name in self._use_joint:
            for param_name, limits, info_name in [('acceleration', self._acc_limit, 'dynamics'),
                                                  ('velocity', self._vel_limit, 'kinematics')]:
                name = joint_name + '.' + param_name
                value = self._get_param(name, limits[joint_name])
                if value <= 0.0:
                    self.get_logger().fatal('limit have to be positive')
                    sys.exit()
                limits[joint_name] = value
                self.get_logger().info(f'{info_name}.limit:{name} {value}')

        self._minimum_dt = self._get_param('minimum_dt', self._MINIMUM_DT)
        if self._minimum_dt <= 0.0:
            self.get_logger().fatal('minimum_dt have to be positive')
            sys.exit()

        self._timeopt_resolution = self._get_param('timeopt_resolution', self._TIMEOPT_RESOLUTION_S)
        if not (0.0 < self._timeopt_resolution <= 1.0):
            self.get_logger().fatal('timeopt_resolution have to be (0.0, 1.0]')
            sys.exit()

        self._srv = self.create_service(FilterJointTrajectory, srv_name, self._callback_timeopt_filter)

    def _get_param(self, name, default_value):
        self.declare_parameter(name, default_value)
        return self.get_parameter(name).value

    def _update_velocity_limit(self, joint_name, value):
        u"""Overwrite speed constraints"""
        self._kinematics.set_limit(joint_name, 'velocity',
                                   (-self._velocity_ratio * value, self._velocity_ratio * value))

    def _update_acceleration_limit(self, joint_name, value):
        u"""Overwrite acceleration constraints"""
        limits = (-self._acceleration_ratio * value, self._acceleration_ratio * value)
        self._kinematics.set_limit(joint_name, 'acceleration', limits)
        self._dynamics.set_limit(joint_name, 'acceleration', limits)

    def _set_joint_limit_request(self, joint_limits):
        u"""Set TMC_MANIPULATION_MSGS/JINTLIMITS for the request"""
        for limit in joint_limits:
            if limit.has_velocity_limits:
                self._update_velocity_limit(limit.joint_name, limit.max_velocity)
            if limit.has_acceleration_limits:
                self._update_acceleration_limit(limit.joint_name, limit.max_acceleration)

    def _reset_joint_limits(self):
        u"""Reset speed and acceleration constraints with current parameters"""
        for joint in self._use_joint:
            self._update_velocity_limit(joint, self._vel_limit[joint])
            self._update_acceleration_limit(joint, self._acc_limit[joint])

    def _parameters_callback(self, params):
        for param in params:
            if param.name == 'velocity_ratio':
                self._velocity_ratio = param.value
                self.get_logger().info(f'change vel ratio {self._velocity_ratio}')
            elif param.name == 'acceleration_ratio':
                self._acceleration_ratio = param.value
                self.get_logger().info(f'change acc ratio {self._acceleration_ratio}')
        return SetParametersResult(successful=True)

    def _timeopt_trajectory_from_ros(self, start_state, trajectory):
        u"""Convert ROS orbit to Timeopt orbit

        Functionation to implement robot -specific processing by overlighting
        """
        return _timeopt_trajectory_from_ros(
            start_state, trajectory, self._target.names,
            self._DECIMATE_THRESHOLD)

    def _set_trajectory(self, timeopt_trajectory):
        u"""Set the orbit to the time optimization instance

        Functionation to implement robot -specific processing by overlighting
        """
        self._timeopt.set_trajectory(timeopt_trajectory)

    def _callback_timeopt_filter(self, req, res):
        u"""Filter_trajectory service callback.

        Receive command spaces (TMC_MANIPULATION/FilterJointtrajectory)
        Return the shortest time orbit (JointTrajectory)
        """
        self.get_logger().debug('\n%s' % req)
        start = self.get_clock().now()

        self._reset_joint_limits()
        self._set_joint_limit_request(req.limits)

        try:
            timeopt_traj = self._timeopt_trajectory_from_ros(req.start_state.joint_state, req.trajectory)
        except Exception as e:
            self.get_logger().error('A exception occured {0}'.format(e))
            res.trajectory = JointTrajectory()
            res.is_success = False
            return res

        if timeopt_traj is None:
            # If there is no point through, return the trajectory with the minimum time without optimization.
            self.get_logger().info('no movement')
            output_traj = JointTrajectory()
            output_traj.joint_names = req.trajectory.joint_names
            point = req.trajectory.points[-1]
            point.time_from_start = Duration(seconds=self._minimum_dt).to_msg()
            output_traj.points = [point]

            res.trajectory = output_traj
            res.is_success = True
            return res

        self._set_trajectory(timeopt_traj)
        try:
            self._timeopt.preprocess(self._timeopt_resolution)
            self._timeopt.update()
        except Exception as e:
            self.get_logger().error('A exception occured in timeopt {0}'.format(e))
            res.trajectory = JointTrajectory()
            res.is_success = False
            return res

        trajectory = _ros_trajectory_from_timeopt(
            self._timeopt.get_optimal_trajectory(),
            req.trajectory.joint_names,
            self.get_logger(),
            _extract_acc_limit_dict(req.trajectory.joint_names, self._kinematics.limits))
        # The first point is for calculation, so delete it
        trajectory.points = trajectory.points[1:]
        res.trajectory = trajectory
        res.is_success = True

        self.get_logger().debug('req.traj = ')
        self.get_logger().debug(req.trajectory)
        self.get_logger().debug('res.traj = ')
        self.get_logger().debug(res.trajectory)

        end = self.get_clock().now()
        self.get_logger().info('elapsed time = {0}'.format((end - start).nanoseconds / 1000000000.0))
        return res


def main():
    rclpy.init()
    node = TimeoptFilterNode()

    rclpy.spin(node)

    rclpy.try_shutdown()
    node.destroy_node()
