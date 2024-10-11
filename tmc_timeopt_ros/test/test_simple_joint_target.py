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
from nose.tools import eq_
from tmc_timeopt_ros.simple_joint_target import SimpleJointTarget


def test_calc_dynamics():
    u"""Update dynamics"""
    target = SimpleJointTarget(['j1', 'j2', 'j3'])
    target.point = {}
    target.point['j1'] = (0, 1.0, 2.0)
    target.point['j2'] = (0, 3.0, 4.0)
    target.point['j3'] = (0, 5.0, 6.0)
    (a, b, c, d) = target.get_dynamics()

    eq_(a['j1', 'acceleration'], 1.0)
    eq_(b['j1', 'acceleration'], 2.0)
    eq_(c['j1', 'acceleration'], 0.0)
    eq_(d['j1', 'acceleration'], 0.0)
    eq_(a['j2', 'acceleration'], 3.0)
    eq_(b['j2', 'acceleration'], 4.0)
    eq_(c['j2', 'acceleration'], 0.0)
    eq_(d['j2', 'acceleration'], 0.0)
    eq_(a['j3', 'acceleration'], 5.0)
    eq_(b['j3', 'acceleration'], 6.0)
    eq_(c['j3', 'acceleration'], 0.0)
    eq_(d['j3', 'acceleration'], 0.0)
