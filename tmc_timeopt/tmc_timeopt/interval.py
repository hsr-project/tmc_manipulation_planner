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
u"""Module to manage intervals."""

from copy import deepcopy


def join(a, b):
    u"""Take the union of a and b."""
    if a[1] < b[0]:
        return [a, b]
    if a[0] > b[1]:
        return [b, a]
    if a[0] <= b[0]:
        if a[1] <= b[1]:
            return [a[0], b[1]]
        else:
            return a
    if b[0] <= a[0]:
        if b[1] <= a[1]:
            return [b[0], a[1]]
        else:
            return b


def meet(a, b):
    u"""Take the intersection of a and b."""
    if a[1] < b[0] or a[0] > b[1]:
        return []
    if a[0] >= b[0]:
        if a[1] > b[1]:
            return [a[0], b[1]]
        else:
            return a
    if b[0] >= a[0]:
        if b[1] > a[1]:
            return [b[0], a[1]]
        else:
            return b


class Interval(object):
    u"""Class representing intervals expressed by multiple inequalities."""

    def __init__(self, lst=[]):
        u"""Initialize with a list.

        Args:
            lis: Initialize with a list.
        """
        if not lst:
            self.list = []
            return
        if isinstance(lst[0], list):
            self.list = lst
        else:
            self.list = [lst]
        self.merge()

    def get_list(self):
        u"""Return as a list."""
        return self.list

    def set_list(self, lst):
        u"""Set using a list."""
        if isinstance(lst[0], list):
            self.list = lst
        else:
            self.list = [lst]

    def __add__(a, b):
        c = deepcopy(a)
        c.list.extend(b.list)
        c.merge()
        return c

    def __mul__(a, b):
        c = Interval()
        if a.list == [] or b.list == []:
            return []
        for i in a.list:
            for j in b.list:
                m = meet(i, j)
                if m:
                    c.list.append(m)
        return c

    def __repr__(self):
        return repr(self.list)

    def merge(self):
        u"""Organize the set."""
        self.list.sort()
        # Take the union of adjacent sets
        i = 0
        while i != len(self.list) - 1:
            if meet(self.list[i], self.list[i + 1]):
                self.list[i:i + 2] = [join(self.list[i], self.list[i + 1])]
            else:
                i += 1
            if i == len(self.list) - 1:
                break
