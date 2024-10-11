/*
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
*/
#include <tmc_timeopt/trajectory_filter_via_stop_state.hpp>

#include <list>

#include <console_bridge/console.h>

#include "Path.hpp"
#include "Trajectory.hpp"
#include "trajectory_filter_utils.hpp"

namespace {
std::function<bool()> ReturnFalse = []() -> bool{ return false; };
}  // namespace

namespace tmc_timeopt {

TrajectoryFilterViaStopState::TrajectoryFilterViaStopState(const Eigen::VectorXd& initial_positions,
                                                           const Eigen::VectorXd& initial_velocities,
                                                           const std::vector<Eigen::VectorXd>& way_points,
                                                           const Eigen::VectorXd& max_velocities,
                                                           const Eigen::VectorXd& max_accelerations)
    : TrajectoryFilterViaStopState(initial_positions, initial_velocities, way_points, max_velocities,
                                   max_accelerations, 1.0, ReturnFalse) {}

TrajectoryFilterViaStopState::TrajectoryFilterViaStopState(const Eigen::VectorXd& initial_positions,
                                                           const Eigen::VectorXd& initial_velocities,
                                                           const std::vector<Eigen::VectorXd>& way_points,
                                                           const Eigen::VectorXd& max_velocities,
                                                           const Eigen::VectorXd& max_accelerations,
                                                           std::function<bool()>& interrupt)
    : TrajectoryFilterViaStopState(initial_positions, initial_velocities, way_points, max_velocities,
                                   max_accelerations, 1.0, interrupt) {}

TrajectoryFilterViaStopState::TrajectoryFilterViaStopState(const Eigen::VectorXd& initial_positions,
                                                           const Eigen::VectorXd& initial_velocities,
                                                           const std::vector<Eigen::VectorXd>& way_points,
                                                           const Eigen::VectorXd& max_velocities,
                                                           const Eigen::VectorXd& max_accelerations,
                                                           double acceleration_rate_for_stop,
                                                           std::function<bool()>& interrupt)
    : is_valid_(false), trajectory_(nullptr) {
  if (!ValidateInput(initial_positions, initial_velocities, way_points, max_velocities, max_accelerations)) {
    return;
  }
  std::vector<Eigen::VectorXd> valid_way_points;
  if (!ExtractValidWayPoints(initial_positions, way_points, valid_way_points)) {
    CONSOLE_BRIDGE_logError("No valid way points");
    return;
  }

  std::list<Eigen::VectorXd> way_points_impl;
  if (initial_velocities.isZero()) {
    stop_duration_ = 0.0;

    way_points_impl.push_back(initial_positions);
  } else {
    const auto max_accelerations_for_stop = acceleration_rate_for_stop * max_accelerations;
    stop_duration_ = (initial_velocities.array() / max_accelerations_for_stop.array()).abs().maxCoeff();
    const auto stop_positions = initial_positions + 0.5 * stop_duration_ * initial_velocities;

    initial_positions_ = initial_positions;
    initial_velocities_ = initial_velocities;
    accelerations_for_stop_ = initial_velocities.array() / -stop_duration_;

    if (!IsSameWayPoints(stop_positions, valid_way_points.front())) {
      way_points_impl.push_back(stop_positions);
    } else if (valid_way_points.size() == 1) {
      // One point that is substantially stopped and once stops, is unnecessary if the voting points are used.
      is_valid_ = true;
      return;
    }
  }
  way_points_impl.insert(way_points_impl.end(), valid_way_points.begin(),
                         valid_way_points.end());

  auto path = Path(way_points_impl, kMaxDeviation);
  trajectory_ = std::make_shared<Trajectory>(path, max_velocities, max_accelerations, 0.0, interrupt, kTimeStep);
  is_valid_ = trajectory_->isValid();
}

Eigen::VectorXd TrajectoryFilterViaStopState::GetPosition(
    double time_from_start) const {
  if (IsValid()) {
    if (time_from_start < stop_duration_) {
      return (initial_positions_ + initial_velocities_ * time_from_start
              + 0.5 * accelerations_for_stop_ * time_from_start * time_from_start);
    } else if (trajectory_ == nullptr) {
      return (initial_positions_ + initial_velocities_ * stop_duration_
              + 0.5 * accelerations_for_stop_ * stop_duration_ * stop_duration_);
    } else {
      return trajectory_->getPosition(time_from_start - stop_duration_);
    }
  } else {
    return Eigen::VectorXd();
  }
}

Eigen::VectorXd TrajectoryFilterViaStopState::GetVelocity(
    double time_from_start) const {
  if (IsValid()) {
    if (time_from_start < stop_duration_) {
      return initial_velocities_ + accelerations_for_stop_ * time_from_start;
    } else if (trajectory_ == nullptr) {
      return initial_velocities_ + accelerations_for_stop_ * stop_duration_;
    } else {
      return trajectory_->getVelocity(time_from_start - stop_duration_);
    }
  } else {
    return Eigen::VectorXd();
  }
}

double TrajectoryFilterViaStopState::GetDuration() const {
  if (IsValid()) {
    if (trajectory_ == nullptr) {
      return stop_duration_;
    } else {
      return stop_duration_ + trajectory_->getDuration();
    }
  } else {
    return 0.0;
  }
}

}  // namespace tmc_timeopt
