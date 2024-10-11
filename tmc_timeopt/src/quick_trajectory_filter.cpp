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
/// @brief Trajectory filter
#include <tmc_timeopt/quick_trajectory_filter.hpp>

#include <console_bridge/console.h>

#include "Path.hpp"
#include "Trajectory.hpp"
#include "trajectory_filter_utils.hpp"

namespace {
std::function<bool()> ReturnFalse = []() -> bool{ return false; };
}  // namespace

namespace tmc_timeopt {

QuickTrajectoryFilter::QuickTrajectoryFilter(const Eigen::VectorXd& initial_positions,
                                             const Eigen::VectorXd& initial_velocities,
                                             const std::vector<Eigen::VectorXd>& way_points,
                                             const Eigen::VectorXd& max_velocities,
                                             const Eigen::VectorXd& max_accelerations)
    : QuickTrajectoryFilter(initial_positions, initial_velocities, way_points, max_velocities, max_accelerations,
                            ReturnFalse) {}

QuickTrajectoryFilter::QuickTrajectoryFilter(const Eigen::VectorXd& initial_positions,
                                             const Eigen::VectorXd& initial_velocities,
                                             const std::vector<Eigen::VectorXd>& way_points,
                                             const Eigen::VectorXd& max_velocities,
                                             const Eigen::VectorXd& max_accelerations,
                                             std::function<bool()>& interrupt)
    : trajectory_(nullptr) {
  if (!ValidateInput(initial_positions, initial_velocities, way_points, max_velocities, max_accelerations)) {
    return;
  }
  std::vector<Eigen::VectorXd> valid_way_points;
  if (!ExtractValidWayPoints(initial_positions, way_points, valid_way_points)) {
    CONSOLE_BRIDGE_logError("No valid way points");
    return;
  }

  // Normalized behavior when INITIAL_VELOCITIES is a zero vector
  // It depends on the version of Eigen
  // Old (3.3 ~ Beta1-2) and elements return the new (3.3.4-4) and zero vector
  // Branch and process with ISZERO () to respond to both
  Eigen::VectorXd init_vel_normalized;
  if (initial_velocities.isZero()) {
    init_vel_normalized = Eigen::VectorXd::Zero(initial_velocities.size());
  } else {
    init_vel_normalized = initial_velocities.normalized();
  }
  auto first_length = (valid_way_points.front() - initial_positions).norm();

  for (double rate = 0.01; rate < 2.0; rate += 0.1) {
    if (interrupt()) {
      CONSOLE_BRIDGE_logInform("Otimization interrupted");
      break;
    }
    // Put a via point in the initial speed direction from Initial_positions to respond to any initial speed
    //
    // Of the second PATH, the part that extends from the first point is a straight line SEGMENT.
    // If the direction of the straight line SEGMENT is the initial speed direction, the initial speed given can be obtained
    // The length of the straight line SEGMENT should be shorter, but I do not know the formula that derives the minimum price, so explore.
    std::list<Eigen::VectorXd> way_points_impl;
    way_points_impl.push_back(initial_positions);
    way_points_impl.push_back(initial_positions +
                              init_vel_normalized * first_length * rate);
    way_points_impl.insert(way_points_impl.end(), valid_way_points.begin(),
                           valid_way_points.end());

    auto path = Path(way_points_impl, kMaxDeviation);
    trajectory_ =
        std::make_shared<Trajectory>(path, max_velocities, max_accelerations,
                                     initial_velocities.norm(), interrupt, kTimeStep);
    if (trajectory_->isValid()) {
      return;
    }
  }
  trajectory_.reset();
}

Eigen::VectorXd QuickTrajectoryFilter::GetPosition(
    double time_from_start) const {
  if (IsValid()) {
    return trajectory_->getPosition(time_from_start);
  } else {
    return Eigen::VectorXd();
  }
}

Eigen::VectorXd QuickTrajectoryFilter::GetVelocity(
    double time_from_start) const {
  if (IsValid()) {
    return trajectory_->getVelocity(time_from_start);
  } else {
    return Eigen::VectorXd();
  }
}

double QuickTrajectoryFilter::GetDuration() const {
  if (IsValid()) {
    return trajectory_->getDuration();
  } else {
    return 0.0;
  }
}
}  // namespace tmc_timeopt
