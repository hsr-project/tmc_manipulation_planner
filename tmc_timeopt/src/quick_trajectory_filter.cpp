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

  // Behavior of normalized when initial_velocities is a zero vector
  // Varies depending on the version of Eigen
  // Returns a vector with nan elements in older versions (3.3~beta1-2) and a zero vector in newer versions (3.3.4-4)
  // To accommodate both, process by branching with isZero()
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
    // Place waypoints in the direction of initial velocity from initial_positions to accommodate any initial velocity
    //
    // Of the path from the first point to the second point, the part extending from the first point becomes a straight segment
    // If the direction of the straight segment is the direction of initial velocity, the given initial velocity can be achieved
    // The length of the straight segment should be shorter, but since the formula to derive the minimum value is unknown, it will be explored
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

// Obtain joint velocity at time_from_start
// @param[in] time_from_start  Time [sec] at which joint velocity is to be obtained
// @return Eigen::VectorXd  Joint velocity
Eigen::VectorXd QuickTrajectoryFilter::GetVelocity(
    double time_from_start) const {
  if (IsValid()) {
    return trajectory_->getVelocity(time_from_start);
  } else {
    return Eigen::VectorXd();
  }
}

// Obtain the playback time of the trajectory
// @return double  Playback time of the trajectory [sec]
double QuickTrajectoryFilter::GetDuration() const {
  if (IsValid()) {
    return trajectory_->getDuration();
  } else {
    return 0.0;
  }
}
}  // namespace tmc_timeopt
