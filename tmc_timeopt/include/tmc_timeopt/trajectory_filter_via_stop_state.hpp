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
#ifndef TMC_TIMEOPT_TRAJECTORY_FILTER_VIA_STOP_STATE_HPP_
#define TMC_TIMEOPT_TRAJECTORY_FILTER_VIA_STOP_STATE_HPP_

#include <memory>
#include <vector>

#include <tmc_timeopt/trajectory_filter.hpp>

namespace tmc_timeopt {

class Trajectory;

class TrajectoryFilterViaStopState : public ITrajectoryFilter {
 public:
  // Constructor
  // @param[in] initial_positions  Current joint positions
  // @param[in] initial_velocities  Current joint velocities
  // @param[in] way_points  Intermediate joint positions
  // @param[in] max_velocities  Maximum joint velocities
  // @param[in] max_accelerations  Maximum joint accelerations
  TrajectoryFilterViaStopState(const Eigen::VectorXd& initial_positions,
                               const Eigen::VectorXd& initial_velocities,
                               const std::vector<Eigen::VectorXd>& way_points,
                               const Eigen::VectorXd& max_velocities,
                               const Eigen::VectorXd& max_accelerations);

  // Constructor
  // @param[in] initial_positions  Current joint positions
  // @param[in] initial_velocities  Current joint velocities
  // @param[in] way_points  Intermediate joint positions
  // @param[in] max_velocities  Maximum joint velocities
  // @param[in] max_accelerations  Maximum joint accelerations
  // @param[in] interrupt  Interrupt function, interrupts optimization process if returns true
  TrajectoryFilterViaStopState(const Eigen::VectorXd& initial_positions,
                               const Eigen::VectorXd& initial_velocities,
                               const std::vector<Eigen::VectorXd>& way_points,
                               const Eigen::VectorXd& max_velocities,
                               const Eigen::VectorXd& max_accelerations,
                               std::function<bool()>& interrupt);

  // Constructor
  // @param[in] initial_positions  Current joint positions
  // @param[in] initial_velocities  Current joint velocities
  // @param[in] way_points  Intermediate joint positions
  // @param[in] max_velocities  Maximum joint velocities
  // @param[in] max_accelerations  Maximum joint accelerations
  // @param[in] acceleration_rate_for_stop  Acceleration rate for initial stop
  // @param[in] interrupt  Interrupt function, interrupts optimization process if returns true
  TrajectoryFilterViaStopState(const Eigen::VectorXd& initial_positions,
                               const Eigen::VectorXd& initial_velocities,
                               const std::vector<Eigen::VectorXd>& way_points,
                               const Eigen::VectorXd& max_velocities,
                               const Eigen::VectorXd& max_accelerations,
                               double acceleration_rate_for_stop,
                               std::function<bool()>& interrupt);
  virtual ~TrajectoryFilterViaStopState() = default;

  // Get joint positions at time_from_start
  // @param[in] time_from_start  Time at which to get joint positions [sec]
  // @return Eigen::VectorXd  Joint positions
  Eigen::VectorXd GetPosition(double time_from_start) const override;

  // Get joint velocities at time_from_start
  // @param[in] time_from_start  Time at which to get joint velocities [sec]
  // @return Eigen::VectorXd  Joint velocities
  Eigen::VectorXd GetVelocity(double time_from_start) const override;

  // Get trajectory playback time
  // @return double  Trajectory playback time [sec]
  double GetDuration() const override;

  // Check if optimization was successful
  // @return bool  True if optimization was successful and Get functions are available
  bool IsValid() const override { return is_valid_; }

 private:
  bool is_valid_;

  std::shared_ptr<Trajectory> trajectory_;

  Eigen::VectorXd initial_positions_;
  Eigen::VectorXd initial_velocities_;
  Eigen::VectorXd accelerations_for_stop_;
  double stop_duration_;
};

}  // namespace tmc_timeopt
#endif  // TMC_TIMEOPT_TRAJECTORY_FILTER_VIA_STOP_STATE_HPP_
