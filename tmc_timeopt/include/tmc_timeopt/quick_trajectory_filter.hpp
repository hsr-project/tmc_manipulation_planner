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
#ifndef TMC_TIMEOPT_QUICK_TRAJECTORY_FILTER_HPP_
#define TMC_TIMEOPT_QUICK_TRAJECTORY_FILTER_HPP_

#include <memory>
#include <vector>

#include <tmc_timeopt/trajectory_filter.hpp>

namespace tmc_timeopt {

class Trajectory;

class QuickTrajectoryFilter : public ITrajectoryFilter {
 public:
  // constructor
  // @param[in] initial_positions Current joint position
  // @param[in] initial_velocities Current joint velocity
  // @param[in] way_points Joint position
  // @param[in] max_velocities Maximum values ​​of joint velocity
  // @param[in] max_accelerations Maximum value of joint acceleration
  QuickTrajectoryFilter(const Eigen::VectorXd& initial_positions,
                        const Eigen::VectorXd& initial_velocities,
                        const std::vector<Eigen::VectorXd>& way_points,
                        const Eigen::VectorXd& max_velocities,
                        const Eigen::VectorXd& max_accelerations);

  // constructor
  // @param[in] initial_positions Current joint position
  // @param[in] initial_velocities Current joint velocity
  // @param[in] way_points Joint position
  // @param[in] max_velocities Maximum values ​​of joint velocity
  // @param[in] max_accelerations Maximum value of joint acceleration
  // @param[in] interrupt Interrupt function
  QuickTrajectoryFilter(const Eigen::VectorXd& initial_positions,
                        const Eigen::VectorXd& initial_velocities,
                        const std::vector<Eigen::VectorXd>& way_points,
                        const Eigen::VectorXd& max_velocities,
                        const Eigen::VectorXd& max_accelerations,
                        std::function<bool()>& interrupt);
  virtual ~QuickTrajectoryFilter() = default;

  // Get the joint position
  // @param[in] time_from_start Time [sec]
  Eigen::VectorXd GetPosition(double time_from_start) const override;

  // Get the joint velocity
  // @param[in] time_from_start Time [sec]
  Eigen::VectorXd GetVelocity(double time_from_start) const override;

  // Get the path playback time
  double GetDuration() const override;

  // Get whether you have succeeded in optimization
  // @return If you succeed in the Bool optimization and the GET function is available, True
  bool IsValid() const override { return trajectory_ != nullptr; }

 private:
  std::shared_ptr<Trajectory> trajectory_;
};

}  // namespace tmc_timeopt
#endif  // TMC_TIMEOPT_QUICK_TRAJECTORY_FILTER_HPP_
