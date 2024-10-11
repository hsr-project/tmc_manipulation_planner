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
#ifndef TMC_TIMEOPT_TRAJECTORY_FILTER_UTILS_HPP_
#define TMC_TIMEOPT_TRAJECTORY_FILTER_UTILS_HPP_

#include <vector>

#include <Eigen/Core>

namespace tmc_timeopt {
// Experimental parameters
constexpr double kMaxDeviation = 0.03;
constexpr double kTimeStep = 0.001;
constexpr double kMinPointsDistance = 1e-6;


// Judge whether or not the same point
bool IsSameWayPoints(const Eigen::VectorXd& frist, const Eigen::VectorXd& second);

// In the algorithm, if the same point is continuous, it will be unstable, so take out only valid points.
bool ExtractValidWayPoints(const Eigen::VectorXd& initial_positions,
                           const std::vector<Eigen::VectorXd>& way_points_in,
                           std::vector<Eigen::VectorXd>& way_points_out);

// Validation for input
bool ValidateInput(const Eigen::VectorXd& initial_positions,
                   const Eigen::VectorXd& initial_velocities,
                   const std::vector<Eigen::VectorXd>& way_points,
                   const Eigen::VectorXd& max_velocities,
                   const Eigen::VectorXd& max_accelerations);

}  // namespace tmc_timeopt

#endif  // TMC_TIMEOPT_TRAJECTORY_FILTER_UTILS_HPP_
