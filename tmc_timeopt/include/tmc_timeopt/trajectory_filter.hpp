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
#ifndef TMC_TIMEOPT_TRAJECTORY_FILTER_HPP_
#define TMC_TIMEOPT_TRAJECTORY_FILTER_HPP_

#include <memory>

#include <Eigen/Core>

namespace tmc_timeopt {

class ITrajectoryFilter {
 public:
  using Ptr = std::shared_ptr<ITrajectoryFilter>;

  virtual ~ITrajectoryFilter() = default;

  // Get joint position at time_from_start
  // @param[in] time_from_start  Time [sec] at which joint position is desired
  // @return Eigen::VectorXd  Joint position
  virtual Eigen::VectorXd GetPosition(double time_from_start) const = 0;

  // Get joint velocity at time_from_start
  // @param[in] time_from_start  Time [sec] at which joint velocity is desired
  // @return Eigen::VectorXd  Joint velocity
  virtual Eigen::VectorXd GetVelocity(double time_from_start) const = 0;

  // Get playback time of the trajectory
  // @return double  Playback time of the trajectory [sec]
  virtual double GetDuration() const = 0;

  // Check if optimization was successful
  // @return bool  Returns true if optimization was successful and Get functions are available
  virtual bool IsValid() const = 0;
};

}  // namespace tmc_timeopt
#endif  // TMC_TIMEOPT_TRAJECTORY_FILTER_HPP_
