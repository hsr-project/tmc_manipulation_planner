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
/// @file     birrt_planner.hpp
/// @brief Implementation of PointToPointPlanner using birrt_planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#ifndef TMC_MANIPULATION_TMC_RPLANNER_BIRRT_PLANNER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_BIRRT_PLANNER_HPP_

#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/point_to_point_planner.hpp>

namespace tmc_rplanner {

/// @class Birrt_Planner
/// @brief Two-point planner implementing BIRRT_PLANNER
/// @note See J.J. Kuffner and S.M. LaValle. RRT-Connect:
///       An efficient approach to single-query path planning.
///       In Proc. IEEE Int’l Conf. on Robotics and Automation (ICRA‘2000
///       pages 995-1001, San Francisco, CA, April 2000.
class BiRrtPlanner : public IPointToPointPlanner {
 public:
  /// @brief Pass the planner space and termination conditions
  /// @param space Pointer to the planner space
  /// @param delta Exploration width of BiRRT. This width also becomes the width of the output trajectory.
  /// @param max_itr Maximum number of iterations
  /// @param is_terminate Forced termination condition
  BiRrtPlanner(ConfigurationSpace::Ptr space,
               double delta,
               int32_t max_itr,
               TerminateConditionFunc is_terminate) :
      space_(space), delta_(delta), max_itr_(max_itr),
      is_terminate_(is_terminate) {}
  /// @brief Pass the planner space and termination conditions
  /// @param space Pointer to the planner space
  /// @param max_itr Maximum number of iterations
  BiRrtPlanner(ConfigurationSpace::Ptr space,
               double delta,
               int32_t max_itr) :
      space_(space), delta_(delta), max_itr_(max_itr) {}

  virtual ~BiRrtPlanner() {}
  /// Path creation
  virtual PlanRet PlanPath(const Config& init_config,
                           const Config& goal_config,
                           Path& path_out);

 private:
  // Prohibition of copying
  BiRrtPlanner(const BiRrtPlanner&);
  BiRrtPlanner& operator=(const BiRrtPlanner&);
  const ConfigurationSpace::Ptr space_;
  double delta_;
  const int32_t max_itr_;
  TerminateConditionFunc is_terminate_;
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_BIRRT_PLANNER_HPP_
