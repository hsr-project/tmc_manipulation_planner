/*
Copyright (c) 2026 TOYOTA MOTOR CORPORATION
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
/// @file     rrt_planner.hpp
/// @brief Implementation of PointToConditionPlanner using rrt_planner
/// @author   Koji Terada
/// @version  1.0.0
/// @date     2011.10.25
/// @note     [1.0.0] 2011.10.19 Newly created

#ifndef TMC_MANIPULATION_TMC_RPLANNER_RRT_PLANNER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_RRT_PLANNER_HPP_

#include <ctime>
#include <random>

#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/point_to_condition_planner.hpp>


namespace tmc_rplanner {

class ConfigurationTree;


/// @class rrt_Planner
/// @brief Planner implementing RRT
/// @note See J.J. Kuffner and S.M. LaValle. RRT-Connect:
///       An efficient approach to single-query path planning.
///       In Proc. IEEE Int’l Conf. on Robotics and Automation (ICRA‘2000
///       pages 995-1001, San Francisco, CA, April 2000.
class RrtPlanner : public IPointToConditionPlanner {
 public:
  /// @brief Pass the planner space and termination conditions
  /// @param space Pointer to the planner space
  /// @param delta Exploration step size
  /// @param max_itr Maximum number of iterations
  /// @param goal_bias Ratio of aiming for the goal [0.0-1.0]
  /// @param greedy Flag indicating whether to continue extending feasibly when aiming for the goal
  /// @param is_terminate Forced termination condition
  RrtPlanner(ConfigurationSpace::Ptr space,
             double delta,
             int32_t max_itr,
             double goal_bias,
             bool greedy,
             TerminateConditionFunc is_terminate) :
      space_(space), delta_(delta), max_itr_(max_itr),
      goal_bias_(goal_bias), greedy_(greedy), is_terminate_(is_terminate),
      eng_(static_cast<uint32_t>(std::time(0))) {}
  /// @brief Pass the planner space and termination conditions
  /// @param space Pointer to the planner space
  /// @param delta Exploration step size
  /// @param max_itr Maximum number of iterations
  /// @param goal_bias Ratio of aiming for the goal [0.0-1.0]
  /// @param greedy Flag indicating whether to continue extending feasibly when aiming for the goal
  RrtPlanner(ConfigurationSpace::Ptr space,
             double delta,
             int32_t max_itr,
             double goal_bias,
             bool greedy) :
      space_(space), delta_(delta), max_itr_(max_itr),
      goal_bias_(goal_bias), greedy_(greedy),
      eng_(static_cast<uint32_t>(std::time(0))) {}

  virtual ~RrtPlanner() {}
  /// Path creation
  virtual PlanRet PlanPath(const Config& init_config,
                           Path& path_out);

 private:
  // Prohibit copying
  RrtPlanner(const RrtPlanner&);
  RrtPlanner& operator=(const RrtPlanner&);
  bool BuildOneStep_(ConfigurationTree& tree);
  const ConfigurationSpace::Ptr space_;
  const double delta_;
  const int32_t max_itr_;
  const double goal_bias_;
  const bool greedy_;
  TerminateConditionFunc is_terminate_;
  std::mt19937 eng_;
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_RRT_PLANNER_HPP_
