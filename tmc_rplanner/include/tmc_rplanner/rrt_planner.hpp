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
/// @brief    Implementation of PointtoConditionPlanner with RRT_PLANNER

#ifndef TMC_MANIPULATION_TMC_RPLANNER_RRT_PLANNER_HPP_
#define TMC_MANIPULATION_TMC_RPLANNER_RRT_PLANNER_HPP_

#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/point_to_condition_planner.hpp>


namespace tmc_rplanner {

class ConfigurationTree;


/// @class rrt_Planner
/// @brief Planna implemented RRT
/// @note See J.J. Kuffner and S.M. LaValle. RRT-Connect:
///       An efficient approach to single-query path planning.
///       In Proc. IEEE Int’l Conf. on Robotics and Automation (ICRA‘2000
///       pages 995-1001, San Francisco, CA, April 2000.
class RrtPlanner : public IPointToConditionPlanner {
 public:
  /// @brief Pass the planner space and the termination conditions
  /// @param space Configuration space
  /// @param delta Exploration carved width
  /// @param max_itr Maximum number of repetitions
  /// @param goal_bias The ratio of aiming for goal [0.0-1.0]
  /// @param greedy Flag to keep extending FEASIBLE when aiming for goal
  /// @param is_terminate Forced termination conditions
  RrtPlanner(ConfigurationSpace::Ptr space,
             double delta,
             int32_t max_itr,
             double goal_bias,
             bool greedy,
             TerminateConditionFunc is_terminate) :
      space_(space), delta_(delta), max_itr_(max_itr),
      goal_bias_(goal_bias), greedy_(greedy), is_terminate_(is_terminate) {}
  /// @brief Pass the planner space and the termination conditions
  /// @param space Configuration space
  /// @param delta Exploration carved width
  /// @param max_itr Maximum number of repetitions
  /// @param goal_bias The ratio of aiming for goal [0.0-1.0]
  /// @param greedy Flag to keep extending FEASIBLE when aiming for goal
  RrtPlanner(ConfigurationSpace::Ptr space,
             double delta,
             int32_t max_itr,
             double goal_bias,
             bool greedy) :
      space_(space), delta_(delta), max_itr_(max_itr),
      goal_bias_(goal_bias), greedy_(greedy) {}

  virtual ~RrtPlanner() {}
  /// Creating paths
  virtual PlanRet PlanPath(const Config& init_config,
                           Path& path_out);

 private:
  // Copy prohibition
  RrtPlanner(const RrtPlanner&);
  RrtPlanner& operator=(const RrtPlanner&);
  bool BuildOneStep_(ConfigurationTree& tree);
  const ConfigurationSpace::Ptr space_;
  const double delta_;
  const int32_t max_itr_;
  const double goal_bias_;
  const bool greedy_;
  TerminateConditionFunc is_terminate_;
};
}  // namespace tmc_rplanner

#endif  // TMC_MANIPULATION_TMC_RPLANNER_RRT_PLANNER_HPP_
