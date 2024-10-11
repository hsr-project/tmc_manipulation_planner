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
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <tmc_rplanner/path_short_cutter.hpp>
#include <tmc_rplanner/point_to_condition_planner.hpp>
#include <tmc_rplanner/raund_robin_short_cutter.hpp>
#include <tmc_rplanner/rrt_planner.hpp>

using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::IPathShortCutter;
using tmc_rplanner::IPointToConditionPlanner;
using tmc_rplanner::kSuccess;
using tmc_rplanner::Path;
using tmc_rplanner::RoundRobinShortCutter;
using tmc_rplanner::RrtPlanner;

const int32_t kDim(2);
const double kDelta(0.02);

static double Randd() {
  uint32_t seed(0);
  return static_cast<double>(rand_r(&seed))/RAND_MAX;
}

Eigen::VectorXd RandomConfig() {
  Eigen::VectorXd v(2);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

bool FeasibilityCheck(const Config& config) {
  // [0.5 3.5], [0.5 1.0].
  if (((config(0) > 0.5) && (config(0) < 3.5)) && ((config(1) > 0.5) && (config(1) < 1.0))) {
    return false;
  }
  // [1.5 2.5], [1.0 4.0] locking body
  if (((config(0) > 1.5) && (config(0) < 2.5)) && ((config(1) > 1.0) && (config(1) < 4.0))) {
    return false;
  }
    return true;
}

// Appropriately on the right and left
bool GenerateGoal(Config v) {
  v.resize(2);
  if (Randd() < 0.5) {
    v(0) = 1.0;
    v(1) = 3.5;
  } else {
    v(0) = 3.0;
    v(1) = 3.5;
  }
  return true;
}

bool IsGoal(const Config& config) {
  // (0.9 1.1), (3.4 3.6), (3.4 3.6)
  if (((config(0) > 0.9) && (config(0) < 1.1)) && ((config(1) > 3.4) && (config(1) < 3.6))) {
    return true;
  }
  // (2.9 3.1), (3.4 3.6), (3.4 3.6)
  if (((config(0) > 2.9) && (config(0) < 3.1)) && ((config(1) > 3.4) && (config(1) < 3.6))) {
    return true;
  }
    return false;
}

int main(int argc, char* argv[]) {
    Config init(kDim);
    init << 2.0, 0.0;
    Config goal(kDim);
    goal << 4.0, 4.0;

    ConfigurationSpace::Ptr space(new ConfigurationSpace(kDim));
    space->set_random_config(RandomConfig);
    space->set_check_feasibility(FeasibilityCheck);
    space->set_generate_goal_config(GenerateGoal);
    space->set_check_goal(IsGoal);

    IPointToConditionPlanner::Ptr planner(new RrtPlanner(space, kDelta, 10000, 0.2, false));

    Path path;
    Path opt_path;
    double length = 0.0;
    if (planner->PlanPath(init, path) == kSuccess) {
      IPathShortCutter::Ptr short_cutter(new RoundRobinShortCutter(space, kDelta, true, 1));
      short_cutter->ShortCut(path, opt_path);
      // for (size_t i = 0; i < path.size()-1; ++i) {
      //   std::cout << path[i].transpose() << " " << path[i+1].transpose() << std::endl;
      //   length += space->CalcDistance(path[i], path[i+1]);
      // }
      // //space->ShortCutBiDirectional(1,path,opt_path);
      for (size_t i = 0; i < opt_path.size()-1; ++i) {
        std::cout << opt_path[i].transpose() << " " << opt_path[i+1].transpose() << std::endl;
        length += space->CalcDistance(opt_path[i], opt_path[i+1]);
      }
    }
    //    std::cerr << "path_length = " << length << std::endl;
    return 0;
}
