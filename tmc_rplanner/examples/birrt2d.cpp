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
#include <fstream>
#include <iostream>

#include <tmc_rplanner/birrt_planner.hpp>
#include <tmc_rplanner/path_short_cutter.hpp>
#include <tmc_rplanner/point_to_point_planner.hpp>
#include <tmc_rplanner/raund_robin_short_cutter.hpp>

using tmc_rplanner::BiRrtPlanner;
using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::IPathShortCutter;
using tmc_rplanner::IPointToPointPlanner;
using tmc_rplanner::kSuccess;
using tmc_rplanner::Path;
using tmc_rplanner::RoundRobinShortCutter;

const int32_t kDim(2);
const double kDelta(0.1);

static double Randd() {
  uint32_t seed(0);
  return static_cast<double>(rand_r(&seed))/RAND_MAX;
}

Config RandomConfig() {
  Config v(2);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

bool CollisionCheck(const Config& config) {
  if (((config(0) < 3.5) && (config(0) > 0)) && ((config(1) < 1.5) && (config(1) > 1.0))) {
    return false;
  }
  if (((config(0) < 4.0) && (config(0) > 0.5)) && ((config(1) < 3.5) && (config(1) > 3.0))) {
    return false;
  }
    return true;
}

int main(int argc, char* argv[]) {
  Config init(kDim);
  init << 0.0, 0.0;
  Config goal(kDim);
  goal << 4.0, 4.0;

  ConfigurationSpace::Ptr space(new ConfigurationSpace(kDim));
  space->set_random_config(RandomConfig);
  space->set_check_feasibility(CollisionCheck);

  IPointToPointPlanner::Ptr planner(new BiRrtPlanner(space, kDelta, 10000));

  Path path;
  Path opt_path;
  double length = 0.0;
  std::ofstream os_path("path.dat");
  std::ofstream os_opt_path("optim.dat");
  if (planner->PlanPath(init, goal, path) == kSuccess) {
    IPathShortCutter::Ptr short_cutter(new RoundRobinShortCutter(space, kDelta, true, 1));
    short_cutter->ShortCut(path, opt_path);
    for (size_t i = 0; i < path.size()-1; ++i) {
      os_path << path[i].transpose() << " " << path[i+1].transpose() << std::endl;
       length += space->CalcDistance(path[i], path[i+1]);
     }
    for (size_t i = 0; i < opt_path.size()-1; ++i) {
      os_opt_path << opt_path[i].transpose() << " " << opt_path[i+1].transpose() << std::endl;
      length += space->CalcDistance(opt_path[i], opt_path[i+1]);
    }
  }

  return 0;
}

