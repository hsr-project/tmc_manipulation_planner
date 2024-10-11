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

#include <gtest/gtest.h>

#include <tmc_eigen_utils/eigen_utils.hpp>
#include <tmc_robot_planner/task_space_region.hpp>

using Eigen::Affine3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using tmc_manipulation_types::TaskSpaceRegion;
using tmc_manipulation_types::RegionValues;
using tmc_manipulation_types::Pose;
using tmc_robot_planner::CalcDistanceToTsr;
using tmc_robot_planner::CalcClosestPose;
using tmc_robot_planner::GenerateSample;

namespace {
/// A threshold that is regarded as the value of DOUBLE is near
double kDoubleEps = 1.0e-10;
}

// Check if the correct distance can be achieved in TSR only in parallel
TEST(DistanceTest, pos) {
  RegionValues min;
  RegionValues max;
  min << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  max << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  TaskSpaceRegion tsr(Pose(Translation3d(0.0, 0.0, 0.0)),  // origin_to_tsr
                      Pose(Translation3d(0.0, 0.0, 0.0)),  // tsr_to_end
                      min,  // min_bounds
                      max,  // max_bounds
                      "origin",
                      "end");
  RegionValues distance1 = CalcDistanceToTsr(tsr, Pose(Translation3d(0.5, 0.0, 0.0)));
  EXPECT_NEAR(distance1(0), 0.0, kDoubleEps);
  // std::cerr << "distance = " << distance1(0) << std::endl;

  RegionValues distance2 = CalcDistanceToTsr(tsr, Pose(Translation3d(1.5, 0.0, 0.0)));
  EXPECT_NEAR(distance2(0), 0.5, kDoubleEps);
  // std::cerr << "distance = " << distance2(0) << std::endl;
}

// Check if you can sampled it for TSR only in parallel
TEST(SamplingTest, pos) {
  RegionValues min;
  RegionValues max;
  min << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  max << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  TaskSpaceRegion tsr(Pose(Translation3d(0.0, 0.0, 0.0)),  // origin_to_tsr
                      Pose(Translation3d(0.0, 0.0, 0.0)),  // tsr_to_end
                      min,  // min_bounds
                      max,  // max_bounds
                      "origin",
                      "end");
  Pose sample;
  double norm;
  for (int32_t i = 0; i < 100; ++i) {
    sample = GenerateSample(tsr);
    norm = CalcDistanceToTsr(tsr, sample).norm();
    EXPECT_NEAR(norm, 0.0, kDoubleEps);
  }
}

// Check if you can sample the TSR of rotation
TEST(SamplingTest, rot) {
  RegionValues min;
  RegionValues max;
  min << 0.0, 0.0, 0.0, -1.0, -1.0, 0.0;
  max << 0.0, 0.0, 0.0, 1.0, 1.0, 0.0;
  TaskSpaceRegion tsr(Pose(Translation3d(0.0, 0.0, 0.0)),  // origin_to_tsr
                      Pose(Translation3d(0.0, 0.0, 0.0)),  // tsr_to_end
                      min,  // min_bounds
                      max,  // max_bounds
                      "origin",
                      "end");
  Pose sample;
  double norm;
  for (int32_t i = 0; i < 100; ++i) {
    sample = GenerateSample(tsr);
    norm = CalcDistanceToTsr(tsr, sample).norm();
    EXPECT_NEAR(norm, 0.0, kDoubleEps);
    // std::cerr << "sample.pos = \n" << sample.translation() << std::endl;
    // std::cerr << "sample.rot = \n" << tmc_eigen_utils::QuaternionToRPY(Quaterniond(sample.linear())) << std::endl;
  }
}

// Check whether the near -point can be calculated recently for the rotation TSR
TEST(ClosestTest, rot) {
  RegionValues min;
  RegionValues max;
  min << 0.0, 0.0, 0.0, -0.1, -0.5, 0.0;
  max << 0.0, 0.0, 0.0, 0.1, 0.5, 0.0;
  TaskSpaceRegion tsr(Pose(Translation3d(0.0, 0.0, 0.0)),  // origin_to_tsr
                      Pose(Translation3d(0.0, 0.0, 0.0)),  // tsr_to_end
                      min,  // min_bounds
                      max,  // max_bounds
                      "origin",
                      "end");
  Pose sample;
  Pose closest;
  double norm;
  for (int32_t i = 0; i < 100; ++i) {
    Vector3d rand = Vector3d::Random();
    sample = Pose(tmc_eigen_utils::RPYToQuaternion(rand));
    closest = CalcClosestPose(tsr, sample);
    norm = CalcDistanceToTsr(tsr, closest).norm();
    EXPECT_NEAR(norm, 0.0, kDoubleEps);
  }
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
