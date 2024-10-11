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
/// @brief    random_optimizer

#include <stdlib.h>
#include <gtest/gtest.h>
#include <tmc_rplanner/configuration_space.hpp>
#include <tmc_rplanner/random_optimizer.hpp>

using tmc_rplanner::Config;
using tmc_rplanner::ConfigurationSpace;
using tmc_rplanner::IConfigOptimizer;
using tmc_rplanner::RandomOptimizer;

namespace {
// State space used in the test
int32_t kDim = 2;
// Tolerance for optimization
double kOptimThreshold = 1.0e-2;

static double Randd() {
  static uint32_t seed = 1;
  return static_cast<double>(rand_r(&seed))/RAND_MAX;
}

// Random function for testing
Config RandomConfig() {
  Config v(kDim);
  v(0) = Randd()*4.0;
  v(1) = Randd()*4.0;
  return v;
}

// Configuration check for testing
bool CheckConfig(const Config& config) {
  return true;
}

// Maximum functions in test configuration evaluation (2.0, 2.0)
double EvalConfig(const Config& config) {
  Config center(kDim);
  center(0) = 2.0;
  center(1) = 2.0;
  return exp(-pow((config-center).norm(), 2));
}

}  // anonymous namespace

// I don't know how to test it, but I hope the value is close to CENTER
TEST(RandomOptimizerCheck, optim_check) {
  ConfigurationSpace::Ptr cspace(new ConfigurationSpace(kDim));
  cspace->set_random_config(RandomConfig);
  cspace->set_evaluate_config(EvalConfig);
  cspace->set_check_feasibility(CheckConfig);
  IConfigOptimizer::Ptr optimizer(new RandomOptimizer(cspace, 100000, 100000));
  Config optim_config(kDim);
  double value;
  EXPECT_TRUE(optimizer->Optimize(optim_config, value));
  EXPECT_NEAR(2.0, optim_config(0), kOptimThreshold);
  EXPECT_NEAR(2.0, optim_config(1), kOptimThreshold);
  EXPECT_NEAR(1.0, value, kOptimThreshold);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
