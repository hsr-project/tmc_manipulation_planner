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
///  Planner library test
///  Moving and checking URDF models

#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include <tmc_manipulation_tests/configs.hpp>
#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_planner/robot_cbirrt_planner.hpp>
#include <tmc_robot_planner/task_space_region.hpp>

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::JointTrajectory;
using tmc_manipulation_types::MultiDOFJointTrajectory;
using tmc_manipulation_types::RobotTrajectory;
using tmc_manipulation_types::TaskSpaceRegion;
using tmc_manipulation_types::TaskSpaceRegionSeq;
using tmc_manipulation_types::AttachedObject;
using tmc_manipulation_types::AttachedObjectSeq;
using tmc_robot_kinematics_model::IRobotKinematicsModel;
using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_robot_kinematics_model::NumericIKSolver;
using tmc_robot_kinematics_model::PinocchioWrapper;
using tmc_robot_collision_detector::RobotCollisionDetector;
using tmc_robot_collision_detector::CuboidOverlapType;
using tmc_robot_collision_detector::CuboidOverlapGroupType;
using Eigen::Affine3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using tmc_manipulation_types::TaskSpaceRegion;
using tmc_manipulation_types::RegionValues;
using tmc_manipulation_types::Pose;
using tmc_manipulation_types::NameSeq;
using tmc_robot_planner::CalcDistanceToTsr;
using tmc_robot_planner::CalcClosestPose;
using tmc_robot_planner::GenerateSample;
using tmc_robot_planner::RobotCBiRrtPlanner;
using tmc_robot_planner::CBiRrtRequest;
using tmc_robot_planner::CBiRrtParameters;
using tmc_robot_planner::ErrorCode;
using tmc_robot_planner::IConfigurationConstraint;
using tmc_rplanner::Config;

namespace {
// Margin for floating point comparison
const double kDoubleEps = 1.0e-3;
// Hand name
const char* const kHandName = "link7";
// Numerical IK maximum number of repetitions
const int32_t kMaxItrIK = 1000;
// Numerical IK tolerance error
const double kIKDelta = 1.0e-3;
// Numerical IK tolerance fluctuation
const double kIKConvergeThreshold = 1.0e-10;
// Timeout [S]
const double kTimeOutPlanning = 120.0;
// Timeout [S]
const double kTimeOutPlanningVeryShort = 0.0001;
// Exploration
const double kDelta = 0.01;
// Interference check width
const double kSubDelta = 0.005;
// Maximum number of repetitions
const int32_t kMaxItrPlanning = 1000;
}  // anonymous namespace

class RobotCBiRrtPlannerUrdfTest :  public ::testing::Test {
 protected:
  RobotCBiRrtPlannerUrdfTest() {
    const std::string model_xml_string = tmc_manipulation_tests::stanford_manipulator::GetUrdf();
    const std::string collision_xml_string = tmc_manipulation_tests::stanford_manipulator::GetCollisionConfig();

    robot_ = std::make_shared<PinocchioWrapper>(model_xml_string);
    ik_solver_ = std::make_shared<NumericIKSolver>(IKSolver::Ptr(), robot_, kMaxItrIK, kIKDelta, kIKConvergeThreshold);
    robot_collision_detector_ = std::make_shared<RobotCollisionDetector>(
        model_xml_string, collision_xml_string, "ODE");
    planner_ = std::make_shared<RobotCBiRrtPlanner>(robot_, robot_collision_detector_, ik_solver_);

    NameSeq all_name(6);
    all_name[0] = "joint1";
    all_name[1] = "joint2";
    all_name[2] = "joint3";
    all_name[3] = "joint4";
    all_name[4] = "joint5";
    all_name[5] = "joint6";

    initial_config_.name = all_name;
    initial_config_.position.resize(6);
    initial_config_.position <<  0.5, 0.2, 0.4, 0.3, 0.6, 0.2;

    Eigen::Affine3d unit(Eigen::Affine3d::Identity());
    robot_collision_detector_->SetRobotTransform(unit);
    robot_collision_detector_->SetRobotNamedAngle(initial_config_);
  }

  // Check if the orbit is correct
  // *Continuous (width between Config is Delta or less)
  // *Not interfered
  // *If you have a Constraint_tsr, meet it
  // *The initial value is one of the following:
  //   1. What was set in Start_configs 2.Start_tsrs
  // *The terminal value is one of the following:
  //   1. Those set in Goal_configs 2.Goal_tsrs generated
  void CheckResultTrajectory(const RobotTrajectory& trajectory,
                             const CBiRrtRequest& req,
                             const CBiRrtParameters& params);

  IRobotKinematicsModel::Ptr robot_;
  RobotCollisionDetector::Ptr robot_collision_detector_;
  IKSolver::Ptr ik_solver_;
  RobotCBiRrtPlanner::Ptr planner_;
  JointState initial_config_;
};

void RobotCBiRrtPlannerUrdfTest::CheckResultTrajectory(
    const RobotTrajectory& robot_trajectory,
    const CBiRrtRequest& req,
    const CBiRrtParameters& params) {
  JointTrajectory trajectory = robot_trajectory.joint_trajectory;
  MultiDOFJointTrajectory base_trajectory =
      robot_trajectory.multi_dof_joint_trajectory;
  // Check if the track interval is beyond the delta
  for (uint32_t i = 0; i < trajectory.path.size(); ++i) {
    if (i != 0) {
      EXPECT_GE(params.delta + kDoubleEps,
                (trajectory.path[i] - trajectory.path[i-1]).norm());
    }

    std::vector<tmc_robot_collision_detector::PairString> contact_pair;
    JointState joint_state = {req.use_joints, trajectory.path[i]};
    robot_collision_detector_->SetRobotTransform(base_trajectory.path[i][0]);
    robot_collision_detector_->SetRobotNamedAngle(joint_state);
    robot_collision_detector_->RefleshOverlappedCuboids(
        tmc_robot_collision_detector::kOverlapAabb,
        tmc_robot_collision_detector::kOverlapGroup);
    bool feasible = !(robot_collision_detector_->CheckCollision(
        true, contact_pair));
    EXPECT_TRUE(feasible);

    // Constraint_tsr Check
    if (!req.constraint_tsrs.empty()) {
      JointState joint_state;
      joint_state.name = req.use_joints;
      joint_state.position = trajectory.path[i];
      // Set joint_angle, solve Kinematics in order, and check if it is within the range.
      robot_collision_detector_->SetRobotTransform(req.origin_to_basejoint);
      robot_collision_detector_->SetRobotNamedAngle(joint_state);
      Eigen::Affine3d origin_to_end = robot_collision_detector_->
          GetObjectTransform(req.constraint_tsrs[0].end_frame_id);

      // Confirm that it is fitted in TSR restraint
      double distance = (CalcDistanceToTsr(req.constraint_tsrs[0],
                                           origin_to_end)).norm();
      // IK is Angleaxis, TSR is an RPY expression, so it can be tolerated because there can be about 3 times the error.
      EXPECT_GE(kIKDelta * 3.0, distance);
    }
  }

  // Initial value check
  bool valid_start = false;
  // Global angle initial value
  for (std::vector<Config>::const_iterator config = req.start_configs.begin();
       config != req.start_configs.end();
       ++config) {
    if ((trajectory.path[0] - *config).norm() < kDoubleEps) {
      valid_start = true;
      break;
    }
  }
  for (TaskSpaceRegionSeq::const_iterator tsr = req.start_tsrs.begin();
       tsr != req.start_tsrs.end();
       ++tsr) {
    JointState joint_state;
    joint_state.name = req.use_joints;
    joint_state.position = trajectory.path[0];
    robot_collision_detector_->SetRobotTransform(req.origin_to_basejoint);
    robot_collision_detector_->SetRobotNamedAngle(joint_state);
    Eigen::Affine3d origin_to_end =
        robot_collision_detector_->GetObjectTransform(
            req.start_tsrs[0].end_frame_id);
    double distance = CalcDistanceToTsr(*tsr, origin_to_end).norm();
    if (distance < 1e-02) {
      valid_start = true;
      break;
    }
  }
  EXPECT_TRUE(valid_start);

  // End value check
  bool valid_goal = false;
  // Global angle initial value
  for (std::vector<Config>::const_iterator config = req.goal_configs.begin();
       config != req.goal_configs.end();
       ++config) {
    if ((trajectory.path.back() - *config).norm() < kDoubleEps) {
      valid_goal = true;
      break;
    }
  }
  for (TaskSpaceRegionSeq::const_iterator tsr = req.goal_tsrs.begin();
       tsr != req.goal_tsrs.end();
       ++tsr) {
    JointState joint_state;
    joint_state.name = req.use_joints;
    joint_state.position = trajectory.path.back();
    robot_collision_detector_->SetRobotTransform(base_trajectory.path.back()[0]);
    robot_collision_detector_->SetRobotNamedAngle(joint_state);
    Eigen::Affine3d origin_to_end = robot_collision_detector_->
        GetObjectTransform(req.goal_tsrs[0].end_frame_id);
    double distance = CalcDistanceToTsr(*tsr, origin_to_end).norm();
    if (distance <  1e-02) {
      valid_goal = true;
      break;
    }
  }
  EXPECT_TRUE(valid_goal);
}


// Testing as a simple BIRRT
TEST_F(RobotCBiRrtPlannerUrdfTest, plan_as_birrt) {
  NameSeq use_name(6);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(6);
  start_config << 0.5, 0.2, 0.4, 0.3, 0.6, 0.2;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(6);
  goal_config << 0.5, 0.2, 0.2, 0.1, 0.5, 1.0;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(6);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(6);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory, req, params);
  }
}


// Test as a Birlt including Base movement in the X -direction
TEST_F(RobotCBiRrtPlannerUrdfTest, plan_with_rail_x) {
  NameSeq use_name(6);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(6);
  start_config << 0.5, 0.2, 0.4, 0.3, 0.6, 0.2;
  Eigen::Affine3d start_basejoint_to_base = unit;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(6);
  goal_config << 0.5, 0.2, 0.2, 0.1, 0.5, 1.0;
  Eigen::Affine3d goal_basejoint_to_base =
      unit.translate(0.1 * Eigen::Vector3d::UnitX());

  CBiRrtRequest req;
  req.base_type = tmc_manipulation_types::kRailX;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.start_basejoint_to_bases.push_back(start_basejoint_to_base);
  req.goal_configs.push_back(goal_config);
  req.goal_basejoint_to_bases.push_back(goal_basejoint_to_base);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(7);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(7);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory, req, params);
  }
}

// Testing as a Birlt of hand -specified hand, including Planar's Base movement
TEST_F(RobotCBiRrtPlannerUrdfTest, plan_with_planar) {
  NameSeq use_name(6);
  use_name[0] = ("joint1");
  use_name[1] = ("joint2");
  use_name[2] = ("joint3");
  use_name[3] = ("joint4");
  use_name[4] = ("joint5");
  use_name[5] = ("joint6");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  std::string hand = "link7";

  Eigen::Affine3d origin_to_hand =
      robot_collision_detector_->GetObjectTransform(hand);

  // Completely fixed TSR
  RegionValues goal_min;
  RegionValues goal_max;
  goal_min << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  goal_max << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  Eigen::Affine3d origin_to_hand_goal = origin_to_hand *
      Eigen::Translation3d(0.5 * Eigen::Vector3d::UnitX());
  TaskSpaceRegion goal_tsr(
      origin_to_hand_goal,
      unit,  // tsr_to_end
      goal_min,  // min_bounds
      goal_max,  // max_bounds
      "origin",
      "link7");

  // Initial joint angle
  Config start_config;
  start_config.resize(6);
  start_config << 0.5, 0.2, 0.4, 0.3, 0.6, 0.2;
  Eigen::Affine3d start_basejoint_to_base = unit;

  CBiRrtRequest req;
  req.base_type = tmc_manipulation_types::kPlanar;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.start_basejoint_to_bases.push_back(start_basejoint_to_base);
  req.goal_tsrs.push_back(goal_tsr);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(9);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(9);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;
  params.probability_start_generate = 0.3;
  params.probability_goal_generate = 0.3;
  params.sampling_around_initial = true;
  params.sampling_distribution = 0.5;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory, req, params);
  }
}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
