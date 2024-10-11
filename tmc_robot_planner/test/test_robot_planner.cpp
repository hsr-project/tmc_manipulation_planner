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

#include <stdlib.h>

#include <chrono>
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
const char* const kHandName = "CARM/BASE_HAND";
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
const double kDelta = 0.1;
// Interference check width
const double kSubDelta = 0.01;
// Maximum number of repetitions
const int32_t kMaxItrPlanning = 1000;

// Contribution of the neck axis to the shoulder axis
class NeckShoulderConstraint : public IConfigurationConstraint {
 public:
  NeckShoulderConstraint() {}
  virtual ~NeckShoulderConstraint() {}
  virtual bool Constrain(
      const std::vector<std::string>& use_joints,
      const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
      const tmc_rplanner::Config& config_in,
      tmc_rplanner::Config& config_out) {
    uint32_t neck_yaw_index = 0;
    uint32_t shoulder_yaw_index = 0;
    std::vector<std::string>::const_iterator neck_it =
        std::find(use_joints.begin(), use_joints.end(), "CARM/HEAD/NECK_Y");
    if (neck_it == use_joints.end()) {
      return false;
    } else {
      neck_yaw_index = std::distance(use_joints.begin(), neck_it);
    }
    std::vector<std::string>::const_iterator shoulder_it =
        std::find(use_joints.begin(), use_joints.end(), "CARM/SHOULDER_Y");
    if (shoulder_it == use_joints.end()) {
      return false;
    } else {
      shoulder_yaw_index = std::distance(use_joints.begin(), shoulder_it);
    }
    config_out = config_in;
    config_out(neck_yaw_index) = - config_in(shoulder_yaw_index);
    return true;
  }
};
}  // anonymous namespace

// Interference check callback
void PrintCheckFeasibility(const Eigen::VectorXd& config, bool feasible,
                           const RobotCBiRrtPlanner::Ptr& robot) {
  if (feasible) {
  } else {
    std::vector<tmc_robot_collision_detector::PairString> contact_pair_out;
    contact_pair_out = robot->last_contact_pair();
  }
}

// Node additional callback
void PrintAddNode(const Eigen::VectorXd& src, const Eigen::VectorXd& dst) {
}


// Callback at restraint
void PrintConstrain(const Eigen::VectorXd& src,
                    const Eigen::VectorXd& dst,
                    bool success) {
}



class RobotCBiRrtPlannerTest :  public ::testing::Test {
 protected:
  RobotCBiRrtPlannerTest() {
    const std::string model_xml_string = tmc_manipulation_tests::hsra::GetUrdf();
    const std::string collision_xml_string = tmc_manipulation_tests::hsra::GetCollisionConfig();

    robot_ = std::make_shared<PinocchioWrapper>(model_xml_string);
    ik_solver_ = std::make_shared<NumericIKSolver>(IKSolver::Ptr(), robot_, kMaxItrIK, kIKDelta, kIKConvergeThreshold);
    robot_collision_detector_ = std::make_shared<RobotCollisionDetector>(
        model_xml_string, collision_xml_string, "ODE");
    planner_ = std::make_shared<RobotCBiRrtPlanner>(robot_, robot_collision_detector_, ik_solver_);

    NameSeq all_name(14);
    all_name[0] = "CARM/LINEAR";
    all_name[1] = "CARM/SHOULDER_Y";
    all_name[2] = "CARM/SHOULDER_R";
    all_name[3] = "CARM/SHOULDER_P";
    all_name[4] = "CARM/ELBOW_P";
    all_name[5] = "CARM/WRIST_Y";
    all_name[6] = "CARM/WRIST_R";
    all_name[7] = "CARM/WRIST_P";
    all_name[8] = "CARM/HEAD/NECK_Y";
    all_name[9] = "CARM/HEAD/NECK_P";
    all_name[10] = "CARM/HAND/JOINT_11";
    all_name[11] = "CARM/HAND/JOINT_12";
    all_name[12] = "CARM/HAND/JOINT_21";
    all_name[13] = "CARM/HAND/JOINT_22";
    initial_config_.name = all_name;
    initial_config_.position.resize(14);
    initial_config_.position << 0.3, 0.0, 0.0, 0.8, 1.57, 1.57, 0.0, -1.5,
        0.0, 0.0, 0.5, -0.5, 0.5, -0.5;

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
  void CheckResultTrajectory(const JointTrajectory& trajectory,
                             const CBiRrtRequest& req,
                             const CBiRrtParameters& params);

  IRobotKinematicsModel::Ptr robot_;
  RobotCollisionDetector::Ptr robot_collision_detector_;
  IKSolver::Ptr ik_solver_;
  RobotCBiRrtPlanner::Ptr planner_;
  JointState initial_config_;
};

void RobotCBiRrtPlannerTest::CheckResultTrajectory(
    const JointTrajectory& trajectory,
    const CBiRrtRequest& req,
    const CBiRrtParameters& params) {
  // Check if the track interval is beyond the delta
  for (uint32_t i = 0; i < trajectory.path.size(); ++i) {
    if (i != 0) {
      EXPECT_GE(params.delta + kDoubleEps,
                (trajectory.path[i] - trajectory.path[i-1]).norm());
    }

    std::vector<tmc_robot_collision_detector::PairString> contact_pair;
    JointState joint_state = {req.use_joints, trajectory.path[i]};
    robot_collision_detector_->SetRobotTransform(req.origin_to_basejoint);
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
      // IK is Angleaxis, TSR is an RPY expression, so it is acceptable because there can be up to 6 times an error.
      // Thinking about 3D positive and negative of rotation
      EXPECT_GE(kIKDelta * 6.0, distance);
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
    robot_collision_detector_->SetRobotTransform(req.origin_to_basejoint);
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
TEST_F(RobotCBiRrtPlannerTest, plan_as_birrt) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(8);
  goal_config << 0.45, 0.3, 0.2, 0.1, 2.0, 1.0, 0.0, -1.0;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

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
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }

  // The initial joint angle works slightly outside
  req.start_configs[0][0] = -1.0e-2 + 1.0e-3;
  EXPECT_EQ(tmc_robot_planner::kSuccess, planner_->PlanPath(req, params, trajectory));

  // If you exceed "slightly", the initial value error
  req.start_configs[0][0] = -1.0e-2 - 1.0e-3;
  EXPECT_EQ(tmc_robot_planner::kStartStateInCollision, planner_->PlanPath(req, params, trajectory));
}


// If Start and Goal have exactly the same posture
// Confirm that Path size is 2
TEST_F(RobotCBiRrtPlannerTest, plan_as_birrt_same) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Completely matched end angle
  Config goal_config;
  goal_config.resize(8);
  goal_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

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
    EXPECT_EQ(trajectory.joint_trajectory.path.size(), 2);
    EXPECT_TRUE(trajectory.joint_trajectory.path[0] == start_config);
    EXPECT_TRUE(trajectory.joint_trajectory.path[1] == goal_config);
  }
}


TEST_F(RobotCBiRrtPlannerTest, plan_as_birrt_without_shortcut) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(8);
  goal_config = start_config;
  goal_config[0] += 1.99 * kDelta;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = false;

  uint32_t max_size = 0;
  for (int i = 0; i < 10; ++i) {
    RobotTrajectory trajectory;
    ASSERT_EQ(tmc_robot_planner::kSuccess, planner_->PlanPath(req, params, trajectory));
    max_size = std::max<uint32_t>(max_size, trajectory.joint_trajectory.path.size());
  }
  // If you have a shortcut, you will always get 3 points
  EXPECT_LT(3, max_size);
}

TEST_F(RobotCBiRrtPlannerTest, plan_as_birrt_near) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Completely matched end angle
  Config goal_config;
  goal_config.resize(8);
  goal_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;
  goal_config += 0.001 * Config::Ones(8);

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

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
    EXPECT_EQ(trajectory.joint_trajectory.path.size(), 2);
    EXPECT_TRUE(trajectory.joint_trajectory.path[0] == start_config);
    EXPECT_TRUE(trajectory.joint_trajectory.path[1] == goal_config);
  }
}

// Test with restraint TSR
TEST_F(RobotCBiRrtPlannerTest, plan_with_constraint_tsr) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  JointState start_state;
  start_state.name = use_name;
  start_state.position = start_config;

  // The target value is the result of 0.05 [m] in the straight direction of X
  // Joint_angle set and solve Kinematics
  robot_collision_detector_->SetRobotTransform(unit);
  robot_collision_detector_->SetRobotNamedAngle(start_state);
  Eigen::Affine3d origin_to_hand_start = robot_collision_detector_->
      GetObjectTransform(kHandName);
  Eigen::Affine3d origin_to_hand_goal = origin_to_hand_start;
  origin_to_hand_goal.translation().x() += 0.1;

  tmc_robot_kinematics_model::IKRequest ikreq;
  ikreq.frame_name = kHandName;
  ikreq.frame_to_end = unit;
  ikreq.ref_origin_to_end = origin_to_hand_goal;
  ikreq.origin_to_base = unit;
  ikreq.initial_angle.name = use_name;
  ikreq.initial_angle.position = start_config;
  ikreq.use_joints = use_name;


  Eigen::Affine3d origin_to_hand_solved;
  JointState goal_state;
  ASSERT_EQ(tmc_robot_kinematics_model::kSuccess,
            ik_solver_->Solve(ikreq, goal_state, origin_to_hand_solved));

  // Restraint in the straight line of X
  RegionValues min;
  RegionValues max;
  min << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  max << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  TaskSpaceRegion constraint_tsr(
      unit,  // origin_to_tsr
      origin_to_hand_start,  // tsr_to_end
      min,  // min_bounds
      max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_state.position);
  req.constraint_tsrs.push_back(constraint_tsr);
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

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
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }
}


// Test with restraint TSR
TEST_F(RobotCBiRrtPlannerTest, plan_with_abnormal_constraint_tsr) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  JointState start_state;
  start_state.name = use_name;
  start_state.position = start_config;

  // The target value is the result of 0.05 [m] in the straight direction of X
  // Joint_angle set and solve Kinematics
  robot_collision_detector_->SetRobotTransform(unit);
  robot_collision_detector_->SetRobotNamedAngle(start_state);
  Eigen::Affine3d origin_to_hand_start = robot_collision_detector_->
      GetObjectTransform(kHandName);
  Eigen::Affine3d origin_to_hand_goal = origin_to_hand_start;
  origin_to_hand_goal.translation().x() += 0.1;

  tmc_robot_kinematics_model::IKRequest ikreq;
  ikreq.frame_name = kHandName;
  ikreq.frame_to_end = unit;
  ikreq.ref_origin_to_end = origin_to_hand_goal;
  ikreq.origin_to_base = unit;
  ikreq.initial_angle.name = use_name;
  ikreq.initial_angle.position = start_config;
  ikreq.use_joints = use_name;


  Eigen::Affine3d origin_to_hand_solved;
  JointState goal_state;
  ASSERT_EQ(tmc_robot_kinematics_model::kSuccess,
            ik_solver_->Solve(ikreq, goal_state, origin_to_hand_solved));

  // Restraint with +0.01 [m] to -0.01 [m] in the straight direction of X
  RegionValues min;
  RegionValues max;
  min << -0.01, 0.0, 0.0, 0.0, 0.0, 0.0;
  max << 0.01, 0.0, 0.0, 0.0, 0.0, 0.0;

  TaskSpaceRegion constraint_tsr(
      unit,  // origin_to_tsr
      origin_to_hand_start,  // tsr_to_end
      min,  // min_bounds
      max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_state.position);
  req.constraint_tsrs.push_back(constraint_tsr);
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kGoalStateInCollision, result);

  // Restraint with +0.04 [m] to 0.06 [m] in the straight direction of X
  req.constraint_tsrs[0].min_bounds[0] = 0.04;
  req.constraint_tsrs[0].max_bounds[0] = 0.06;

  result = planner_->PlanPath(req, params, trajectory);
  EXPECT_EQ(tmc_robot_planner::kStartStateInCollision, result);
}

// Plan with TSR start and goal
TEST_F(RobotCBiRrtPlannerTest, plan_with_goal_start_tsr) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  Eigen::Affine3d origin_to_hand_start =
      robot_collision_detector_->GetObjectTransform(kHandName);

  // OK to move on the pitch
  RegionValues start_min;
  RegionValues start_max;
  start_min << 0.0, 0.0, 0.0, -M_PI, 0.0, 0.0;
  start_max << 0.0, 0.0, 0.0, M_PI, 0.0, 0.0;

  TaskSpaceRegion start_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      start_min,  // min_bounds
      start_max,  // max_bounds
      "origin",
      kHandName);

  RegionValues goal_min;
  RegionValues goal_max;
  goal_min << 0.0, 0.1, 0.0, -M_PI, 0.0, 0.0;
  goal_max << 0.0, 0.1, 0.0, M_PI, 0.0, 0.0;

  TaskSpaceRegion goal_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      goal_min,  // min_bounds
      goal_max,  // max_bounds
      "origin",
      kHandName);

  RegionValues const_min;
  RegionValues const_max;
  const_min << -1.0, -1.0, -1.0, -M_PI, 0.0, 0.0;
  const_max << 1.0, 1.0, 1.0, M_PI, 0.0, 0.0;

  TaskSpaceRegion constraint_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      const_min,  // min_bounds
      const_max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.constraint_tsrs.push_back(constraint_tsr);
  req.start_tsrs.push_back(start_tsr);
  req.goal_tsrs.push_back(goal_tsr);
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;
  params.probability_start_generate = 0.3;
  params.probability_goal_generate = 0.3;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }
}


// Plan with TSR start and goal
TEST_F(RobotCBiRrtPlannerTest, plan_with_goal_start_tsr_init_sampling) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // OK to move on the pitch
  RegionValues start_min;
  RegionValues start_max;
  start_min << 0.0, 0.0, 0.0, -M_PI, 0.0, 0.0;
  start_max << 0.0, 0.0, 0.0, M_PI, 0.0, 0.0;

  Eigen::Affine3d origin_to_hand_start =
      robot_collision_detector_->GetObjectTransform(kHandName);

  TaskSpaceRegion start_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      start_min,  // min_bounds
      start_max,  // max_bounds
      "origin",
      kHandName);

  RegionValues goal_min;
  RegionValues goal_max;
  goal_min << 0.0, 0.1, 0.0, -M_PI, 0.0, 0.0;
  goal_max << 0.0, 0.1, 0.0, M_PI, 0.0, 0.0;

  TaskSpaceRegion goal_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      goal_min,  // min_bounds
      goal_max,  // max_bounds
      "origin",
      kHandName);

  RegionValues const_min;
  RegionValues const_max;
  const_min << -1.0, -1.0, -1.0, -M_PI, 0.0, 0.0;
  const_max << 1.0, 1.0, 1.0, M_PI, 0.0, 0.0;

  TaskSpaceRegion constraint_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      const_min,  // min_bounds
      const_max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.constraint_tsrs.push_back(constraint_tsr);
  req.start_tsrs.push_back(start_tsr);
  req.goal_tsrs.push_back(goal_tsr);
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

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
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }
}

// Check if the IK exclusion joint angle works correctly
TEST_F(RobotCBiRrtPlannerTest, plan_with_no_ik_joint) {
  NameSeq use_name(9);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");
  // Joints that are not related to IK
  use_name[8] = ("CARM/HEAD/NECK_Y");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  JointState goal_no_ik_joint_state;
  goal_no_ik_joint_state.name.push_back("CARM/HEAD/NECK_Y");
  goal_no_ik_joint_state.position.resize(1);
  goal_no_ik_joint_state.position << 0.5;

  // Initial joint angle
  Config start_config;
  start_config.resize(9);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5, 0.0;

  RegionValues goal_min;
  RegionValues goal_max;
  goal_min << 0.0, 0.1, 0.0, -M_PI, 0.0, 0.0;
  goal_max << 0.0, 0.1, 0.0, M_PI, 0.0, 0.0;

  JointState start_state;
  start_state.name = use_name;
  start_state.position = start_config;

  robot_collision_detector_->SetRobotNamedAngle(start_state);

  Eigen::Affine3d origin_to_hand_start =
      robot_collision_detector_->GetObjectTransform(kHandName);
  TaskSpaceRegion goal_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      goal_min,  // min_bounds
      goal_max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_tsrs.push_back(goal_tsr);
  req.goal_no_ik_joint_state = goal_no_ik_joint_state;

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
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }
  // Confirm that the terminal value of Neck_y is 0.5
  EXPECT_NEAR(trajectory.joint_trajectory.path.back()(8), 0.5, 1e-5);
}

// Check if EXTRA restraint works properly
TEST_F(RobotCBiRrtPlannerTest, plan_with_extra_constraint) {
  NameSeq use_name(9);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");
  // EXTEND joint
  use_name[8] = ("CARM/HEAD/NECK_Y");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(9);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5, 0.0;

  RegionValues goal_min;
  RegionValues goal_max;
  goal_min << 0.0, 0.1, 0.0, -M_PI, 0.0, 0.0;
  goal_max << 0.0, 0.1, 0.0, M_PI, 0.0, 0.0;

  JointState start_state;
  start_state.name = use_name;
  start_state.position = start_config;

  robot_collision_detector_->SetRobotNamedAngle(start_state);

  Eigen::Affine3d origin_to_hand_start =
      robot_collision_detector_->GetObjectTransform(kHandName);

  IConfigurationConstraint::Ptr simple_const(new NeckShoulderConstraint());


  TaskSpaceRegion goal_tsr(
      origin_to_hand_start,  // origin_to_tsr
      unit,  // tsr_to_end
      goal_min,  // min_bounds
      goal_max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_tsrs.push_back(goal_tsr);
  req.extra_start_constraints.push_back(simple_const);
  req.extra_goal_constraints.push_back(simple_const);
  req.extra_constraints.push_back(simple_const);

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
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }

  for (size_t i = 0; i < trajectory.joint_trajectory.path.size(); ++i) {
    // Confirm that Neck_y is inverted in Shoulder_y
    EXPECT_NEAR(trajectory.joint_trajectory.path[i](8),
                -trajectory.joint_trajectory.path[i](1), 1e-5);
  }
}


// Simple BIRRT timeout
TEST_F(RobotCBiRrtPlannerTest, plan_as_birrt_timeout) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(8);
  goal_config << 0.45, 0.2, 0.2, 0.1, 2.0, 1.0, 0.0, -1.0;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanningVeryShort;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kTimedOut, result);
}


// Test with restraint TSR
TEST_F(RobotCBiRrtPlannerTest, plan_with_constraint_tsr_timeout) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  JointState start_state;
  start_state.name = use_name;
  start_state.position = start_config;

  // The target value is the result of 0.05 [m] in the straight direction of X
  // Joint_angle set and solve Kinematics
  robot_collision_detector_->SetRobotTransform(unit);
  robot_collision_detector_->SetRobotNamedAngle(start_state);

  Eigen::Affine3d origin_to_hand_start =
      robot_collision_detector_->GetObjectTransform(kHandName);
  Eigen::Affine3d origin_to_hand_goal = origin_to_hand_start;
  origin_to_hand_goal.translation().x() += 0.1;

  tmc_robot_kinematics_model::IKRequest ikreq;
  ikreq.frame_name = kHandName;
  ikreq.frame_to_end = unit;
  ikreq.ref_origin_to_end = origin_to_hand_goal;
  ikreq.origin_to_base = unit;
  ikreq.initial_angle.name = use_name;
  ikreq.initial_angle.position = start_config;
  ikreq.use_joints = use_name;


  Eigen::Affine3d origin_to_hand_solved;
  JointState goal_state;
  ASSERT_EQ(tmc_robot_kinematics_model::kSuccess,
            ik_solver_->Solve(ikreq, goal_state, origin_to_hand_solved));

  // Restraint in the straight line of X
  RegionValues min;
  RegionValues max;
  min << -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  max << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  TaskSpaceRegion constraint_tsr(
      unit,  // origin_to_tsr
      origin_to_hand_start,  // tsr_to_end
      min,  // min_bounds
      max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_state.position);
  req.constraint_tsrs.push_back(constraint_tsr);
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanningVeryShort;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kTimedOut, result);
}


// Confirm that time out in a reasonable time
TEST_F(RobotCBiRrtPlannerTest, plan_timeout_normal) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(8);
  goal_config << 0.45, 0.2, 0.2, 0.1, 2.0, 1.0, 0.0, -1.0;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanningVeryShort;
  // Timeout with small Delta
  params.delta = kDelta * 1.0e-10;
  params.sub_delta = kSubDelta * 1.0e-10;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;
  RobotTrajectory trajectory;
  auto t1 = std::chrono::system_clock::now();
  ErrorCode result = planner_->PlanPath(req, params, trajectory);
  auto t2 = std::chrono::system_clock::now();
  EXPECT_TRUE((result == tmc_robot_planner::kTimedOut) || (result == tmc_robot_planner::kShortcutTimedOut));
  // Timeout error within 0.5 [s]
  EXPECT_NEAR(std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count(), 500, 500);
}

// Check if the joints there is small enough when weighted
TEST_F(RobotCBiRrtPlannerTest, weight_check) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.0, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  // Terminal joint angle
  Config goal_config;
  goal_config.resize(8);
  goal_config << 0.45, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  CBiRrtRequest req;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_configs.push_back(goal_config);
  req.initial_config = initial_config_;
  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 10.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = 0.05;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }

  // In search of the average DELTA of joint 0
  double sum_delta1 = 0.0;
  for (uint32_t i = 0; i < trajectory.joint_trajectory.path.size()-1; ++i) {
    double delta = (trajectory.joint_trajectory.path[i+1](0) -
                    trajectory.joint_trajectory.path[i](0));
    sum_delta1 += delta;
  }
  double mean_delta1 = sum_delta1 / (trajectory.joint_trajectory.path.size()-1.0);

  // Joint weight
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // Joint weight
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }

  // In search of the average DELTA of joint 0
  double sum_delta2 = 0.0;
  for (uint32_t i = 0; i < trajectory.joint_trajectory.path.size()-1; ++i) {
    double delta = (trajectory.joint_trajectory.path[i+1](0) -
                    trajectory.joint_trajectory.path[i](0));
    sum_delta2 += delta;
  }
  double mean_delta2 = sum_delta2 / (trajectory.joint_trajectory.path.size() - 1.0);
  EXPECT_NEAR(mean_delta2 / mean_delta1, 10.0, 1.0);
}


// Test with restraint TSR
TEST_F(RobotCBiRrtPlannerTest, plan_with_constraint_tsr_weighted_ik) {
  NameSeq use_name(8);
  use_name[0] = ("CARM/LINEAR");
  use_name[1] = ("CARM/SHOULDER_Y");
  use_name[2] = ("CARM/SHOULDER_R");
  use_name[3] = ("CARM/SHOULDER_P");
  use_name[4] = ("CARM/ELBOW_P");
  use_name[5] = ("CARM/WRIST_Y");
  use_name[6] = ("CARM/WRIST_R");
  use_name[7] = ("CARM/WRIST_P");

  Eigen::Affine3d unit(Eigen::Affine3d::Identity());

  // Initial joint angle
  Config start_config;
  start_config.resize(8);
  start_config << 0.3, 0.0, 0.0, 0.1, 3.0, 1.57, 0.0, -1.5;

  JointState start_state;
  start_state.name = use_name;
  start_state.position = start_config;


  // The target value is the result of 0.1 [m] in the straight direction of x and z
  // Joint_angle set and solve Kinematics
  robot_collision_detector_->SetRobotTransform(unit);
  robot_collision_detector_->SetRobotNamedAngle(start_state);
  Eigen::Affine3d origin_to_hand_start
      = robot_collision_detector_->GetObjectTransform(kHandName);
  Eigen::Affine3d origin_to_hand_goal = origin_to_hand_start;
  origin_to_hand_goal.translation().x() += 0.1;
  origin_to_hand_goal.translation().z() += 0.1;

  RegionValues goal_min;
  RegionValues goal_max;
  goal_min << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  goal_max << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  TaskSpaceRegion goal_tsr(
      origin_to_hand_goal,  // origin_to_tsr
      unit,  // tsr_to_end
      goal_min,  // min_bounds
      goal_max,  // max_bounds
      "origin",
      kHandName);

  CBiRrtRequest req;
  req.initial_config = initial_config_;
  req.use_joints = use_name;
  req.origin_to_basejoint = unit;
  req.start_configs.push_back(start_config);
  req.goal_tsrs.push_back(goal_tsr);
  req.weight_config.resize(8);
  req.weight_config << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  req.weight_config_ik.resize(8);
  req.weight_config_ik << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  CBiRrtParameters params;
  params.timeout = kTimeOutPlanning;
  params.delta = kDelta;
  params.sub_delta = kSubDelta;
  params.max_itr = kMaxItrPlanning;
  params.do_shortcut = true;
  params.probability_goal_generate = 0.3;
  params.sampling_around_initial = true;
  params.sampling_distribution = 0.01;
  params.increase_sampling_deviation = true;

  RobotTrajectory trajectory;
  ErrorCode result = planner_->PlanPath(req, params, trajectory);

  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }

  double last_linear_no_weight = trajectory.joint_trajectory.path.back()[0];
  req.weight_config_ik << 1000.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  result = planner_->PlanPath(req, params, trajectory);
  EXPECT_EQ(tmc_robot_planner::kSuccess, result);
  if (result == tmc_robot_planner::kSuccess) {
    CheckResultTrajectory(trajectory.joint_trajectory, req, params);
  }
  double last_linear_weight = trajectory.joint_trajectory.path.back()[0];
  EXPECT_GT(fabs(last_linear_no_weight - start_config(0)),
            fabs(last_linear_weight - start_config(0)));
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
