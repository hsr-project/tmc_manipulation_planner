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
/// @brief    Planna using ROBOT CBirrt

#ifndef TMC_ROBOT_PLANNER_ROBOT_CBIRRT_PLANNER_HPP_
#define TMC_ROBOT_PLANNER_ROBOT_CBIRRT_PLANNER_HPP_

#include <stdint.h>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <tmc_robot_collision_detector/robot_collision_detector.hpp>

#include <boost/optional.hpp>  // NOLINT
#include <boost/random/ranlux.hpp>  // NOLINT
#include <boost/random/uniform_int.hpp>  // NOLINT
#include <boost/random/uniform_real.hpp>  // NOLINT

#include <tmc_manipulation_types/manipulation_types.hpp>
#include <tmc_robot_kinematics_model/ik_solver.hpp>
#include <tmc_robot_kinematics_model/robot_kinematics_model.hpp>
#include <tmc_robot_planner/configuration_constraint.hpp>
#include <tmc_robot_planner/robot_planner_common.hpp>
#include <tmc_robot_planner/task_space_region.hpp>

namespace tmc_robot_planner {

const double kDefaultDelta = 0.1;
const double kDefaultSubDelta = 0.05;
const double kDefaultMaxItr = 100;
const double kDefaultTimeOut = 10.0;
const double kDefaultSamplingDistribution = 0.1;
const double kDefaultStepSampling = 0.1;
/// Limit of the bogie parallel [M]
const double kDefaultBaseTranslationMax = 10.0;

/// Requests given to CBIRRT
struct CBiRrtRequest {
  CBiRrtRequest() :
      use_joints(0),
      base_type(tmc_manipulation_types::kNone),
      start_configs(0),
      start_basejoint_to_bases(0),
      goal_configs(0),
      goal_basejoint_to_bases(0),
      start_tsrs(0),
      constraint_tsrs(0),
      goal_tsrs(0),
      start_no_ik_joint_state(),
      goal_no_ik_joint_state(),
      origin_to_basejoint(Eigen::Affine3d::Identity()),
      known_objects(0),
      collision_map(0),
      attached_objects(0),
      weight_config(),
      weight_config_ik(),
      weight_linear_base(1.0),
      weight_rotational_base(1.0),
      weight_linear_base_ik(1.0),
      weight_rotational_base_ik(1.0),
      extra_constraints(0),
      extra_start_constraints(0),
      extra_goal_constraints(0) {}
  /// List of joint name used.Only the joints listed here are subject to search.
  std::vector<std::string> use_joints;
  /// Base operation New in version 0.16.0
  tmc_manipulation_types::BaseMovementType base_type;
  /// The initial value of all joints is specified in USE_Joints.
  tmc_manipulation_types::JointState initial_config;
  /// A set of initial joint angles.Specified in the configuration space.
  std::vector<Config> start_configs;
  /// If the initial value of Base Base_movement is not KNONE, specify specified New in version 0.16.0
  tmc_manipulation_types::PoseSeq start_basejoint_to_bases;
  /// A set of terminal joint angles.Specified in the configuration space.
  std::vector<Config> goal_configs;
  /// Base terminal value Base_movement specified New in version 0.16.0 if other than KNONE
  tmc_manipulation_types::PoseSeq goal_basejoint_to_bases;
  /// A set of initial TSR.Sampling from here to add the initial value.
  tmc_manipulation_types::TaskSpaceRegionSeq start_tsrs;
  /// A set of restraint TSR.All trajectory is restrained by this
  tmc_manipulation_types::TaskSpaceRegionSeq constraint_tsrs;
  /// A set of terminal TSR.Sampling from here to add the initial value.
  tmc_manipulation_types::TaskSpaceRegionSeq goal_tsrs;
  /// Specify the target value of the joints that are not affected by IK when Start is specified in TSR
  tmc_manipulation_types::JointState start_no_ik_joint_state;
  /// Specify the target value of the joints that are not affected by IK when Start is specified in TSR
  tmc_manipulation_types::JointState goal_no_ik_joint_state;
  /// Robot reference position (this is the standard for BASE except for Knone)
  Eigen::Affine3d origin_to_basejoint;
  /// Interference checks to be checked
  tmc_manipulation_types::OuterObjectParametersSeq known_objects;
  /// Environment to check interference
  tmc_manipulation_types::CuboidSeq collision_map;
  /// A set of things that the robot has
  tmc_manipulation_types::AttachedObjectSeq attached_objects;
  /// A joint that does not want to move as much as the weight of each joint is normal 1
  /// Is a large value 2 or a large value
  Config weight_config;
  /// Heavy value of each joint for IK Normally 1 and not wanting to move too much
  /// Is 100 or large value
  Config weight_config_ik;
  /// It should be about 0.1 in the straight line direction about Base
  double weight_linear_base;
  /// It is good to set the weight of the rotation direction about 0.1
  double weight_rotational_base;
  /// It is good to make the IK weight about 0.1 in the linear direction related to BASE
  double weight_linear_base_ik;
  /// It is good to make the IK weight about 0.1 in the direction of rotation related to BASE
  double weight_rotational_base_ik;
  /// Restrained from the 0th element after restraint TSR in the entire track
  std::vector<IConfigurationConstraint::Ptr> extra_constraints;
  /// Restrained from the 0th element after Start restraint TSR.
  std::vector<IConfigurationConstraint::Ptr> extra_start_constraints;
  /// Restrained from the 0th element after the restraint of the Goal restraint TSR
  std::vector<IConfigurationConstraint::Ptr> extra_goal_constraints;
  // For Eigen fixed length members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Planning parameter
struct CBiRrtParameters {
  CBiRrtParameters() : delta(kDefaultDelta), sub_delta(kDefaultSubDelta),
                       probability_start_generate(0.0),
                       probability_goal_generate(0.0),
                       max_itr(kDefaultMaxItr), timeout(kDefaultTimeOut),
                       do_shortcut(true), sampling_around_initial(false),
                       sampling_distribution(kDefaultSamplingDistribution),
                       increase_sampling_deviation(true),
                       step_sampling_deviation(kDefaultStepSampling),
                       base_translation_max(kDefaultBaseTranslationMax) {}
  /// Exploration width. Configuration space CALC_DITANCE_ A standard distance.
  /// The default is Euglid Distance
  double delta;
  /// The width of the interference check Delta> = sub_delta. The unit is equivalent to DELTA
  double sub_delta;
  /// Probability of generating an initial position using Start_tsrs
  double probability_start_generate;
  /// Probability of generating an initial position using Goal_tsrs
  double probability_goal_generate;
  /// Maximum number of repetitions
  int32_t max_itr;
  /// Timeout [S]
  double timeout;
  /// Do you want a shortcut
  bool do_shortcut;
  /// Do you do a biased sampling when sampling the goal and start?
  /// Limited to sampling the initial value terminal value with TSR
  bool sampling_around_initial;
  /// Standard deviation for biased sampling
  double sampling_distribution;
  /// Do you start sampling for biased sampling from 0
  bool increase_sampling_deviation;
  /// Increase_sampling_deviation increases the number of sampling by solving IK once
  /// Specify in the range of (0.0, 1.0]. The less time to search around the initial value, the more time you explore.
  double step_sampling_deviation;
  /// Base maximum parallel value [M]
  double base_translation_max;
  /// Maximum number of Connect operation
  boost::optional<int32_t> max_connect;
};

class RobotCBiRrtPlanner {
 public:
  using Ptr = std::shared_ptr<tmc_robot_planner::RobotCBiRrtPlanner>;

  RobotCBiRrtPlanner(
      tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_model,
      tmc_robot_collision_detector::RobotCollisionDetector::Ptr detector,
      tmc_robot_kinematics_model::IKSolver::Ptr ik_solver)
      : robot_model_(robot_model),
        robot_collision_detector_(detector),
        ik_solver_(ik_solver) {}
  ErrorCode PlanPath(const CBiRrtRequest& request,
                     const CBiRrtParameters& params,
                     tmc_manipulation_types::RobotTrajectory& result_out);
  void set_check_feasibility_callback(
      tmc_rplanner::CheckFeasibilityCallBackFunc check_feasibility_callback) {
    check_feasibility_callback_ = check_feasibility_callback;
  }
  void set_add_node_callback(
      tmc_rplanner::AddNodeCallBackFunc add_node_callback) {
    add_node_callback_ = add_node_callback;
  }
  void set_add_start_callback(
      tmc_rplanner::AddStartCallBackFunc add_start_callback) {
    add_start_callback_ = add_start_callback;
  }
  void set_add_goal_callback(
      tmc_rplanner::AddGoalCallBackFunc add_goal_callback) {
    add_goal_callback_ = add_goal_callback;
  }
  void set_constrain_config_callback(
      tmc_rplanner::ConstrainConfigCallBackFunc constrain_config_callback) {
    constrain_config_callback_ = constrain_config_callback;
  }
  std::vector<tmc_robot_collision_detector::PairString>
  last_contact_pair() const {return last_contact_pair_;}
  std::string limit_joint() const {return limit_joint_;}

 private:
  /// Robot geometric model
  tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr robot_model_;
  /// Interference check model
  tmc_robot_collision_detector::RobotCollisionDetector::Ptr
  robot_collision_detector_;
  /// IK Solva
  tmc_robot_kinematics_model::IKSolver::Ptr ik_solver_;

  /// For debugging the function of the function called when checking the configuration
  tmc_rplanner::CheckFeasibilityCallBackFunc check_feasibility_callback_;
  /// For debugging the function of the function called when node is added
  tmc_rplanner::AddNodeCallBackFunc add_node_callback_;
  /// For debugging the function of the function called at the time of START
  tmc_rplanner::AddStartCallBackFunc add_start_callback_;
  /// For debugging the function of the function called when GOAL is generated
  tmc_rplanner::AddGoalCallBackFunc add_goal_callback_;
  /// For debugging mainly callbacks called at ConstraintConfig
  tmc_rplanner::ConstrainConfigCallBackFunc constrain_config_callback_;
  /// Record the pair that hit the interference check
  std::vector<tmc_robot_collision_detector::PairString> last_contact_pair_;
  /// Record the joint on Limit
  std::string limit_joint_;
  /// Random number generator by MT19937
  std::mt19937 random_engine_;
  /// Rando number generator by Ranlux64_base_01
  boost::random::ranlux64_base_01 random_real_engine_;
};
// end of namespace tmc_robot_cbirrt_rplanner
}  // namespace tmc_robot_planner

#endif
