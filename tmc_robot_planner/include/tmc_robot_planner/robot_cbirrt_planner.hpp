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
/// @file     robot_cbirrt_planner.hpp
/// @brief    Planner using robot's CBIRRT
/// @author   Koji Terada

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
/// Limit of the cart's translation [m]
const double kDefaultBaseTranslationMax = 10.0;

/// Request given to CBiRrt
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
  /// List of joint names to be used. Only the joints listed here will be the target of exploration.
  std::vector<std::string> use_joints;
  /// Base movement New in version 0.16.0
  tmc_manipulation_types::BaseMovementType base_type;
  /// Initial values for all joints, including those not listed in use_joints.
  tmc_manipulation_types::JointState initial_config;
  /// Set of initial joint angles. Specified in configuration space.
  std::vector<Config> start_configs;
  /// Initial value of Base, specify if base_movement is other than kNone New in version 0.16.0
  tmc_manipulation_types::PoseSeq start_basejoint_to_bases;
  /// Set of terminal joint angles. Specified in configuration space.
  std::vector<Config> goal_configs;
  /// Terminal value of Base, specify if base_movement is other than kNone New in version 0.16.0
  tmc_manipulation_types::PoseSeq goal_basejoint_to_bases;
  /// Set of initial TSRs. Sample from here to add initial values.
  tmc_manipulation_types::TaskSpaceRegionSeq start_tsrs;
  /// Set of constraint TSRs. The entire trajectory is constrained by this.
  tmc_manipulation_types::TaskSpaceRegionSeq constraint_tsrs;
  /// Set of terminal TSRs. Sample from here to add initial values.
  tmc_manipulation_types::TaskSpaceRegionSeq goal_tsrs;
  /// Specify target values for joints not affected by IK when Start is specified with TSR
  tmc_manipulation_types::JointState start_no_ik_joint_state;
  /// Specify target values for joints not affected by IK when Start is specified with TSR
  tmc_manipulation_types::JointState goal_no_ik_joint_state;
  /// Robot reference position (this becomes the base reference when not kNone)
  Eigen::Affine3d origin_to_basejoint;
  /// Known objects for interference check
  tmc_manipulation_types::OuterObjectParametersSeq known_objects;
  /// Environment for interference check
  tmc_manipulation_types::CuboidSeq collision_map;
  /// Set of items held by the robot
  tmc_manipulation_types::AttachedObjectSeq attached_objects;
  /// Weight of each joint Positive value Usually set to 1 for joints you don't want to move much
  /// Set to a large value like 2 for joints you don't want to move much
  Config weight_config;
  /// Weight of each joint for IK Positive value Usually set to 1 for joints you don't want to move much
  /// Set to a large value like 100 for joints you don't want to move much
  Config weight_config_ik;
  /// Linear direction weight for base Set to about 0.1
  double weight_linear_base;
  /// Rotational direction weight for base Set to about 0.1
  double weight_rotational_base;
  /// Linear direction IK weight for base Set to about 0.1
  double weight_linear_base_ik;
  /// Rotational direction IK weight for base Set to about 0.1
  double weight_rotational_base_ik;
  /// Overall trajectory constraint Constrained from the 0th element after TSR constraint
  std::vector<IConfigurationConstraint::Ptr> extra_constraints;
  /// Start constraint Constrained from the 0th element after TSR constraint
  std::vector<IConfigurationConstraint::Ptr> extra_start_constraints;
  /// Goal constraint Constrained from the 0th element after TSR constraint
  std::vector<IConfigurationConstraint::Ptr> extra_goal_constraints;
  // For eigen fixed-length members
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// Planning parameters
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
  /// Width of exploration. Distance based on calc_distance_criterion in configuration space.
  /// Default is Euclidean distance
  double delta;
  /// Width of interference check Ensure delta >= sub_delta. Unit follows delta
  double sub_delta;
  /// Probability of generating initial position using start_tsrs
  double probability_start_generate;
  /// Probability of generating initial position using goal_tsrs
  double probability_goal_generate;
  /// Maximum number of iterations
  int32_t max_itr;
  /// Timeout [s]
  double timeout;
  /// Whether to perform a shortcut
  bool do_shortcut;
  /// Whether to perform biased sampling when sampling goal and start?
  /// Limited to when you want to sample initial and terminal values with TSR
  bool sampling_around_initial;
  /// Standard deviation for biased sampling
  double sampling_distribution;
  /// Whether to start sampling from zero during biased sampling
  bool increase_sampling_deviation;
  /// Increase in sampling per IK solution when increase_sampling_deviation is performed
  /// Specify in the range of (0.0, 1.0]. The smaller, the more time is spent exploring around the initial value
  double step_sampling_deviation;
  /// Maximum translation value for base movement [m]
  double base_translation_max;
  /// Maximum number of Connect operations
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
  /// IK solver
  tmc_robot_kinematics_model::IKSolver::Ptr ik_solver_;

  /// Function called during configuration check Mainly for debugging
  tmc_rplanner::CheckFeasibilityCallBackFunc check_feasibility_callback_;
  /// Function called when adding a node Mainly for debugging
  tmc_rplanner::AddNodeCallBackFunc add_node_callback_;
  /// Function called when generating start Mainly for debugging
  tmc_rplanner::AddStartCallBackFunc add_start_callback_;
  /// Function called when generating goal Mainly for debugging
  tmc_rplanner::AddGoalCallBackFunc add_goal_callback_;
  /// Callback called during ConstraintConfig Mainly for debugging
  tmc_rplanner::ConstrainConfigCallBackFunc constrain_config_callback_;
  /// Record the pair that collided during interference check
  std::vector<tmc_robot_collision_detector::PairString> last_contact_pair_;
  /// Record the joint that hit the limit
  std::string limit_joint_;
  /// Random number generator using mt19937
  std::mt19937 random_engine_;
  /// Random number generator using ranlux64_base_01
  boost::random::ranlux64_base_01 random_real_engine_;
};
// end of namespace tmc_robot_cbirrt_rplanner
}  // namespace tmc_robot_planner

#endif
