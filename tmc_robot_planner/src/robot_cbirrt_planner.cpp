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

#include <algorithm>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include <boost/timer/timer.hpp>

#include <tmc_eigen_utils/eigen_utils.hpp>
#include <tmc_robot_planner/robot_cbirrt_planner.hpp>
#include <tmc_robot_planner/robot_planner_common.hpp>
#include <tmc_robot_planner/task_space_region.hpp>
#include <tmc_rplanner/multi_birrt_planner.hpp>
#include <tmc_rplanner/raund_robin_short_cutter.hpp>

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
using tmc_robot_collision_detector::RobotCollisionDetector;
using tmc_robot_collision_detector::CuboidOverlapType;
using tmc_robot_collision_detector::CuboidOverlapGroupType;
using tmc_rplanner::MultiBirrtPlanner;
using tmc_rplanner::MultiBirrtPlannerParam;
using tmc_rplanner::ConfigurationSpace;

namespace {
// If the actual machine is zero -pointed, the simulator is a problem with accuracy, so the current posture can be a bit outside the limit, so it tolerates it.
constexpr double kJointLimitsTorelance = 1.0e-2;

const double kTsrNearThreshold = 1e-3;
/// Epsilon determines that the base restraint has been broken
const double kBaseNearThreshold = 1e-5;
/// Timeout Terminate object
class Terminate {
 public:
  explicit Terminate(double timeout) : timeout_(timeout), timer_() {
    timeout_nanosec_ = static_cast<boost::timer::nanosecond_type>(timeout_ * 1e9);
  }
  bool IsTerminate() {
    return (timer_.elapsed().user + timer_.elapsed().system) > timeout_nanosec_;
  }
  void Reset() {
    timer_.stop();
    timer_.start();
  }
 private:
  double timeout_;
  boost::timer::nanosecond_type timeout_nanosec_;
  boost::timer::cpu_timer timer_;
};


/// Part of Joint_state's joint angle,
///        Overwrite with diff_joint_state, return Merged_Joint_state
///        Ignore if there is no joint name of Diff_Joint_state
JointState MergeJointState(
    const JointState& joint_state,
    const JointState& diff_joint_state) {
  JointState modified_joint_state = joint_state;

  // Find a corresponding joint
  for (std::vector<std::string>::const_iterator diff_joint_it =
           diff_joint_state.name.begin();
       diff_joint_it != diff_joint_state.name.end();
       ++diff_joint_it) {
    std::vector<std::string>::const_iterator modified_joint_it =
        find(modified_joint_state.name.begin(),
             modified_joint_state.name.end(),
             *diff_joint_it);
    if (modified_joint_it != modified_joint_state.name.end()) {
      modified_joint_state.position(
          modified_joint_it - modified_joint_state.name.begin()) =
          diff_joint_state.position(
              diff_joint_it - diff_joint_state.name.begin());
    }
  }
  return modified_joint_state;
}

/// Obtain the lower and upper limit of Base_type
/// @param[in] base_type Base degree of freedom
/// @param[in] base_translation_max maximum parallel movement amount [M]
/// @param[out] config_min_out lower limit
/// @param[out] config_max_out upper limit
void GetBaseMinMax(tmc_manipulation_types::BaseMovementType base_type,
                   double base_translation_max,
                   Eigen::VectorXd& config_min_out,
                   Eigen::VectorXd& config_max_out) {
  uint32_t base_dof = GetBaseDof(base_type);
  Eigen::VectorXd config_min(base_dof);
  Eigen::VectorXd config_max(base_dof);
  switch (base_type) {
    case tmc_manipulation_types::kFloat:
      // Alradic limit
      config_min(0) = -base_translation_max;
      config_min(1) = -base_translation_max;
      config_min(2) = -base_translation_max;
      config_max(0) = base_translation_max;
      config_max(1) = base_translation_max;
      config_max(2) = base_translation_max;
      // Limit of rotation
      config_min(3) = -M_PI;
      config_min(4) = -0.5 * M_PI;
      config_min(5) = -M_PI;
      config_max(3) = M_PI;
      config_max(4) = 0.5 * M_PI;
      config_max(5) = M_PI;
      break;
    case tmc_manipulation_types::kPlanar:
      // Alradic limit
      config_min(0) = -base_translation_max;
      config_min(1) = -base_translation_max;
      config_max(0) = base_translation_max;
      config_max(1) = base_translation_max;
      // Limit of rotation
      config_min(2) = -M_PI;
      config_max(2) = M_PI;
      break;
    case tmc_manipulation_types::kRailX:
      config_min(0) = -base_translation_max;
      config_max(0) = base_translation_max;
      break;
    case tmc_manipulation_types::kRailY:
      config_min(0) = -base_translation_max;
      config_max(0) = base_translation_max;
      break;
    case tmc_manipulation_types::kRailZ:
      config_min(0) = -base_translation_max;
      config_max(0) = base_translation_max;
      break;
    case tmc_manipulation_types::kRotationX:
      config_min(0) = -M_PI;
      config_max(0) = M_PI;
      break;
    case tmc_manipulation_types::kRotationY:
      config_min(0) = -M_PI;
      config_max(0) = M_PI;
      break;
    case tmc_manipulation_types::kRotationZ:
      config_min(0) = -M_PI;
      config_max(0) = M_PI;
      break;
    case tmc_manipulation_types::kNone:
      break;
    default:
      break;
  }
  config_min_out = config_min;
  config_max_out = config_max;
}

/// separate joint angles and Base coordinates with bases_type
void DecodeConfigToJointAndBase(
    const Eigen::VectorXd& combined_config,
    tmc_manipulation_types::BaseMovementType base_type,
    Eigen::VectorXd& joint_config_out,
    Eigen::Affine3d& basejoint_to_base_out) {
  uint32_t base_dof = GetBaseDof(base_type);
  uint32_t joint_dof = combined_config.size() - base_dof;
  assert(joint_dof >= 0);

  Eigen::VectorXd base_config;
  basejoint_to_base_out = Eigen::Affine3d::Identity();

  joint_config_out = combined_config.head(joint_dof);
  base_config = combined_config.tail(base_dof);
  switch (base_type) {
    case tmc_manipulation_types::kFloat:
      basejoint_to_base_out.translation() = base_config.head(3);
      basejoint_to_base_out.rotate(
          tmc_eigen_utils::RPYToQuaternion(
              Eigen::Vector3d(base_config.tail(3))));
      break;
    case tmc_manipulation_types::kPlanar:
      basejoint_to_base_out.translation() =
          base_config(0) * Eigen::Vector3d::UnitX() +
          base_config(1) * Eigen::Vector3d::UnitY();
      basejoint_to_base_out.rotate(Eigen::AngleAxisd(base_config(2),
                                                     Eigen::Vector3d::UnitZ()));
      break;
    case tmc_manipulation_types::kRailX:
      basejoint_to_base_out.translation() =
          base_config(0) * Eigen::Vector3d::UnitX();
      break;
    case tmc_manipulation_types::kRailY:
      basejoint_to_base_out.translation() =
          base_config(0) * Eigen::Vector3d::UnitY();
      break;
    case tmc_manipulation_types::kRailZ:
      basejoint_to_base_out.translation() =
          base_config(0) * Eigen::Vector3d::UnitZ();
      break;
    case tmc_manipulation_types::kRotationX:
      basejoint_to_base_out.rotate(Eigen::AngleAxisd(base_config(0),
                                                     Eigen::Vector3d::UnitX()));
      break;
    case tmc_manipulation_types::kRotationY:
      basejoint_to_base_out.rotate(Eigen::AngleAxisd(base_config(0),
                                                     Eigen::Vector3d::UnitY()));
      break;
    case tmc_manipulation_types::kRotationZ:
      basejoint_to_base_out.rotate(Eigen::AngleAxisd(base_config(0),
                                                     Eigen::Vector3d::UnitZ()));
      break;
    case tmc_manipulation_types::kNone:
      break;
    default:
      break;
  }
}

/// Base_type combines joint angle and Base coordinates
void EncodeJointAndBaseToConfig(
    const Eigen::VectorXd& joint_config,
    const Eigen::Affine3d& basejoint_to_base,
    tmc_manipulation_types::BaseMovementType base_type,
    Eigen::VectorXd& combined_config_out) {
  uint32_t base_dof = GetBaseDof(base_type);
  uint32_t joint_dof = joint_config.size();
  uint32_t dof = base_dof + joint_dof;
  Eigen::VectorXd base_config(base_dof);
  Eigen::AngleAxisd basejoint_to_base_rotation;
  combined_config_out.resize(dof);

  switch (base_type) {
    case tmc_manipulation_types::kFloat:
      base_config.head(3) = basejoint_to_base.translation();
      base_config.tail(3) = tmc_eigen_utils::QuaternionToRPY(
          Eigen::Quaterniond(basejoint_to_base.rotation()));
      break;
    case tmc_manipulation_types::kPlanar:
      base_config(0) = basejoint_to_base.translation().x();
      base_config(1) = basejoint_to_base.translation().y();
      basejoint_to_base_rotation = Eigen::AngleAxisd(
          basejoint_to_base.rotation());
      base_config(2) = basejoint_to_base_rotation.angle()
                     * basejoint_to_base_rotation.axis().z();
      break;
    case tmc_manipulation_types::kRailX:
      base_config(0) = basejoint_to_base.translation().x();
      break;
    case tmc_manipulation_types::kRailY:
      base_config(0) = basejoint_to_base.translation().y();
      break;
    case tmc_manipulation_types::kRailZ:
      base_config(0) = basejoint_to_base.translation().z();
      break;
    case tmc_manipulation_types::kRotationX:
      basejoint_to_base_rotation = Eigen::AngleAxisd(
          basejoint_to_base.rotation());
      base_config(0) = basejoint_to_base_rotation.angle()
                     * basejoint_to_base_rotation.axis().x();
      break;
    case tmc_manipulation_types::kRotationY:
      basejoint_to_base_rotation = Eigen::AngleAxisd(
          basejoint_to_base.rotation());
      base_config(0) = basejoint_to_base_rotation.angle()
                       * basejoint_to_base_rotation.axis().y();
      break;
    case tmc_manipulation_types::kRotationZ:
      basejoint_to_base_rotation = Eigen::AngleAxisd(
          basejoint_to_base.rotation());
      base_config(0) = basejoint_to_base_rotation.angle()
                     * basejoint_to_base_rotation.axis().z();
      break;
    case tmc_manipulation_types::kNone:
      combined_config_out = joint_config;
      return;
    default:
      combined_config_out = joint_config;
      return;
  }
  combined_config_out.head(joint_dof) = joint_config;
  combined_config_out.tail(base_dof) = base_config;
}


/// end of noname namespace
}  // anonymous namespace

namespace tmc_robot_planner {

Config ExtractConfig(
    const JointState& joint_state,
    const std::vector<std::string>& joint_names) {
  Config config(joint_names.size());
  for (uint32_t i = 0; i < joint_names.size(); ++i) {
    std::vector<std::string>::const_iterator name =
        std::find(joint_state.name.begin(), joint_state.name.end(),
                  joint_names.at(i));
    if (name == joint_state.name.end()) {
      throw(std::invalid_argument(joint_names.at(i) + " is not found."));
    }
    config(i) = joint_state.position(
        std::distance(joint_state.name.begin(), name));
  }
  return config;
}

/// @brief Give a vector of configuration and return it with a certain probability
Config RandomConfigOrConfigs(
    const std::vector<Config>& ref_configs,
    double use_ref_threshold,
    std::mt19937& eng,
    tmc_rplanner::RandomConfigFunc random_config) {
  boost::uniform_real<> randf(0.0, 1.0);
  if ((randf(eng) < use_ref_threshold) && (!ref_configs.empty())) {
    boost::uniform_int<> randi(0, ref_configs.size()-1);
    return ref_configs.at(randi(eng));
  } else {
    return random_config();
  }
}


/// @brief Returns a random joint angle vector that satisfies the movable range
Config RandomConfig(
    const Config& config_min,
    const Config& config_max,
    std::mt19937& eng) {
  if (config_min.size() != config_max.size()) {
    throw(std::invalid_argument("Invalid Config size."));
  }
  size_t dim = config_min.size();
  Config random_config(dim);
  for (size_t i = 0; i < dim; ++i) {
    if (config_min(i) < config_max(i)) {
      boost::uniform_real<> randf(config_min(i), config_max(i));
      random_config(i) = randf(eng);
    } else {
      random_config(i) = config_min(i);
    }
  }
  return random_config;
}

/// @brief Returns a random joint angle vector that satisfies the movable range
///        However, follow the positive distribution around the REF
Config RandomConfigAroundRef(
    const Config& config_min,
    const Config& config_max,
    const Config& config_ref,
    const Config& config_sigma,
    boost::random::ranlux64_base_01& eng) {
  int32_t dim = config_min.size();
  if ((config_max.size() != dim) ||
      (config_ref.size() != dim) ||
      (config_sigma.size() != dim)) {
    throw(std::invalid_argument("Invalid Config size."));
  }
  Config random_config(dim);
  for (int32_t i = 0; i < dim; ++i) {
    std::normal_distribution<double> randn(config_ref(i), config_sigma(i));
    random_config(i) = std::min(
        std::max(randn(eng), config_min(i)), config_max(i));
  }
  return random_config;
}

/// Funkta that returns a random joint angle vector that satisfies the movable range
class RandomConfigIncreaseDev {
 public:
  RandomConfigIncreaseDev()
      : deviation_ratio_(0.0) {
  }
  /// Returns a random joint angle vector that satisfies the movable range
  /// However, every time it is called according to the normal distribution around the REF
  /// Deviation_step_ for the decentralization to config_sigma
  /// It will increase.
  Config Generate(
      double step_sampling_deviation,
      const Config& config_min,
      const Config& config_max,
      const Config& config_ref,
      const Config& config_sigma,
      boost::random::ranlux64_base_01& eng) {
    int32_t dim = config_min.size();
    if ((config_max.size() != dim) ||
        (config_ref.size() != dim) ||
        (config_sigma.size() != dim) ||
        (step_sampling_deviation < std::numeric_limits<double>::min())) {
      throw(std::invalid_argument("Invalid Config size."));
    }
    Config random_config(dim);
    for (int32_t i = 0; i < dim; ++i) {
      std::normal_distribution<double> randn(
          config_ref(i), deviation_ratio_ * config_sigma(i));
      random_config(i) = std::min(
          std::max(randn(eng),
                   config_min(i)),
          config_max(i));
    }
    deviation_ratio_ += step_sampling_deviation;
    if (deviation_ratio_ > 1.0) {
      deviation_ratio_ = 1.0;
    }
    return random_config;
  }

 private:
  double deviation_ratio_;
};


/// @brief Interference check + movable range check function
bool CheckContactAndLimit(
    const Config& config,
    const std::vector<std::string>& use_joints,
    tmc_manipulation_types::BaseMovementType base_type,
    const Eigen::Affine3d& origin_to_basejoint,
    const Config& joint_min,
    const Config& joint_max,
    RobotCollisionDetector::Ptr robot_collision_detector,
    std::string& joint_limit_out,
    std::vector<tmc_robot_collision_detector::PairString>& contact_pair_out) {
  joint_limit_out = "";
  contact_pair_out = std::vector<tmc_robot_collision_detector::PairString>();
  // Check if all joint angles are between min and max
  for (int32_t i = 0; i < config.size(); ++i) {
    if ((joint_min(i) - kJointLimitsTorelance > config(i)) || (joint_max(i) + kJointLimitsTorelance < config(i))) {
      if (i < use_joints.size()) {
        joint_limit_out += use_joints[i] + " ";
      } else {
        joint_limit_out += "base_movement ";
      }
      return false;
    }
  }
  // Set config
  Config joint_config;
  Eigen::Affine3d basejoint_to_base;
  DecodeConfigToJointAndBase(
      config, base_type, joint_config, basejoint_to_base);
  JointState joint_state = {use_joints, joint_config};
  robot_collision_detector->SetRobotTransform(
      origin_to_basejoint * basejoint_to_base);
  robot_collision_detector->SetRobotNamedAngle(joint_state);

  // Update Collision_map
  CuboidOverlapType over = tmc_robot_collision_detector::kOverlapAabb;
  CuboidOverlapGroupType group = tmc_robot_collision_detector::kOverlapGroup;
  robot_collision_detector->RefleshOverlappedCuboids(over, group);

  bool feasible = !(robot_collision_detector->CheckCollision(true, contact_pair_out));
  return feasible;
}

/// Check if the specified posture is in TSR
/// If there is no TSR restriction itself, be sure to return True
bool IsConfigInTsr(const Config& config,
                        const std::vector<std::string>& use_joints,
                        tmc_manipulation_types::BaseMovementType base_type,
                        const TaskSpaceRegionSeq& tsrs,
                        const Eigen::Affine3d& origin_to_basejoint,
                        IRobotKinematicsModel::Ptr robot_model) {
  // If you don't have TSR, you'll always be successful without doing anything
  if (tsrs.empty()) {
    return true;
  }

  Config joint_config;
  Eigen::Affine3d basejoint_to_base;
  DecodeConfigToJointAndBase(
      config, base_type, joint_config, basejoint_to_base);

  // Get END coordinates with Forward_kinematics
  JointState joint_state = {use_joints, joint_config};
  Eigen::Affine3d end_pose;

  robot_model->SetRobotTransform(origin_to_basejoint * basejoint_to_base);
  robot_model->SetNamedAngle(joint_state);
  end_pose = robot_model->GetObjectTransform(tsrs[0].end_frame_id);

  // Check if Pose is close
  if (CalcDistanceToTsr(tsrs.at(0), end_pose).norm() < kTsrNearThreshold) {
    return true;
  } else {
    return false;
  }
}

/// Restraint function to TSR
/// Currently only one TSR is supported
/// If there is no TSR, copy config to config_out and finish
/// If there is a TSR, use IK_SLOVER near the recently TSR.
/// Output the calculated IK value
bool ConstrainToTsr(const Config& config,
                    const std::vector<std::string>& use_joints,
                    tmc_manipulation_types::BaseMovementType base_type,
                    const TaskSpaceRegionSeq& tsrs,
                    const Eigen::Affine3d& origin_to_basejoint,
                    const Config& weight_ik,
                    IRobotKinematicsModel::Ptr robot_model,
                    IKSolver::Ptr ik_solver,
                    Config& config_out) {
  // If you don't have TSR, you'll always be successful without doing anything
  if (tsrs.empty()) {
    config_out = config;
    return true;
  }

  Config joint_config;
  Eigen::Affine3d basejoint_to_base;
  DecodeConfigToJointAndBase(
      config, base_type, joint_config, basejoint_to_base);

  /// Get END coordinates with Forward_kinematics
  JointState joint_state = {use_joints, joint_config};
  Eigen::Affine3d end_pose;

  robot_model->SetRobotTransform(origin_to_basejoint * basejoint_to_base);
  robot_model->SetNamedAngle(joint_state);
  end_pose = robot_model->GetObjectTransform(tsrs[0].end_frame_id);

  // If it is not exactly the same POSE, it will take on TSR
  if (CalcDistanceToTsr(tsrs.at(0), end_pose).norm() < kTsrNearThreshold) {
    config_out = config;
    return true;
  } else {
    // Recent TSR points
    Eigen::Affine3d origin_to_closest = CalcClosestPose(tsrs.at(0), end_pose);
    Eigen::Affine3d solution;
    IKRequest req(base_type);
    req.frame_name = tsrs[0].end_frame_id;
    req.frame_to_end = Eigen::Affine3d::Identity();
    req.ref_origin_to_end = origin_to_closest;
    req.origin_to_base = origin_to_basejoint * basejoint_to_base;
    req.initial_angle = joint_state;
    req.use_joints = use_joints;
    req.weight = weight_ik;
    JointState joint_result;
    Eigen::Affine3d origin_to_base_result;
    IKResult result;

    result = ik_solver->Solve(req, joint_result,
                              origin_to_base_result, solution);
    if (result == tmc_robot_kinematics_model::kSuccess) {
      Eigen::Affine3d basejoint_to_base =
          origin_to_basejoint.inverse() * origin_to_base_result;
      EncodeJointAndBaseToConfig(
          joint_result.position,
          basejoint_to_base,
          base_type,
          config_out);
      return true;
    } else {
      return false;
    }
  }
}

/// Auxiliary function for processing Extra_constraint
bool ConstrainTsrAndExtra(
    const tmc_rplanner::ConstraintFunc& tsr_constraint_func,
    const std::vector<std::string>& use_joints,
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const std::vector<IConfigurationConstraint::Ptr>& extra_constraints,
    const Config& config_in,
    Config& config_out) {
  Config tsr_config;
  if (!tsr_constraint_func(config_in, tsr_config)) {
    return false;
  }
  Config before_config = tsr_config;
  Config after_config = tsr_config;
  for (std::vector<IConfigurationConstraint::Ptr>::const_iterator constraint =
           extra_constraints.begin();
       constraint != extra_constraints.end();
       ++constraint) {
    if (!((*constraint)->Constrain(
            use_joints, robot, before_config, after_config))) {
      return false;
    } else {
      before_config = after_config;
    }
  }
  config_out = after_config;
  return true;
}

/// Auxiliary function for processing Extra_constraint
bool CheckFeasibilityAndExtra(
    const tmc_rplanner::CheckFeasibilityFunc& tsr_feasibility_func,
    const std::vector<std::string>& use_joints,
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const std::vector<IConfigurationConstraint::Ptr>& extra_constraints,
    const Config& config_in,
    Config& config_out) {
  Config before_config = config_in;
  Config after_config = config_in;
  for (std::vector<IConfigurationConstraint::Ptr>::const_iterator constraint =
           extra_constraints.begin();
       constraint != extra_constraints.end();
       ++constraint) {
    if (!((*constraint)->Constrain(
            use_joints, robot, before_config, after_config))) {
      return false;
    } else {
      before_config = after_config;
    }
  }

  if (!tsr_feasibility_func(after_config)) {
    return false;
  }

  config_out = after_config;
  return true;
}

/// Auxiliary function to restrain only EXTRA_CONSTRAINT
bool ConstrainExtra(
    const std::vector<std::string>& use_joints,
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const std::vector<IConfigurationConstraint::Ptr>& extra_constraints,
    const Config& config_in,
    Config& config_out) {
  Config before_config = config_in;
  Config after_config = config_in;
  for (std::vector<IConfigurationConstraint::Ptr>::const_iterator constraint =
           extra_constraints.begin();
       constraint != extra_constraints.end();
       ++constraint) {
    if (!((*constraint)->Constrain(
            use_joints, robot, before_config, after_config))) {
      return false;
    } else {
      before_config = after_config;
    }
  }
  config_out = after_config;
  return true;
}

/// Generate Config from TSR
bool SampleFromTsr(
    std::mt19937& gen,
    const CBiRrtRequest& request,
    const TaskSpaceRegionSeq& tsrs,
    const JointState& no_ik_joint_state,
    IKSolver::Ptr ik_solver,
    tmc_rplanner::RandomConfigFunc randconf,
    Config& config_out) {
  std::vector<std::string> use_joints = request.use_joints;
  tmc_manipulation_types::BaseMovementType base_type = request.base_type;
  Eigen::Affine3d origin_to_basejoint = request.origin_to_basejoint;
  Config weight_ik = request.weight_config_ik;
  // If there is no TSR, always fails
  if (tsrs.empty()) {
    return false;
  }
  Eigen::Affine3d solution;
  // Sampling from TSR
  boost::uniform_int<> randi(0, tsrs.size()-1);
  TaskSpaceRegion tsr = tsrs[randi(gen)];
  Eigen::Affine3d origin_to_sample = GenerateSample(tsr);
  // The initial value is generated by random numbers
  Config random_config = randconf();
  Config joint_config;
  Eigen::Affine3d basejoint_to_base;
  DecodeConfigToJointAndBase(
      random_config, base_type, joint_config, basejoint_to_base);

  JointState joint_state = {use_joints, joint_config};
  joint_state = MergeJointState(joint_state, no_ik_joint_state);
  IKRequest req(base_type);
  req.frame_name = tsr.end_frame_id;
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.ref_origin_to_end = origin_to_sample;
  req.origin_to_base = origin_to_basejoint * basejoint_to_base;
  req.initial_angle = joint_state;
  req.use_joints = use_joints;
  req.weight = weight_ik;

  JointState joint_result;
  Eigen::Affine3d origin_to_base_result;
  IKResult result = ik_solver->Solve(req, joint_result, origin_to_base_result, solution);
  if (result == tmc_robot_kinematics_model::kSuccess) {
    Eigen::Affine3d basejoint_to_base =
        origin_to_basejoint.inverse() * origin_to_base_result;
    EncodeJointAndBaseToConfig(
        joint_result.position,
        basejoint_to_base,
        base_type,
        config_out);
    return true;
  } else {
    return false;
  }
}

/// @brief Heavy joint angle distance calculation
/// l = Σw_i * (x_i - y_i)
/// Define the distance
double CalcWeightedDistance(const Config& config1,
                            const Config& config2,
                            const Config& weight_config) {
  // Configuration size is correct
  if ((config1.size() == config2.size()) &&
      (config1.size() == weight_config.size())) {
    return ((weight_config.cwiseProduct(config1 - config2)).norm());
  } else {
    throw tmc_rplanner::DimensionMismatch("Configuration size mismatch.");
  }
}

/// @brief Planning from any joint posture to any posture
ErrorCode RobotCBiRrtPlanner::PlanPath(
    const CBiRrtRequest& request,
    const CBiRrtParameters& params,
    RobotTrajectory& result_out) {
  uint32_t base_dof = GetBaseDof(request.base_type);
  uint32_t joint_dof = request.use_joints.size();
  uint32_t dof = base_dof + joint_dof;

  Config weight_config = request.weight_config;
  Config weight_config_ik = request.weight_config_ik;

  // Argument check
  if (!(params.delta > 0.0)) {
    throw std::invalid_argument("delta must be positive real");
  }
  // Argument check
  if (!(params.sub_delta > 0.0) || (params.sub_delta > params.delta)) {
    throw std::invalid_argument("sub_delta must be positive real");
  }
  // Argument check
  if (!(params.timeout > 0.0)) {
    throw std::invalid_argument("time out must be positive real");
  }
  // Argument check
  if ((params.probability_start_generate < 0.0) ||
      (params.probability_start_generate > 1.0)) {
    throw std::invalid_argument("probability must be [0, 1.0]");
  }
  // Argument check
  if ((params.probability_goal_generate < 0.0) ||
      (params.probability_goal_generate > 1.0)) {
    throw std::invalid_argument("probability must be [0, 1.0]");
  }
  // Argument check
  if (!(params.max_itr > 0)) {
    throw std::invalid_argument("max itr must be positive int");
  }

  // Check out Start, if you are empty, set a unit matrix, so it's through.
  if (!request.start_basejoint_to_bases.empty()) {
    if (request.start_configs.size() != request.start_basejoint_to_bases.size()) {
      throw std::invalid_argument(
          "uneven length of start_configs and start_basejoint_to_bases.");
    }
  }

  // Check out Goal, if Empty is EMPTY, sets a unit matrix.
  if (!request.goal_basejoint_to_bases.empty()) {
    if (request.goal_configs.size() != request.goal_basejoint_to_bases.size()) {
      throw std::invalid_argument(
          "uneven length of goal_configs and goal_basejoint_to_bases.");
    }
  }

  // Erase all objects
  robot_collision_detector_->DestroyAllOuterObject();
  // Create an object
  for (tmc_manipulation_types::OuterObjectParametersSeq::const_iterator
           object = request.known_objects.begin();
       object != request.known_objects.end(); ++object) {
    robot_collision_detector_->CreateOuterObject(*object);
  }
  // Added VOXEL environment
  robot_collision_detector_->CreateCuboids(request.collision_map, false);
  // Mounting object settings
  for (AttachedObjectSeq::const_iterator attached_object
           = request.attached_objects.begin();
       attached_object != request.attached_objects.end();
       ++attached_object) {
    robot_collision_detector_->HoldObject(attached_object->object_id,
                                          attached_object->frame_name,
                                          attached_object->frame_to_object);
    for (std::vector<std::string>::const_iterator expected_object =
             attached_object->expected_objects.begin();
         expected_object != attached_object->expected_objects.end();
         ++expected_object) {
      robot_collision_detector_->DisableCollisionCheckObjectToObject(
          attached_object->object_id,
          *expected_object);
    }
  }
  tmc_rplanner::PlanRet plan_ret;
  // Set the initial posture
  robot_collision_detector_->SetRobotNamedAngle(request.initial_config);
  robot_model_->SetNamedAngle(request.initial_config);
  // Robot position posture setting
  robot_collision_detector_->SetRobotTransform(request.origin_to_basejoint);
  robot_model_->SetRobotTransform(request.origin_to_basejoint);
  Terminate terminate(params.timeout);
  Config joint_max(joint_dof);
  Config joint_min(joint_dof);
  Config base_max(base_dof);
  Config base_min(base_dof);
  Config config_max(dof);
  Config config_min(dof);
    // Joint's min and MAX acquisition
  robot_model_->GetMinMax(request.use_joints, joint_min, joint_max);
  GetBaseMinMax(request.base_type, params.base_translation_max, base_min, base_max);
  config_min.head(joint_dof) = joint_min;
  config_min.tail(base_dof) = base_min;
  config_max.head(joint_dof) = joint_max;
  config_max.tail(base_dof) = base_max;
  // Creating a configuration space
  const auto space = std::make_shared<ConfigurationSpace>(dof);
  // Sunpla settings
  space->set_random_config(std::bind(RandomConfig, config_min, config_max, std::ref(random_engine_)));

  tmc_rplanner::CheckFeasibilityFunc check_feasibility =
      std::bind(CheckContactAndLimit, std::placeholders::_1,
                request.use_joints, request.base_type, request.origin_to_basejoint, config_min, config_max,
                robot_collision_detector_, std::ref(limit_joint_), std::ref(last_contact_pair_));
  space->set_check_feasibility(check_feasibility);
  // Set constrain
  tmc_rplanner::ConstraintFunc tsr_constrain_func =
      std::bind(ConstrainToTsr,
                std::placeholders::_1,
                request.use_joints, request.base_type, request.constraint_tsrs, request.origin_to_basejoint,
                request.weight_config_ik, robot_model_, ik_solver_,
                std::placeholders::_2);
  tmc_rplanner::ConstraintFunc constrain_config =
      std::bind(ConstrainTsrAndExtra,
                tsr_constrain_func, request.use_joints, robot_model_, request.extra_constraints,
                std::placeholders::_1, std::placeholders::_2);

  tmc_rplanner::CheckFeasibilityFunc tsr_feasibility_func =
      std::bind(IsConfigInTsr, std::placeholders::_1,
                request.use_joints, request.base_type, request.constraint_tsrs, request.origin_to_basejoint,
                robot_model_);
  tmc_rplanner::ConstraintFunc constrain_start_config =
      std::bind(CheckFeasibilityAndExtra,
                tsr_feasibility_func, request.use_joints, robot_model_, request.extra_start_constraints,
                std::placeholders::_1, std::placeholders::_2);
  tmc_rplanner::ConstraintFunc constrain_goal_config =
      std::bind(CheckFeasibilityAndExtra,
                tsr_feasibility_func, request.use_joints, robot_model_, request.extra_goal_constraints,
                std::placeholders::_1, std::placeholders::_2);
  space->set_constrain_config(constrain_config);
  space->set_constrain_start_config(constrain_start_config);
  space->set_constrain_goal_config(constrain_goal_config);
  // Set Weighted Distance
  tmc_rplanner::DistanceFunc weighted_distance =
      std::bind(CalcWeightedDistance, std::placeholders::_1, std::placeholders::_2, weight_config);
  space->set_distance(weighted_distance);

  tmc_rplanner::CheckFeasibilityFunc check_feasibility_impl = std::bind(
      &ConfigurationSpace::CheckFeasibility, space, std::placeholders::_1);
  tmc_rplanner::CheckTransferabilityFunc check_transferability =
      std::bind(tmc_rplanner::CheckTransferabilityByDividing, std::placeholders::_1, std::placeholders::_2,
                check_feasibility_impl, weighted_distance, params.sub_delta);
  space->set_check_transferability(check_transferability);

  // Random function used for GENERATE
  tmc_rplanner::RandomConfigFunc random_config_for_generate;
  // Sampling from Initial posture when specified
  Config config_sigma = weight_config_ik.cwiseInverse();
  Config joint_ref = ExtractConfig(request.initial_config, request.use_joints);
  Config config_ref;
  Eigen::Affine3d unit = Eigen::Affine3d::Identity();
  EncodeJointAndBaseToConfig(joint_ref, unit, request.base_type, config_ref);

  RandomConfigIncreaseDev increasing_generator;
  if (params.sampling_around_initial) {
    if (params.increase_sampling_deviation) {
      random_config_for_generate = std::bind(
          &RandomConfigIncreaseDev::Generate, increasing_generator,
          params.step_sampling_deviation, config_min, config_max, config_ref, config_sigma,
          std::ref(random_real_engine_));
    } else {
      random_config_for_generate =
          std::bind(RandomConfigAroundRef, config_min, config_max, config_ref, config_sigma,
          std::ref(random_real_engine_));
    }
  } else {
    random_config_for_generate = std::bind(RandomConfig, config_min, config_max, std::ref(random_engine_));
  }

  // Set Generate_start_config
  tmc_rplanner::GenerateStartConfigFunc generate_start_config =
      std::bind(SampleFromTsr,
                random_engine_, request, request.start_tsrs, request.start_no_ik_joint_state,
                ik_solver_, random_config_for_generate,
                std::placeholders::_1);
  space->set_generate_start_config(generate_start_config);
  // Set Generate_goal_config
  tmc_rplanner::GenerateGoalConfigFunc generate_goal_config =
      std::bind(SampleFromTsr,
                random_engine_, request, request.goal_tsrs, request.goal_no_ik_joint_state,
                ik_solver_, random_config_for_generate,
                std::placeholders::_1);
  space->set_generate_goal_config(generate_goal_config);
  // Added if you have a debugging callback
  if (check_feasibility_callback_) {
    space->set_check_feasibility_callback(check_feasibility_callback_);
  }
  if (add_node_callback_) {
    space->set_add_node_callback(add_node_callback_);
  }
  if (add_start_callback_) {
    space->set_add_start_callback(add_start_callback_);
  }
  if (add_goal_callback_) {
    space->set_add_goal_callback(add_goal_callback_);
  }
  if (constrain_config_callback_) {
    space->set_constrain_config_callback(constrain_config_callback_);
  }
  // Planna parameter settings
  MultiBirrtPlannerParam planner_param;
  planner_param.delta = params.delta;
  planner_param.max_itr = params.max_itr;
  planner_param.probability_start_generate = params.probability_start_generate;
  planner_param.probability_goal_generate = params.probability_goal_generate;
  planner_param.max_connect = params.max_connect;
  planner_param.is_terminate = std::bind(&Terminate::IsTerminate, &terminate);
  // 2 -point combined planna by CBirrt
  MultiBirrtPlanner::Ptr planner(new MultiBirrtPlanner(space, planner_param));
  terminate.Reset();

  std::vector<Config> start_configs;
  std::vector<Config> goal_configs;

  for (uint32_t i = 0; i < request.start_configs.size(); ++i) {
    Config start_config;
    if (request.start_basejoint_to_bases.empty()) {
      EncodeJointAndBaseToConfig(
          request.start_configs[i],
          unit,
          request.base_type,
          start_config);
    } else {
      EncodeJointAndBaseToConfig(
          request.start_configs[i],
          request.start_basejoint_to_bases[i],
          request.base_type,
          start_config);
    }
    start_configs.push_back(start_config);
  }

  for (uint32_t i = 0; i < request.goal_configs.size(); ++i) {
    Config goal_config;
    if (request.goal_basejoint_to_bases.empty()) {
      EncodeJointAndBaseToConfig(
          request.goal_configs[i],
          unit,
          request.base_type,
          goal_config);
    } else {
      EncodeJointAndBaseToConfig(
          request.goal_configs[i],
          request.goal_basejoint_to_bases[i],
          request.base_type,
          goal_config);
    }
    goal_configs.push_back(goal_config);
  }

  // Pass generation
  tmc_rplanner::Path path;
  plan_ret = planner->PlanPath(start_configs, goal_configs, path);

  // terminate.Reset();
  switch (plan_ret) {
    case tmc_rplanner::kSuccess: {
      // terminate.Reset();
      // Pass shortcut
      tmc_rplanner::Path opt_path;
      if (params.do_shortcut) {
        // Bead 2-PASS shortcut
        tmc_rplanner::RoundRobinShortCutter short_cutter(
            space, planner_param.delta, true, 1, std::bind(&Terminate::IsTerminate, &terminate));
        if (!short_cutter.ShortCut(path, opt_path)) {
          return kShortcutTimedOut;
        }
      } else {
        opt_path = path;
      }
      tmc_manipulation_types::Path joint_path;
      tmc_manipulation_types::MultiDOFPath multi_dof_path;
      for (uint32_t i = 0; i < opt_path.size(); ++i) {
          Config joint_config;
          Eigen::Affine3d basejoint_to_base;
          tmc_manipulation_types::PoseSeq multi_dof_config;
          DecodeConfigToJointAndBase(
              opt_path[i], request.base_type,
              joint_config, basejoint_to_base);
          joint_path.push_back(joint_config);
          multi_dof_config.push_back(request.origin_to_basejoint * basejoint_to_base);
          multi_dof_path.push_back(multi_dof_config);
      }
      JointTrajectory joint_trajectory = {request.use_joints, joint_path};
      tmc_manipulation_types::NameSeq multi_dof_name;
      multi_dof_name.push_back("origin_to_base");
      MultiDOFJointTrajectory multi_dof_joint_trajectory = {multi_dof_name, multi_dof_path};
      result_out.joint_trajectory = joint_trajectory;
      result_out.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
      return kSuccess;
    }
    case tmc_rplanner::kTerminate:
      return kTimedOut;
    case tmc_rplanner::kInitConfigFail:
      return kStartStateInCollision;
    case tmc_rplanner::kGoalConfigFail:
      return kGoalStateInCollision;
    case tmc_rplanner::kMaxItr:
      return kPlanningFailed;
    default:
      assert(false);
  }

  return kPlanningFailed;
}
// end of namespace tmc_robot_rplanner
}  // namespace tmc_robot_planner
