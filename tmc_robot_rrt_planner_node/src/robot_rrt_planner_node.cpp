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

#include "robot_rrt_planner_node.hpp"

#include <limits>

#include <tf2_eigen/tf2_eigen.hpp>

#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_utils/parameters.hpp>

namespace {
const double kDefaultDelta = 0.1;
const double kDefaultSubDelta = 0.02;
const int32_t kDefaultPublisherBuffer = 100;
const double kDefaultStepSampling = 0.01;
const double kDefaultWeightLinearBase = 10.0;
const double kDefaultWeightRotationalBase = 10.0;
const double kDefaultWeightLinearBaseIK = 10.0;
const double kDefaultWeightRotationalBaseIK = 10.0;
const double kDefaultBaseTranslationMax = 10.0;

// Numerical IK maximum number of repetitions
const int32_t kMaxItrIK = 1000;
// Numerical IK tolerance error
const double kIKDelta = 1.0e-3;
// Numerical IK tolerance fluctuation
const double kIKConvergeThreshold = 1.0e-10;
// // Frameid for debugging
// const char* const kDebugFrameId = "/debug/robot_base";

// /// List of Marker ID and Robot_collisin_detector
// typedef std::map<string, visualization_msgs::Marker> AttachedMarkerList;

/// @brief Take out the joint angle specified by Joint_names from Initial_state
Eigen::VectorXd ExtractJoints(
    const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& robot_collision_detector,
    const tmc_manipulation_types::JointState& initial_state,
    const std::vector<std::string>& joint_names) {
  robot_collision_detector->SetRobotNamedAngle(initial_state);
  return robot_collision_detector->GetRobotNamedAngle(joint_names).position;
}

/// Take out all joint angles
tmc_manipulation_types::JointState FetchAllJoints(
    const tmc_robot_collision_detector::RobotCollisionDetector::Ptr& robot_collision_detector,
    const tmc_manipulation_types::JointState& partial_joint_state) {
  robot_collision_detector->SetRobotNamedAngle(partial_joint_state);
  return robot_collision_detector->GetRobotNamedAngle();
}

void InitPoseMsg(geometry_msgs::msg::Pose& pose) {
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
}

/// @brief Calculate the simultaneous conversion where the X axis faces AXIS direction
Eigen::Affine3d CalcPoseFromAxisX(const Eigen::Vector3d& x_axis) {
  Eigen::Vector3d x_axis_n(x_axis);
  x_axis_n.normalize();
  Eigen::Vector3d outerp = Eigen::Vector3d::UnitX().cross(x_axis_n);
  double theta = acos(Eigen::Vector3d::UnitX().dot(x_axis_n));
  if (outerp.norm() < std::numeric_limits<double>::epsilon()) {
    outerp = Eigen::Vector3d::UnitY();
  }
  outerp.normalize();
  Eigen::AngleAxisd aa(theta, outerp);
  return Eigen::Affine3d::Identity() * aa;
}

/// @brief Build the last IK's chain of responsibility with numerical IK.
tmc_robot_kinematics_model::IKSolver::Ptr LoadIKSolver(
    const rclcpp::Logger& logger,
    const std::vector<std::string>& ik_plugins,
    const tmc_robot_kinematics_model::IRobotKinematicsModel::Ptr& robot,
    const std::string& robot_model,
    pluginlib::ClassLoader<tmc_robot_kinematics_model::IKSolver>& ik_plugin_loader) {
  /// Read IK_plugin
  std::vector<tmc_robot_kinematics_model::IKSolver::Ptr> ik_plugin_solvers;
  for (const auto& plugin : ik_plugins) {
    try {
      auto ik_plugin_solver = ik_plugin_loader.createSharedInstance(plugin);
      ik_plugin_solver->set_robot_description(robot_model);
      ik_plugin_solvers.push_back(ik_plugin_solver);
    } catch(const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(logger, "The plugin %s failed to load for some reason. Error: %s", plugin.c_str(), ex.what());
    }
  }

  auto numeric_ik_solver = std::make_shared<tmc_robot_kinematics_model::NumericIKSolver>(
      tmc_robot_kinematics_model::IKSolver::Ptr(), robot, kMaxItrIK, kIKDelta, kIKConvergeThreshold);

  tmc_robot_kinematics_model::IKSolver::Ptr chain_ik_solver;
  if (ik_plugin_solvers.empty()) {
    chain_ik_solver = numeric_ik_solver;
  } else {
    for (auto i = 1; i < ik_plugin_solvers.size(); ++i) {
      ik_plugin_solvers[i]->set_successor(ik_plugin_solvers[i - 1]);
    }
    ik_plugin_solvers[0]->set_successor(numeric_ik_solver);
    chain_ik_solver = ik_plugin_solvers.back();
  }
  return chain_ik_solver;
}

/// @brief ORIGINAL_WEIGHT_CONFIG overwrites with weIGHTS to create Weight_config
tmc_robot_planner::Config CalculateWeightConfig(
    const rclcpp::Logger& logger,
    const std::vector<std::string>& use_joints,
    const tmc_robot_planner::Config& original_weights,
    const std::vector<std::string>& weighted_joints,
    const std::vector<double>& weights,
    double weight_linear_base,
    double weight_rotatinal_base,
    tmc_manipulation_types::BaseMovementType base_type) {
  uint32_t base_dof = tmc_manipulation_types::GetBaseDof(base_type);
  if ((weighted_joints.size() != weights.size()) ||
      (original_weights.size() != static_cast<int32_t>(use_joints.size()) + base_dof)) {
    RCLCPP_FATAL(logger, "Mismatch weight joint size and weights size.");
    return tmc_robot_planner::Config();
  }

  tmc_robot_planner::Config weight_config = original_weights;
  for (uint32_t i = 0; i < use_joints.size(); ++i) {
    for (uint32_t j = 0; j < weighted_joints.size(); ++j) {
      if (use_joints.at(i) == weighted_joints.at(j)) {
        if (weights.at(j) > 0.0) {
          weight_config(i) = weights.at(j);
        } else {
          RCLCPP_FATAL(logger, "Invalid weight. Weight must be positive real.");
          return tmc_robot_planner::Config();
        }
      }
    }
  }

  uint32_t joint_dof = use_joints.size();
  // Base weight setting
  switch (base_type) {
    case tmc_manipulation_types::kFloat:
      weight_config.segment(joint_dof, 3).fill(weight_linear_base);
      weight_config.tail(3).fill(weight_rotatinal_base);
      break;
    case tmc_manipulation_types::kPlanar:
      weight_config.segment(joint_dof, 2).fill(weight_linear_base);
      weight_config.tail(1).fill(weight_rotatinal_base);
      break;
    case tmc_manipulation_types::kRailX:
      weight_config.tail(1).fill(weight_linear_base);
      break;
    case tmc_manipulation_types::kRailY:
      weight_config.tail(1).fill(weight_linear_base);
      break;
    case tmc_manipulation_types::kRailZ:
      weight_config.tail(1).fill(weight_linear_base);
      break;
    case tmc_manipulation_types::kRotationX:
      weight_config.tail(1).fill(weight_rotatinal_base);
      break;
    case tmc_manipulation_types::kRotationY:
      weight_config.tail(1).fill(weight_rotatinal_base);
      break;
    case tmc_manipulation_types::kRotationZ:
      weight_config.tail(1).fill(weight_rotatinal_base);
      break;
    case tmc_manipulation_types::kNone:
      break;
    default:
      break;
  }
  return weight_config;
}

/// @brief Extract the weight of target_name
double ExtractWeight(const std::vector<std::string>& joint_names,
                     const std::vector<double>& weights,
                     const std::string& target_name,
                     double default_value) {
  const auto it = std::find(joint_names.begin(), joint_names.end(), target_name);
  if (it == joint_names.end()) {
    return default_value;
  } else {
    return weights.at(std::distance(joint_names.begin(), it));
  }
}

/// @brief Read a plugin that restricts the joints
tmc_robot_planner::IConfigurationConstraint::Ptr LoadConfigurationConstraint(
    const rclcpp::Logger& logger,
    const std::string& name,
    pluginlib::ClassLoader<tmc_robot_planner::IConfigurationConstraint>& loader,
    std::map<std::string, tmc_robot_planner::IConfigurationConstraint::Ptr>& cache) {
  tmc_robot_planner::IConfigurationConstraint::Ptr constraint;

  const auto it = cache.find(name);
  if (it == cache.end()) {
    try {
      constraint = loader.createSharedInstance(name);
    } catch(const pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(logger, "The plugin %s failed to load for some reason. Error: %s", name.c_str(), ex.what());
      return nullptr;
    }
  } else {
    constraint = it->second;
  }

  if (constraint->Reset()) {
    return constraint;
  } else {
    return nullptr;
  }
}

}  // anonymous namespace


namespace tmc_robot_rrt_planner_node {

// Default constructor
RobotRrtPlannerNode::RobotRrtPlannerNode(const rclcpp::NodeOptions& options)
    : Node("robot_rrt_planner_node", options),
      constraint_plugin_loader_("tmc_robot_planner", "tmc_robot_planner::IConfigurationConstraint"),
      ik_plugin_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IKSolver"),
      fk_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IRobotKinematicsModel") {}

bool RobotRrtPlannerNode::Init() {
  auto node = shared_from_this();

  // Robot model, initialization of Planner
  auto robot_model = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_model.empty()) {
    robot_model = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  if (robot_model.empty()) {
    RCLCPP_FATAL(get_logger(), "cannot get paramter robot_description");
    return false;
  }

  const auto robot_collision_pair = tmc_utils::GetParameter<std::string>(node, "robot_collision_pair", "");
  if (robot_collision_pair.empty()) {
    RCLCPP_FATAL(get_logger(), "cannot get paramter robot_collision_pair");
    return false;
  }
  const auto collision_engine = tmc_utils::GetParameter<std::string>(node, "collision_engine", "ODE");

  const auto kinematics_type = tmc_utils::GetParameter<std::string>(
      node, "kinematics_type", "tmc_robot_kinematics_model/PinocchioWrapper");
  auto robot = fk_loader_.createSharedInstance(kinematics_type);
  robot->Initialize(robot_model);

  robot_collision_detector_ = std::make_shared<tmc_robot_collision_detector::RobotCollisionDetector>(
      robot, robot_model, robot_collision_pair, collision_engine);

  const auto ik_plugins = tmc_utils::GetParameter<std::vector<std::string>>(node, "ik_plugins", {});
  auto ik_solver = LoadIKSolver(get_logger(), ik_plugins, robot, robot_model, ik_plugin_loader_);

  planner_ = std::make_shared<tmc_robot_planner::RobotCBiRrtPlanner>(robot, robot_collision_detector_, ik_solver);

  // Whether to output a file request for orbital plan
  const auto save_request = tmc_utils::GetParameter<bool>(node, "save_request", false);
  if (save_request) {
    request_logger_ = std::make_shared<tmc_utils::MessageLogger>(tmc_utils::GetLogDirectory() + "/plan_");
  }

  // Acquisition of weight parameters
  weight_names_ = tmc_utils::GetParameter<std::vector<std::string>>(node, "weight_names", {});
  weights_ = tmc_utils::GetParameter<std::vector<double>>(node, "weights", {});
  if (weight_names_.size() != weights_.size()) {
    RCLCPP_FATAL(get_logger(), "Mismatch weight_names and weights size.");
    return false;
  }

  weight_linear_base_ = tmc_utils::GetParameter<double>(node, "weight_linear_base", kDefaultWeightLinearBase);
  weight_rotational_base_ = tmc_utils::GetParameter<double>(
      node, "weight_rotational_base", kDefaultWeightRotationalBase);

  ik_weight_names_ = tmc_utils::GetParameter<std::vector<std::string>>(node, "ik_weight_names", {});
  ik_weights_ = tmc_utils::GetParameter<std::vector<double>>(node, "ik_weights", {});
  if (ik_weight_names_.size() != ik_weights_.size()) {
    RCLCPP_FATAL(get_logger(), "Mismatch ik_weight_names and ik_weights size.");
    return false;
  }

  weight_linear_base_ik_ = tmc_utils::GetParameter<double>(node, "weight_linear_base_ik", kDefaultWeightLinearBaseIK);
  weight_rotational_base_ik_ = tmc_utils::GetParameter<double>(
      node, "weight_rotational_base_ik", kDefaultWeightRotationalBaseIK);

  delta_ = tmc_utils::GetParameter<double>(node, "delta", kDefaultDelta);
  sub_delta_ = tmc_utils::GetParameter<double>(node, "sub_delta", kDefaultSubDelta);
  base_translation_max_ = tmc_utils::GetParameter<double>(node, "base_translation_max", kDefaultBaseTranslationMax);
  increase_sampling_deviation_ = tmc_utils::GetParameter<bool>(node, "increase_sampling_deviation", true);
  step_sampling_deviation_ = tmc_utils::GetParameter<double>(node, "step_sampling_deviation", kDefaultStepSampling);

  publish_debug_info_ = tmc_utils::GetParameter<bool>(node, "publish_debug_info", false);
  step_mode_ = tmc_utils::GetParameter<bool>(node, "step_mode", false);

  if (publish_debug_info_) {
    debug_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "debug_joint_state", kDefaultPublisherBuffer);
    // debug_environment_pub_ = node_.advertise<visualization_msgs::MarkerArray>(
    //     "debug_environment", kDefaultPublisherBuffer);
    debug_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  plan_with_constraints_service_ = create_service<tmc_planning_msgs::srv::PlanWithTsrConstraints>(
      "plan_with_constraints",
      std::bind(&RobotRrtPlannerNode::PlanWithTsrConstraints, this, std::placeholders::_1, std::placeholders::_2));
  plan_with_joints_service_ = create_service<tmc_planning_msgs::srv::PlanWithJointGoals>(
      "plan_with_joint_goals",
      std::bind(&RobotRrtPlannerNode::PlanWithJointGoals, this, std::placeholders::_1, std::placeholders::_2));
  plan_with_hand_service_ = create_service<tmc_planning_msgs::srv::PlanWithHandGoals>(
      "plan_with_hand_goals",
      std::bind(&RobotRrtPlannerNode::PlanWithHandGoals, this, std::placeholders::_1, std::placeholders::_2));
  plan_with_line_service_ = create_service<tmc_planning_msgs::srv::PlanWithHandLine>(
      "plan_with_hand_line",
      std::bind(&RobotRrtPlannerNode::PlanWithHandLine, this, std::placeholders::_1, std::placeholders::_2));

  return true;
}

void RobotRrtPlannerNode::PlanWithTsrConstraints(
    const tmc_planning_msgs::srv::PlanWithTsrConstraints::Request::SharedPtr req,
    tmc_planning_msgs::srv::PlanWithTsrConstraints::Response::SharedPtr res) {
  if (request_logger_) {
    request_logger_->UpdateStamp(this->now());
    if (!request_logger_->SaveMessage(".request", *req)) {
      RCLCPP_WARN(get_logger(), "Cannot open log file try next.");
    }
  }

  tmc_robot_planner::CBiRrtRequest planning_request;

  tmc_manipulation_types_bridge::BaseMovementTypeMsgToBaseMovement(
      req->base_movement_type, planning_request.base_type);
  tmc_manipulation_types_bridge::JointStateMsgToJointState(req->initial_joint_state, planning_request.initial_config);

  tmc_manipulation_types_bridge::ConvertSequence<tmc_planning_msgs::msg::JointPosition, tmc_rplanner::Config>(
      req->start_joint_states, planning_request.start_configs,
      tmc_manipulation_types_bridge::JointPositionMsgToConfig);
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<geometry_msgs::msg::Pose, Eigen::Affine3d>(
      req->start_basejoint_to_bases, planning_request.start_basejoint_to_bases,
      std::bind<void(const geometry_msgs::msg::Pose&, Eigen::Affine3d&)>(
          tf2::fromMsg, std::placeholders::_1, std::placeholders::_2));

  tmc_manipulation_types_bridge::ConvertSequence<tmc_planning_msgs::msg::JointPosition, tmc_rplanner::Config>(
      req->goal_joint_states, planning_request.goal_configs,
      tmc_manipulation_types_bridge::JointPositionMsgToConfig);
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<geometry_msgs::msg::Pose, Eigen::Affine3d>(
      req->goal_basejoint_to_bases, planning_request.goal_basejoint_to_bases,
      std::bind<void(const geometry_msgs::msg::Pose&, Eigen::Affine3d&)>(
          tf2::fromMsg, std::placeholders::_1, std::placeholders::_2));

  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<tmc_planning_msgs::msg::TaskSpaceRegion,
                                                             tmc_manipulation_types::TaskSpaceRegion>(
      req->constraint_tsrs, planning_request.constraint_tsrs,
      tmc_manipulation_types_bridge::TaskSpaceRegionMsgToTaskSpaceRegion);
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<tmc_planning_msgs::msg::TaskSpaceRegion,
                                                             tmc_manipulation_types::TaskSpaceRegion>(
      req->start_tsrs, planning_request.start_tsrs,
      tmc_manipulation_types_bridge::TaskSpaceRegionMsgToTaskSpaceRegion);
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<tmc_planning_msgs::msg::TaskSpaceRegion,
                                                             tmc_manipulation_types::TaskSpaceRegion>(
      req->goal_tsrs, planning_request.goal_tsrs,
      tmc_manipulation_types_bridge::TaskSpaceRegionMsgToTaskSpaceRegion);

  planning_request.use_joints = req->use_joints;
  planning_request.start_configs.push_back(
      ExtractJoints(robot_collision_detector_, planning_request.initial_config, req->use_joints));

  tmc_manipulation_types_bridge::JointStateMsgToJointState(
      req->start_no_ik_joint_state, planning_request.start_no_ik_joint_state);
  tmc_manipulation_types_bridge::JointStateMsgToJointState(
      req->goal_no_ik_joint_state, planning_request.goal_no_ik_joint_state);

  tf2::fromMsg(req->origin_to_basejoint, planning_request.origin_to_basejoint);

  uint32_t base_dof = tmc_manipulation_types::GetBaseDof(planning_request.base_type);
  planning_request.weight_config = CalculateWeightConfig(
      get_logger(), req->use_joints, Eigen::VectorXd::Ones(req->use_joints.size() + base_dof),
      weight_names_, weights_, weight_linear_base_, weight_rotational_base_, planning_request.base_type);
  if (planning_request.weight_config.size() == 0) {
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return;
  }

  planning_request.weight_config_ik = CalculateWeightConfig(
      get_logger(), req->use_joints, Eigen::VectorXd::Ones(req->use_joints.size() + base_dof),
      ik_weight_names_, ik_weights_, weight_linear_base_ik_, weight_rotational_base_ik_, planning_request.base_type);
  // Overwrite the weight of IK by weighting in Request
  // If you have '_linear_base' in Weighted_Joints, overwrite Linear_base_ik
  // If weighted_Joints has '_rotational_base', overwrite Rotational_base_ik
  if (req->weighted_joints.size() != req->weight.size()) {
    RCLCPP_ERROR(get_logger(), "Mismatch weight joint size and weights size.");
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return;
  }
  planning_request.weight_config_ik = CalculateWeightConfig(
      get_logger(), req->use_joints, planning_request.weight_config_ik, req->weighted_joints, req->weight,
      ExtractWeight(req->weighted_joints, req->weight, "_linear_base", weight_rotational_base_ik_),
      ExtractWeight(req->weighted_joints, req->weight, "_rotational_base", weight_rotational_base_ik_),
      planning_request.base_type);
  if (planning_request.weight_config_ik.size() == 0) {
    res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return;
  }

  // Read the restraint plugin
  for (const auto& plugin : req->extra_constraints) {
    tmc_robot_planner::IConfigurationConstraint::Ptr constraint;
    if (!LoadConfigurationConstraint(get_logger(), plugin, constraint_plugin_loader_, constraint_plugin_cache_)) {
      res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return;
    }
    planning_request.extra_constraints.push_back(constraint);
    planning_request.extra_start_constraints.push_back(constraint);
    planning_request.extra_goal_constraints.push_back(constraint);
  }
  for (const auto& plugin : req->extra_start_constraints) {
    tmc_robot_planner::IConfigurationConstraint::Ptr constraint;
    if (!LoadConfigurationConstraint(get_logger(), plugin, constraint_plugin_loader_, constraint_plugin_cache_)) {
      res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return;
    }
    planning_request.extra_start_constraints.push_back(constraint);
  }
  for (const auto& plugin : req->extra_goal_constraints) {
    tmc_robot_planner::IConfigurationConstraint::Ptr constraint;
    if (!LoadConfigurationConstraint(get_logger(), plugin, constraint_plugin_loader_, constraint_plugin_cache_)) {
      res->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return;
    }
    planning_request.extra_goal_constraints.push_back(constraint);
  }

  // Use of external obstacles and amendment
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<moveit_msgs::msg::CollisionObject,
                                                             tmc_manipulation_types::OuterObjectParameters>(
      req->environment_before_planning.collision_objects,  planning_request.known_objects,
      tmc_manipulation_types_bridge::CollisionObjectToOuterObjectParameters);
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<moveit_msgs::msg::AttachedCollisionObject,
                                                             tmc_manipulation_types::AttachedObject>(
      req->attached_objects, planning_request.attached_objects,
      tmc_manipulation_types_bridge::AttachedObjectMsgToAttachedObject);

  if (publish_debug_info_) {
    // PublishEnvironmentDebug(req->environment_before_planning, true, debug_environment_pub_);
    // SetDebugCallBacks_(req->use_joints, planning_request.attached_objects, req->environment_before_planning);
    SetDebugCallBacks_(req->use_joints);
    RCLCPP_INFO(get_logger(), "[planner_node]:Start planning with tsr constraints!");
  }

  tmc_robot_planner::CBiRrtParameters params;
  params.base_translation_max = base_translation_max_;
  params.timeout = rclcpp::Duration(req->timeout).seconds();
  params.delta = delta_;
  params.sub_delta = sub_delta_;
  params.max_itr = req->max_iteration;
  params.do_shortcut = true;
  params.probability_start_generate = req->probability_start_generate;
  params.probability_goal_generate = req->probability_goal_generate;
  params.sampling_around_initial = !(req->uniform_bound_sampling);
  params.sampling_distribution = req->deviation_for_bound_sampling;
  params.increase_sampling_deviation = increase_sampling_deviation_;
  params.step_sampling_deviation = step_sampling_deviation_;

  tmc_manipulation_types::RobotTrajectory path;
  tmc_robot_planner::ErrorCode result;
  const auto start_stamp = this->now();
  try {
    result = planner_->PlanPath(planning_request, params, path);
  } catch(std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "[Planning failed by exception] %s", ex.what());
    return;
  }
  const auto end_stamp = this->now();
  RCLCPP_INFO_STREAM(get_logger(), "planning time = " << (end_stamp - start_stamp).seconds());

  res->error_code.val = result;
  if (result == tmc_robot_planner::kSuccess) {
    tmc_manipulation_types_bridge::JointTrajectoryToJointTrajectoryMsg(path.joint_trajectory, res->solution);
    tmc_manipulation_types_bridge::MultiDOFJointTrajectoryToMultiDOFJointTrajectoryMsg(
        path.multi_dof_joint_trajectory, res->base_solution);

    tmc_manipulation_types::JointState last_config;
    last_config.name = planning_request.use_joints;
    last_config.position = path.joint_trajectory.path.back();
    tmc_manipulation_types_bridge::JointStateToJointStateMsg(
        FetchAllJoints(robot_collision_detector_, last_config), res->joint_state_after_planning);

    robot_collision_detector_->SetRobotNamedAngle(last_config);
    robot_collision_detector_->SetRobotTransform(path.multi_dof_joint_trajectory.path.back()[0]);
    // FetchCollisionEnvironment(
    //     last_config,
    //     last_origin_to_base,
    //     planning_request.attached_objects,
    //     robot_collision_detector_,
    //     req->environment_before_planning,
    //     res.environment_after_planning);

    if (request_logger_) {
      if (!request_logger_->SaveMessage(".response", *res)) {
        RCLCPP_WARN(get_logger(), "Cannot open log file try next.");
      }
    }
  }
}

void RobotRrtPlannerNode::PlanWithJointGoals(
    const tmc_planning_msgs::srv::PlanWithJointGoals::Request::SharedPtr req,
    tmc_planning_msgs::srv::PlanWithJointGoals::Response::SharedPtr res) {
  const auto tsr_req = std::make_shared<tmc_planning_msgs::srv::PlanWithTsrConstraints::Request>();
  const auto tsr_res = std::make_shared<tmc_planning_msgs::srv::PlanWithTsrConstraints::Response>();

  // copy to tsr planning service
  tsr_req->origin_to_basejoint = req->origin_to_basejoint;
  tsr_req->initial_joint_state = req->initial_joint_state;
  tsr_req->base_movement_type = req->base_movement_type;
  tsr_req->use_joints = req->use_joints;
  tsr_req->weighted_joints = req->weighted_joints;
  tsr_req->weight = req->weight;
  tsr_req->goal_joint_states = req->goal_joint_states;
  tsr_req->goal_basejoint_to_bases = req->goal_basejoint_to_bases;
  tsr_req->constraint_tsrs = req->constraint_tsrs;
  tsr_req->attached_objects = req->attached_objects;
  // tsr_req->hint_trajectory = req->hint_trajectory;
  tsr_req->environment_before_planning = req->environment_before_planning;
  tsr_req->timeout = req->timeout;
  tsr_req->max_iteration = req->max_iteration;
  tsr_req->extra_constraints = req->extra_constraints;
  tsr_req->extra_goal_constraints = req->extra_goal_constraints;

  PlanWithTsrConstraints(tsr_req, tsr_res);

  // copy to tsr planning service
  res->solution = tsr_res->solution;
  res->base_solution = tsr_res->base_solution;
  // res->environment_after_planning = tsr_res->environment_after_planning;
  res->joint_state_after_planning = tsr_res->joint_state_after_planning;
  res->error_code = tsr_res->error_code;
}

/// Exercise plan with the target value of the Hand position
void RobotRrtPlannerNode::PlanWithHandGoals(
    const tmc_planning_msgs::srv::PlanWithHandGoals::Request::SharedPtr req,
    tmc_planning_msgs::srv::PlanWithHandGoals::Response::SharedPtr res) {
  const auto tsr_req = std::make_shared<tmc_planning_msgs::srv::PlanWithTsrConstraints::Request>();
  const auto tsr_res = std::make_shared<tmc_planning_msgs::srv::PlanWithTsrConstraints::Response>();

  // copy to tsr planning service
  tsr_req->origin_to_basejoint = req->origin_to_basejoint;
  tsr_req->initial_joint_state = req->initial_joint_state;
  tsr_req->base_movement_type = req->base_movement_type;
  tsr_req->use_joints = req->use_joints;
  tsr_req->weighted_joints = req->weighted_joints;
  tsr_req->weight = req->weight;
  tsr_req->probability_goal_generate = req->probability_goal_generate;
  tsr_req->constraint_tsrs = req->constraint_tsrs;
  tsr_req->attached_objects = req->attached_objects;
  // tsr_req->hint_trajectory = req->hint_trajectory;
  tsr_req->environment_before_planning = req->environment_before_planning;
  tsr_req->timeout = req->timeout;
  tsr_req->max_iteration = req->max_iteration;
  tsr_req->uniform_bound_sampling = req->uniform_bound_sampling;
  tsr_req->deviation_for_bound_sampling = req->deviation_for_bound_sampling;
  tsr_req->extra_constraints = req->extra_constraints;
  tsr_req->extra_goal_constraints = req->extra_goal_constraints;
  tsr_req->goal_no_ik_joint_state = req->goal_no_ik_joint_state;

  // Converts the target value to TSR
  for (const auto& hand_goal : req->origin_to_hand_goals) {
    tmc_planning_msgs::msg::TaskSpaceRegion hand_goal_tsr;
    hand_goal_tsr.end_frame_id = req->ref_frame_id;
    InitPoseMsg(hand_goal_tsr.tsr_to_end);
    hand_goal_tsr.origin_to_tsr = hand_goal;
    for (auto i = 0; i < 6; ++i) {
      hand_goal_tsr.min_bounds[i] = 0.0;
      hand_goal_tsr.max_bounds[i] = 0.0;
    }
    tsr_req->goal_tsrs.push_back(hand_goal_tsr);
  }

  PlanWithTsrConstraints(tsr_req, tsr_res);

  // copy to tsr planning service
  res->solution = tsr_res->solution;
  res->base_solution = tsr_res->base_solution;
  // res->environment_after_planning = tsr_res->environment_after_planning;
  res->joint_state_after_planning = tsr_res->joint_state_after_planning;
  res->error_code = tsr_res->error_code;
  res->origin_to_hand_after_planning = tf2::toMsg(robot_collision_detector_->GetObjectTransform(req->ref_frame_id));
}

/// Exercise plan with the target value of Hand straight
void RobotRrtPlannerNode::PlanWithHandLine(
    const tmc_planning_msgs::srv::PlanWithHandLine::Request::SharedPtr req,
    tmc_planning_msgs::srv::PlanWithHandLine::Response::SharedPtr res) {
  const auto tsr_req = std::make_shared<tmc_planning_msgs::srv::PlanWithTsrConstraints::Request>();
  const auto tsr_res = std::make_shared<tmc_planning_msgs::srv::PlanWithTsrConstraints::Response>();

  // copy to tsr planning service
  tsr_req->origin_to_basejoint = req->origin_to_basejoint;
  tsr_req->initial_joint_state = req->initial_joint_state;
  tsr_req->base_movement_type = req->base_movement_type;
  tsr_req->use_joints = req->use_joints;
  tsr_req->weighted_joints = req->weighted_joints;
  tsr_req->weight = req->weight;
  tsr_req->probability_goal_generate = req->probability_goal_generate;
  tsr_req->attached_objects = req->attached_objects;
  // tsr_req->hint_trajectory = req->hint_trajectory;
  tsr_req->environment_before_planning = req->environment_before_planning;
  tsr_req->timeout = req->timeout;
  tsr_req->max_iteration = req->max_iteration;
  tsr_req->uniform_bound_sampling = req->uniform_bound_sampling;
  tsr_req->deviation_for_bound_sampling = req->deviation_for_bound_sampling;
  tsr_req->extra_constraints = req->extra_constraints;
  tsr_req->extra_goal_constraints = req->extra_goal_constraints;
  tsr_req->goal_no_ik_joint_state = req->goal_no_ik_joint_state;

  Eigen::Affine3d robot_pose;
  tf2::fromMsg(req->origin_to_basejoint, robot_pose);
  robot_collision_detector_->SetRobotTransform(robot_pose);

  tmc_manipulation_types::JointState initial_config;
  tmc_manipulation_types_bridge::JointStateMsgToJointState(req->initial_joint_state, initial_config);
  robot_collision_detector_->SetRobotNamedAngle(initial_config);

  const auto origin_to_hand = robot_collision_detector_->GetObjectTransform(req->ref_frame_id);
  // Decide ORIGIN_TO_HAND so that REQ AXIS becomes an X -axis
  Eigen::Vector3d axis(req->axis.x, req->axis.y, req->axis.z);
  axis.normalize();

  // Separate processing depending on whether AXIS is local or global
  Eigen::Affine3d origin_to_tsr;
  Eigen::Affine3d tsr_to_hand;
  if (req->local_origin_of_axis) {
    const auto hand_to_axis = CalcPoseFromAxisX(axis);
    origin_to_tsr = origin_to_hand * hand_to_axis;
    tsr_to_hand = hand_to_axis.inverse();
  } else {
    origin_to_tsr = CalcPoseFromAxisX(axis);
    tsr_to_hand = origin_to_tsr.inverse() * origin_to_hand;
  }
  // Straight -line restraint
  tmc_manipulation_types::RegionValues min_constraint;
  tmc_manipulation_types::RegionValues max_constraint;
  if (req->goal_value > 0.0) {
    min_constraint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    max_constraint << req->goal_value, 0.0, 0.0, 0.0, 0.0, 0.0;
  } else {
    min_constraint << req->goal_value, 0.0, 0.0, 0.0, 0.0, 0.0;
    max_constraint << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  }
  tmc_manipulation_types::TaskSpaceRegion tsr_c(
      origin_to_tsr, tsr_to_hand, min_constraint, max_constraint, std::string("origin"), req->ref_frame_id);
  tmc_planning_msgs::msg::TaskSpaceRegion tsr_c_msg;
  tmc_manipulation_types_bridge::TaskSpaceRegionToTaskSpaceRegionMsg(tsr_c, tsr_c_msg);
  tsr_req->constraint_tsrs.push_back(tsr_c_msg);

  tmc_manipulation_types::RegionValues min_goal;
  tmc_manipulation_types::RegionValues max_goal;
  min_goal << req->goal_value, 0.0, 0.0, 0.0, 0.0, 0.0;
  max_goal << req->goal_value, 0.0, 0.0, 0.0, 0.0, 0.0;
  tmc_manipulation_types::TaskSpaceRegion tsr_g(
      origin_to_tsr, tsr_to_hand, min_goal, max_goal, std::string("origin"), req->ref_frame_id);
  tmc_planning_msgs::msg::TaskSpaceRegion tsr_g_msg;
  tmc_manipulation_types_bridge::TaskSpaceRegionToTaskSpaceRegionMsg(tsr_g, tsr_g_msg);
  tsr_req->goal_tsrs.push_back(tsr_g_msg);

  PlanWithTsrConstraints(tsr_req, tsr_res);

  // copy to tsr planning service
  res->solution = tsr_res->solution;
  res->base_solution = tsr_res->base_solution;
  // res->environment_after_planning = tsr_res->environment_after_planning;
  res->joint_state_after_planning = tsr_res->joint_state_after_planning;
  res->error_code = tsr_res->error_code;
  res->origin_to_hand_after_planning = tf2::toMsg(robot_collision_detector_->GetObjectTransform(req->ref_frame_id));
}

  // /// Take out the object posture
  // geometry_msgs::Pose RobotRrtPlannerNode::FetchFrame_(
  //     const tmc_manipulation_types::JointState& joint_state,
  //     const string& object_name) {
  //   robot_collision_detector_->SetRobotNamedAngle(joint_state);
  //   geometry_msgs::Pose origin_to_object;
  //   Affine3dToPoseMsg(
  //       robot_collision_detector_->GetObjectTransform(object_name),
  //       origin_to_object);
  //   return origin_to_object;
  // }


void RobotRrtPlannerNode::SetDebugCallBacks_(const std::vector<std::string>& joint_names) {
    // const AttachedObjectSeq& attached_objects,
    // const tmc_manipulation_msgs::CollisionEnvironment& environment) {
  planner_->set_check_feasibility_callback(
      std::bind(&RobotRrtPlannerNode::PublishJointStateAndAttachedObject_,
                this, std::placeholders::_1, std::placeholders::_2,
                step_mode_, std::string(""), joint_names));
          // boost::cref(attached_objects), boost::cref(environment)));
  planner_->set_add_start_callback(
      std::bind(&RobotRrtPlannerNode::PublishJointStateAndAttachedObject_,
                this, std::placeholders::_1, true,
                step_mode_, std::string("Add start:"), joint_names));
          // boost::cref(attached_objects), boost::cref(environment)));
  planner_->set_add_goal_callback(
      std::bind(&RobotRrtPlannerNode::PublishJointStateAndAttachedObject_,
                this, std::placeholders::_1, true,
                step_mode_, std::string("Add goal:"), joint_names));
          // boost::cref(attached_objects), boost::cref(environment)));
}

/// @brief For debugging
///  Publish of the given joint_state
///  Update the marker position of ATTACHED_OBJECT
///  Display interference PAIR on the console
void RobotRrtPlannerNode::PublishJointStateAndAttachedObject_(
    const Eigen::VectorXd& config,
    bool feasible,
    bool step,
    const std::string& debug_msg,
    const std::vector<std::string>& joint_names) {
    // const tmc_manipulation_types::AttachedObjectSeq& attached_objects,
    // const tmc_manipulation_msgs::CollisionEnvironment& environment) {
  if (publish_debug_info_) {
    tmc_manipulation_types::JointState partial_joint_state = {joint_names, config.head(joint_names.size())};
    robot_collision_detector_->SetRobotNamedAngle(partial_joint_state);

    sensor_msgs::msg::JointState joint_state_msg;
    tmc_manipulation_types_bridge::JointStateToJointStateMsg(
        robot_collision_detector_->GetRobotNamedAngle(), joint_state_msg);

    joint_state_msg.header.stamp = this->now();
    debug_joint_state_pub_->publish(joint_state_msg);

    auto origin_to_base = tf2::eigenToTransform(robot_collision_detector_->GetRobotTransform());
    origin_to_base.header.stamp = this->now();
    origin_to_base.header.frame_id = "origin";
    origin_to_base.child_frame_id = "base_link";
    debug_tf_broadcaster_->sendTransform(origin_to_base);
  }
  if (!debug_msg.empty()) {
    RCLCPP_INFO(get_logger(), "[planner_node]:%s", debug_msg.c_str());
  }

  // if (publish_debug_info_) {
  //   tmc_manipulation_msgs::CollisionEnvironment current_collision_environment;
  //   FetchCollisionEnvironment(
  //       robot_collision_detector_->GetRobotNamedAngle(),
  //       robot_collision_detector_->GetRobotTransform(),
  //       attached_objects,
  //       robot_collision_detector_,
  //       environment,
  //       current_collision_environment);
  //   geometry_msgs::Pose origin_to_robot;
  //   Affine3dToPoseMsg(robot_collision_detector_->GetRobotTransform(),
  //                     origin_to_robot);
  //   PublishEnvironmentDebug(current_collision_environment,
  //                           debug_environment_pub_);
  // }

  if (!feasible) {
    if (!planner_->limit_joint().empty()) {
      RCLCPP_INFO(get_logger(), "[planner_node]: limit = %s", planner_->limit_joint().c_str());
    }
    if (!planner_->last_contact_pair().empty()) {
      RCLCPP_INFO(get_logger(), "[planner_node]: contact pair = %s vs %s",
                  planner_->last_contact_pair().at(0).first.c_str(),
                  planner_->last_contact_pair().at(0).second.c_str());
    }
  }
  if (step) {
    RCLCPP_INFO(get_logger(), "Planner Paused. Hit any key!");
    getchar();
    RCLCPP_INFO(get_logger(), "Continue!");
  }
}

}  // namespace tmc_robot_rrt_planner_node
