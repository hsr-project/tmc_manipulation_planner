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

#ifndef TMC_ROBOT_RRT_PLANNER_NODE_ROBOT_RRT_PLANNER_NODE_HPP_
#define TMC_ROBOT_RRT_PLANNER_NODE_ROBOT_RRT_PLANNER_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <tmc_planning_msgs/srv/plan_with_hand_goals.hpp>
#include <tmc_planning_msgs/srv/plan_with_hand_line.hpp>
#include <tmc_planning_msgs/srv/plan_with_joint_goals.hpp>
#include <tmc_planning_msgs/srv/plan_with_tsr_constraints.hpp>
#include <tmc_robot_planner/robot_cbirrt_planner.hpp>
#include <tmc_utils/msg_io.hpp>

namespace tmc_robot_rrt_planner_node {

/// Robot planner node using RRT
class RobotRrtPlannerNode : public rclcpp::Node {
 public:
  /// Constructor
  RobotRrtPlannerNode() : RobotRrtPlannerNode(rclcpp::NodeOptions()) {}
  explicit RobotRrtPlannerNode(const rclcpp::NodeOptions& options);
  /// Destructor
  virtual ~RobotRrtPlannerNode() = default;

  /// Separate constructor and INIT to use Shared_from_this
  bool Init();

 private:
  // /// Take out all joint angles
  // tmc_manipulation_types::JointState FetchAllJoints_(
  //     const tmc_manipulation_types::JointState& partial_joint_state);

  // geometry_msgs::Pose FetchFrame_(
  //     const tmc_manipulation_types::JointState& joint_state,
  //     const std::string& object_id);
  // geometry_msgs::Pose FetchFrame_(
  //     const std::string& object_id);
  /// PLANWITHTSRCONSTRAINT callback
  void PlanWithTsrConstraints(
      const tmc_planning_msgs::srv::PlanWithTsrConstraints::Request::SharedPtr req,
      tmc_planning_msgs::srv::PlanWithTsrConstraints::Response::SharedPtr res);
  /// PlanwithJointGoals callback
  void PlanWithJointGoals(
      const tmc_planning_msgs::srv::PlanWithJointGoals::Request::SharedPtr req,
      tmc_planning_msgs::srv::PlanWithJointGoals::Response::SharedPtr res);
  /// PlanwithHandposes callback
  void PlanWithHandGoals(
      const tmc_planning_msgs::srv::PlanWithHandGoals::Request::SharedPtr req,
      tmc_planning_msgs::srv::PlanWithHandGoals::Response::SharedPtr res);
  /// PlanwithHandline callback
  void PlanWithHandLine(
      const tmc_planning_msgs::srv::PlanWithHandLine::Request::SharedPtr req,
      tmc_planning_msgs::srv::PlanWithHandLine::Response::SharedPtr res);

  /// Reading PLUGIN to restrain the joints
  pluginlib::ClassLoader<tmc_robot_planner::IConfigurationConstraint> constraint_plugin_loader_;
  /// Plugin that restrains the loaded joint
  std::map<std::string, tmc_robot_planner::IConfigurationConstraint::Ptr> constraint_plugin_cache_;
  /// Reading of reverse athletic plugin
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IKSolver> ik_plugin_loader_;
  /// Reading in order of motor athletic plugin
  pluginlib::ClassLoader<tmc_robot_kinematics_model::IRobotKinematicsModel> fk_loader_;

  /// Exploration
  double delta_;
  /// Interference check width Delta _> = Sub_delta_
  double sub_delta_;
  /// Maximum value in the parallel direction of Base [M]
  double base_translation_max_;

  double increase_sampling_deviation_;
  double step_sampling_deviation_;

  /// Class to save Request
  tmc_utils::MessageLogger::Ptr request_logger_;
  /// Flag for Publish for Debug
  bool publish_debug_info_;
  /// Enable step execution mode
  bool step_mode_;

  /// Set a callback for debugging
  void SetDebugCallBacks_(const std::vector<std::string>& joint_names);
      // const tmc_manipulation_types::AttachedObjectSeq& attached_objects,
      // const tmc_manipulation_msgs::CollisionEnvironment& environment);
  void PublishJointStateAndAttachedObject_(const Eigen::VectorXd& config, bool feasible, bool step,
                                           const std::string& debug_msg, const std::vector<std::string>& joint_names);
      // const tmc_manipulation_types::AttachedObjectSeq& attached_objects,
      // const tmc_manipulation_msgs::CollisionEnvironment& environment);

  /// The joint name that weighs with the planna
  std::vector<std::string> weight_names_;
  /// Planner weight
  std::vector<double> weights_;
  /// The joint name that weighs with IK
  std::vector<std::string> ik_weight_names_;
  /// IK weight
  std::vector<double> ik_weights_;
  /// Parallel weight
  double weight_linear_base_;
  /// Rotation direction weight
  double weight_rotational_base_;
  /// IK weight in parallel direction
  double weight_linear_base_ik_;
  /// IK weight in the direction of rotation
  double weight_rotational_base_ik_;

  /// ROBOT interference checker
  tmc_robot_collision_detector::RobotCollisionDetector::Ptr robot_collision_detector_;
  /// CBIRRT2 planna
  tmc_robot_planner::RobotCBiRrtPlanner::Ptr planner_;

  /// Joint_state_publisher for debugging
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr debug_joint_state_pub_;
  // /// Marker Publisher for debugging
  // ros::Publisher debug_environment_pub_;
  // /// Pose Publisher for debugging
  // ros::Publisher debug_pose_pub_;
  /// TF for debugging
  std::unique_ptr<tf2_ros::TransformBroadcaster> debug_tf_broadcaster_;

  /// Planwithtsrconstraints service
  rclcpp::Service<tmc_planning_msgs::srv::PlanWithTsrConstraints>::SharedPtr plan_with_constraints_service_;
  /// PlanwithJointGoals service
  rclcpp::Service<tmc_planning_msgs::srv::PlanWithJointGoals>::SharedPtr plan_with_joints_service_;
  /// PlanwithHandgoals service
  rclcpp::Service<tmc_planning_msgs::srv::PlanWithHandGoals>::SharedPtr plan_with_hand_service_;
  /// PlanwithHandline service
  rclcpp::Service<tmc_planning_msgs::srv::PlanWithHandLine>::SharedPtr plan_with_line_service_;
};

}  // namespace tmc_robot_rrt_planner_node

#endif  // TMC_ROBOT_RRT_PLANNER_NODE_ROBOT_RRT_PLANNER_NODE_HPP_
