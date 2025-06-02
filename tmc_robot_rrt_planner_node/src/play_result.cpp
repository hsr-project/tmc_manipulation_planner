/*
Copyright (c) 2025 TOYOTA MOTOR CORPORATION
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
///
/// pray_result.cpp - play planned trajectory
///
///

#include <string>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <tmc_eigen_bridge/eigen_bridge.hpp>
#include <tmc_planning_msgs/AttachedObject.h>
#include <tmc_planning_msgs/PlanWithTsrConstraints.h>
#include <tmc_utils/MsgIO.hpp>
#include "debug_utils.hpp"

using tmc_robot_collision_detector::RobotCollisionDetector;
using tmc_manipulation_types_bridge::AttachedObjectMsgToAttachedObject;
using tmc_manipulation_types_bridge::JointTrajectoryMsgToJointTrajectory;
using tmc_manipulation_types_bridge::JointStateToJointStateMsg;
using tmc_manipulation_types_bridge::JointStateMsgToJointState;
using tmc_manipulation_types_bridge::CollisionEnvironmentToOuterObjectSeq;
using tmc_manipulation_types_bridge::CollisionEnvironmentToCuboidSeq;
using tmc_manipulation_types::OuterObjectParametersSeq;
using tmc_eigen_bridge::Affine3dToPoseMsg;
using tmc_manipulation_types::AttachedObjectSeq;
using tmc_manipulation_types::Path;
using tmc_manipulation_types::JointState;
using std::vector;
using std::string;

namespace {

const double kDefaultWaitTime = 0.2;

/// @brief For debugging
///  Publish given joint_state
///  Update marker position of attached_object
///  Display interference pair on console
void PublishTrajectoryAndEvironment(
    bool step,
    double wait_time,
    const trajectory_msgs::JointTrajectory& joint_trajectory_msg,
    const trajectory_msgs::MultiDOFJointTrajectory& base_trajectory_msg,
    const geometry_msgs::Pose& robot_pose_msg,
    const vector<tmc_planning_msgs::AttachedObject>& attached_objects_msg,
    const tmc_manipulation_msgs::CollisionEnvironment& environment_msg,
    const RobotCollisionDetector::Ptr& robot_collision_detector,
    ros::Publisher& joint_state_pub,
    ros::Publisher& environment_pub) {

  tf::TransformBroadcaster tf_broad;

  Eigen::Affine3d robot_pose;
  tf::poseMsgToEigen(robot_pose_msg,
                     robot_pose);
  // Set robot position and orientation
  robot_collision_detector->SetRobotTransform(robot_pose);
  /// Known objects for interference check
  tmc_manipulation_types::OuterObjectParametersSeq known_objects;
  /// Environment for interference check
  tmc_manipulation_types::CuboidSeq collision_map;
  CollisionEnvironmentToOuterObjectSeq(
      environment_msg,
      known_objects);
  CollisionEnvironmentToCuboidSeq(
      environment_msg,
      "cuboid",
      collision_map);
  // Remove all objects
  robot_collision_detector->DestroyAllOuterObject();
  // Create objects
  for (OuterObjectParametersSeq::const_iterator object = known_objects.begin();
       object != known_objects.end();
       ++object) {
    robot_collision_detector->CreateOuterObject(*object);
  }
  // Add environment using voxel
  robot_collision_detector->CreateCuboids(collision_map, false);
  tmc_manipulation_types::JointTrajectory joint_trajectory;
  JointTrajectoryMsgToJointTrajectory(
      joint_trajectory_msg,
      joint_trajectory);

  AttachedObjectSeq attached_objects;
  tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut<
    tmc_planning_msgs::AttachedObject,
    tmc_manipulation_types::AttachedObject>(
        attached_objects_msg,
        attached_objects,
        AttachedObjectMsgToAttachedObject);

  // Set attached objects
  for (AttachedObjectSeq::const_iterator attached_object =
           attached_objects.begin();
       attached_object != attached_objects.end();
       ++attached_object) {
    robot_collision_detector->HoldObject(attached_object->object_id,
                                         attached_object->frame_name,
                                         attached_object->frame_to_object,
                                         attached_object->group_id);
    for (std::vector<std::string>::const_iterator expected_object =
             attached_object->expected_objects.begin();
         expected_object != attached_object->expected_objects.end();
         ++expected_object) {
      robot_collision_detector->DisableCollisionCheckObjectToObject(
          attached_object->object_id,
          *expected_object);
    }
  }

  vector<string> joint_names = joint_trajectory_msg.joint_names;
  for (uint32_t i = 0; i< joint_trajectory.path.size(); ++i) {
    // send joint states
    JointState partial_joint_state
        = {joint_names, joint_trajectory.path[i]};
    robot_collision_detector->SetRobotNamedAngle(partial_joint_state);
    sensor_msgs::JointState joint_state_msg;
    JointStateToJointStateMsg(
        robot_collision_detector->GetRobotNamedAngle(), joint_state_msg);
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_pub.publish(joint_state_msg);

    // move base
    tf::Transform origin_to_base;
    transformMsgToTF(
        base_trajectory_msg.points[i].transforms[0],
        origin_to_base);
    tf_broad.sendTransform(
        tf::StampedTransform(origin_to_base,
                             ros::Time::now(),
                             "origin",
                             "base_link"));
    Eigen::Affine3d origin_to_base_eigen;
    tf::transformTFToEigen(origin_to_base, origin_to_base_eigen);

    // get collision objects
    tmc_manipulation_msgs::CollisionEnvironment current_collision_environment;
    tmc_robot_rrt_planner_node::FetchCollisionEnvironment(
        robot_collision_detector->GetRobotNamedAngle(),
        origin_to_base_eigen,
        attached_objects,
        robot_collision_detector,
        environment_msg,
        current_collision_environment);

    tmc_robot_rrt_planner_node::PublishEnvironmentDebug(
        current_collision_environment,
        environment_pub);
    if (step) {
      ROS_INFO("Player Paused. Hit any key!");
      getchar();
      ROS_INFO("Continue!");
    } else {
      ros::Duration(wait_time).sleep();
    }
  }
}
// end of namespace
}  // anonymous namespace

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "plan_with_joint_from_file");
  if ((argc != 2) && (argc != 3)) {
    ROS_ERROR("usage: %s plan_xxx", argv[0]);
    ROS_ERROR("usage: %s plan_xxx.reqeust plan_xxx.response", argv[0]);
    exit(EXIT_FAILURE);
  }
  ros::NodeHandle node;
  ros::NodeHandle local_node("~");
  ros::Publisher joint_state_pub =
      node.advertise<sensor_msgs::JointState>(
          "debug_joint_state",
          100);
  ros::Publisher environment_pub =
      node.advertise<visualization_msgs::MarkerArray>(
          "debug_environment",
          100);

  string robot_model("");
  // Get robot model path from parameter server
  if (!node.getParam("robot_description", robot_model)) {
    ROS_FATAL("cannot get paramter robot_description_file");
    exit(EXIT_FAILURE);
  }
  string robot_collision_pair("");
  // Get interference settings file path from parameter server
  if (!node.getParam("robot_collision_pair", robot_collision_pair)) {
    ROS_FATAL("cannot get paramter robot_collision_pair");
    exit(EXIT_FAILURE);
  }

  string collision_engine;
  node.param("collision_engine", collision_engine, string("ODE"));

  bool step_mode;
  local_node.param("step_mode", step_mode, false);

  double wait_time;
  local_node.param("wait_time", wait_time, kDefaultWaitTime);

  RobotCollisionDetector::Ptr robot_collision_detector;
  robot_collision_detector.reset(
      new RobotCollisionDetector(robot_model,
                                 robot_collision_pair,
                                 collision_engine));

  // Load msg
  trajectory_msgs::JointTrajectory trajectory;
  tmc_planning_msgs::PlanWithTsrConstraints plan;

  if (argc == 2) {
    if (!tmc_utils::LoadMsg(std::string(argv[1]) + ".request", plan.request)) {
      ROS_ERROR("Fail to load %s.request", argv[1]);
      exit(EXIT_FAILURE);
    }

    if (!tmc_utils::LoadMsg(std::string(argv[1]) + ".response", plan.response)) {
      ROS_ERROR("Fail to load %s.response", argv[1]);
      exit(EXIT_FAILURE);
    }
  } else {
    if (!tmc_utils::LoadMsg(std::string(argv[1]), plan.request)) {
      ROS_ERROR("Fail to load %s", argv[1]);
      exit(EXIT_FAILURE);
    }

    if (!tmc_utils::LoadMsg(std::string(argv[2]), plan.response)) {
      ROS_ERROR("Fail to load %s", argv[2]);
      exit(EXIT_FAILURE);
    }
  }

  ROS_INFO_STREAM("Reqeust = \n" << plan.request);

  ROS_INFO_STREAM("Result = \n" << plan.response.solution);
  ROS_INFO_STREAM("Result_base = \n" << plan.response.base_solution);

  tmc_manipulation_types::JointState initial_joint_state;
  JointStateMsgToJointState(
      plan.request.initial_joint_state,
      initial_joint_state);
  robot_collision_detector->SetRobotNamedAngle(initial_joint_state);


  PublishTrajectoryAndEvironment(
      step_mode,
      wait_time,
      plan.response.solution,
      plan.response.base_solution,
      plan.request.origin_to_basejoint,
      plan.request.attached_objects,
      plan.request.environment_before_planning,
      robot_collision_detector,
      joint_state_pub,
      environment_pub);

  ros::Duration(1.0).sleep();
  return(EXIT_SUCCESS);
}
