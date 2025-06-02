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
/// debug_utils.hpp - debug utilities for planner
///
///

#ifndef TMC_ROBOT_RRT_PLANNER_NODE_DEBUG_UTILS_HPP_
#define TMC_ROBOT_RRT_PLANNER_NODE_DEBUG_UTILS_HPP_

#include <cstdlib>
#include <string>
#include <vector>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>
#include <tmc_eigen_bridge/eigen_bridge.hpp>
#include <tmc_manipulation_types_bridge/manipulation_msg_convertor.hpp>
#include <tmc_robot_collision_detector/robot_collision_detector.hpp>
#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_planner/robot_cbirrt_planner.hpp>
#include "debug_utils.hpp"

using tmc_manipulation_types::JointState;
using tmc_manipulation_types::AttachedObject;
using tmc_manipulation_types::AttachedObjectSeq;
using tmc_manipulation_types::TaskSpaceRegion;
using tmc_manipulation_types::JointTrajectory;
using tmc_manipulation_types::RegionValues;
using tmc_robot_kinematics_model::IRobotKinematicsModel;
using tmc_robot_kinematics_model::IKSolver;
using tmc_robot_kinematics_model::IKRequest;
using tmc_robot_kinematics_model::IKResult;
using tmc_robot_kinematics_model::NumericIKSolver;
using tmc_robot_collision_detector::RobotCollisionDetector;
using tmc_robot_planner::CBiRrtRequest;
using tmc_robot_planner::RobotCBiRrtPlanner;
using tmc_manipulation_types_bridge::JointPositionMsgToConfig;
using tmc_manipulation_types_bridge::JointStateMsgToJointState;
using tmc_manipulation_types_bridge::JointStateToJointStateMsg;
using tmc_manipulation_types_bridge::TaskSpaceRegionMsgToTaskSpaceRegion;
using tmc_manipulation_types_bridge::AttachedObjectMsgToAttachedObject;
using tmc_manipulation_types_bridge::JointTrajectoryToJointTrajectoryMsg;
using tmc_manipulation_types_bridge::CollisionEnvironmentToOuterObjectSeq;
using tmc_manipulation_types_bridge::CollisionEnvironmentToCuboidSeq;
using tmc_manipulation_types_bridge::ConvertSequence;
using tmc_manipulation_types_bridge::ConvertSequenceWithEigenOut;
using tmc_manipulation_types_bridge::ConvertSequenceWithEigenIn;
using tmc_manipulation_types_bridge::ObjectIdentifierMsgToObjectName;
using XmlRpc::XmlRpcValue;
using std::vector;
using std::string;

namespace {

// Marker namespace name for known_object
const char* const kKnownObjectNS = "planner_debug/known_objects";
// Marker namespace name for collision_map
const char* const kCollisionMapNS = "planner_debug/collision_map";
// FrameID for debugging
const char* const kDebugFrameId = "origin";
// Number of colors for debugging
const int32_t kNumColor = 100;

geometry_msgs::Pose MultiPoseMsg(
    const geometry_msgs::Pose& pose1,
    const geometry_msgs::Pose& pose2) {
  tf::Pose tf_pose1;
  tf::poseMsgToTF(pose1, tf_pose1);
  tf::Pose tf_pose2;
  tf::poseMsgToTF(pose2, tf_pose2);
  tf::Pose tf_pose_ret = tf_pose1 * tf_pose2;
  geometry_msgs::Pose pose_ret;
  tf::poseTFToMsg(tf_pose_ret, pose_ret);
  return pose_ret;
}

geometry_msgs::Pose InvertPoseMsg(
    const geometry_msgs::Pose& pose) {
  tf::Pose tf_pose;
  geometry_msgs::Pose pose_ret;
  tf::poseMsgToTF(pose, tf_pose);
  tf::poseTFToMsg(tf_pose.inverse(), pose_ret);
  return pose_ret;
}

/// Clear markers temporarily
/// @param [in] delete_num
/// @param [in] marker_namespace Namespace of the markers to clear
/// @param [in/out] marker_pub Marker publisher
void DeleteMarkers(uint32_t delete_num,
                   const string& marker_namespace,
                   ros::Publisher& marker_pub) {
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = string(kDebugFrameId);
  marker.ns = marker_namespace;
  marker.action = visualization_msgs::Marker::DELETE;
  visualization_msgs::MarkerArray marker_array;
  for (uint32_t i = 0; i < delete_num; ++i) {
    marker.id = i;
    marker_array.markers.push_back(marker);
  }
  marker_pub.publish(marker_array);
}

// end no-name namespace
}  // anonymous namespace


namespace tmc_robot_rrt_planner_node {

visualization_msgs::MarkerArray ObjectToMarker(
    const vector<tmc_manipulation_msgs::CollisionObject>& objects,
    const vector<geometry_msgs::Pose>& origin_to_object) {
  visualization_msgs::MarkerArray object_marker;
  vector<std_msgs::ColorRGBA> colors;
  /// Prepare colors for visualization markers
  uint32_t seed(0);
  double rand_max = static_cast<double>(RAND_MAX);
  colors.resize(kNumColor);
  for (int32_t i = 0; i < kNumColor; ++i) {
    colors[i].r = static_cast<double>(rand_r(&seed))/rand_max;
    colors[i].g = static_cast<double>(rand_r(&seed))/rand_max;
    colors[i].b = static_cast<double>(rand_r(&seed))/rand_max;
    colors[i].a = 1.0;
  }

  int32_t marker_id = 0;
  vector<tmc_manipulation_msgs::CollisionObject>::const_iterator object;
  for (object = objects.begin(); object != objects.end();  ++object) {
    int32_t num_object = std::distance(objects.begin(), object);
    vector<tmc_geometric_shapes_msgs::Shape>::const_iterator shape;
    std_msgs::ColorRGBA color = colors[rand_r(&seed) % kNumColor];
    for (shape = object->shapes.begin();
         shape != object->shapes.end();
         ++shape) {
      // Processing according to shape
      visualization_msgs::Marker marker;
      switch (shape->type) {
        case tmc_geometric_shapes_msgs::Shape::SPHERE: {
          // Create sphere marker
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.scale.x = shape->dimensions[0] * 2.0;
          marker.scale.y = shape->dimensions[0] * 2.0;
          marker.scale.z = shape->dimensions[0] * 2.0;
          break;
        }
        case tmc_geometric_shapes_msgs::Shape::BOX: {
          // Create box marker
          marker.type = visualization_msgs::Marker::CUBE;
          marker.scale.x = shape->dimensions[0];
          marker.scale.y = shape->dimensions[1];
          marker.scale.z = shape->dimensions[2];
          break;
        }
        case tmc_geometric_shapes_msgs::Shape::CYLINDER: {
          // Create cylinder marker
          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.scale.x = shape->dimensions[0] * 2.0;
          marker.scale.y = shape->dimensions[0] * 2.0;
          marker.scale.z = shape->dimensions[1];
          break;
        }
        case tmc_geometric_shapes_msgs::Shape::CAPSULE: {
          // Create capsule marker (simulated by a cylinder)
          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.scale.x = shape->dimensions[0] * 2.0;
          marker.scale.y = shape->dimensions[0] * 2.0;
          marker.scale.z = shape->dimensions[1] + shape->dimensions[0] * 2.0;
          break;
        }
        case tmc_geometric_shapes_msgs::Shape::MESH: {
          // Create mesh marker
          // Only supports loading mesh files
          marker.type = visualization_msgs::Marker::MESH_RESOURCE;
          marker.scale.x = 1.0;
          marker.scale.y = 1.0;
          marker.scale.z = 1.0;
          if ((shape->stl_file_name.find("package://") == 0) ||
              (shape->stl_file_name.find("file://") == 0) ||
              (shape->stl_file_name.find("http://") == 0)) {
            marker.mesh_resource = shape->stl_file_name;
          } else {
            marker.mesh_resource = std::string("file://")
                                 + shape->stl_file_name;
          }
          break;
        }
        default: {
          break;
        }
      }
      // Common processing
      marker.header.stamp = ros::Time::now();
      marker.header.frame_id = string(kDebugFrameId);
      marker.id = marker_id++;
      marker.lifetime = ros::Duration(0);
      marker.ns = kKnownObjectNS;
      int32_t num_pose = std::distance(object->shapes.begin(), shape);
      marker.pose = MultiPoseMsg(origin_to_object[num_object],
                                 object->poses[num_pose]);
      marker.action = visualization_msgs::Marker::ADD;
      marker.color = color;
      object_marker.markers.push_back(marker);
    }
  }
  return object_marker;
}

/// Convert CollisionMap to visualization markers
/// @param [in] collision_map Object of CollisionMap type
/// @param [in] origin_to_robot Position and orientation of the robot from the reference frame
/// @param [in] origin_to_map Position and orientation of the CollisionMap from the reference frame
visualization_msgs::MarkerArray MapToMarker(
    const tmc_mapping_msgs::CollisionMap& collision_map,
    const geometry_msgs::Pose& origin_to_map) {
  visualization_msgs::MarkerArray map_markers;
  // Add collision_map marker
  vector<tmc_geometry_msgs::OrientedBoundingBox>::const_iterator bit;
  int32_t markerid = 0;
  for (bit = collision_map.boxes.begin();
       bit != collision_map.boxes.end(); ++bit) {
    // Create box marker
    visualization_msgs::Marker marker;
    geometry_msgs::Pose map_to_marker;
    map_to_marker.position.x = bit->center.x;
    map_to_marker.position.y = bit->center.y;
    map_to_marker.position.z = bit->center.z;
    // Assumption that there is no tilt in the box
    map_to_marker.orientation.x = 0.0;
    map_to_marker.orientation.y = 0.0;
    map_to_marker.orientation.z = 0.0;
    map_to_marker.orientation.w = 1.0;

    geometry_msgs::Pose origin_to_marker =
        MultiPoseMsg(origin_to_map, map_to_marker);

    marker.header = collision_map.header;
    marker.header.frame_id = string(kDebugFrameId);
    marker.ns = kCollisionMapNS;
    marker.id = markerid++;
    marker.pose = origin_to_marker;
    marker.lifetime = ros::Duration(0);
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = bit->extents.x;
    marker.scale.y = bit->extents.y;
    marker.scale.z = bit->extents.z;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 0.5;
    map_markers.markers.push_back(marker);
  }
  return map_markers;
}

/// Publish debug collision environment
/// @param [in] collision_environment Planning environment
/// @param [in] delete_before_publish Whether to clear markers before publishing
/// @param [in/out] marker_pub publisher
void PublishEnvironmentDebug(
    const tmc_manipulation_msgs::CollisionEnvironment& environment,
    bool delete_before_publish,
    ros::Publisher& marker_pub) {
  static uint32_t num_object_marker = 0;
  static uint32_t num_map_marker = 0;
  if (delete_before_publish) {
    DeleteMarkers(num_map_marker, kCollisionMapNS, marker_pub);
    DeleteMarkers(num_object_marker, kKnownObjectNS, marker_pub);
  }
  visualization_msgs::MarkerArray map_marker = MapToMarker(
      environment.collision_map,
      environment.collision_map_pose);
  visualization_msgs::MarkerArray object_marker = ObjectToMarker(
      environment.known_objects,
      environment.poses);
  marker_pub.publish(map_marker);
  marker_pub.publish(object_marker);
  num_object_marker = object_marker.markers.size();
  num_map_marker = map_marker.markers.size();
}

/// @brief For debugging
/// Publish collision environment
/// @param [in] collision_environment Planning environment
/// @param [in/out] marker_pub publisher
void PublishEnvironmentDebug(
    const tmc_manipulation_msgs::CollisionEnvironment& environment,
    ros::Publisher& marker_pub) {
  PublishEnvironmentDebug(
      environment,
      false,
      marker_pub);
}

/// @brief Extract CollisionEnvironment for the given joint_state
///        Modify the changes in attached_objects from before_collision_environment
/// @param [in] joint_state Joint angles
/// @param [in] origin_to_base Location of the base
/// @param [in] attached_objects Grasped objects
/// @param [in] environment Initial environment
/// @param [in] robot_collision_detector
/// @param [out] environment_out Changed environment in collision_environment
void FetchCollisionEnvironment(
    const tmc_manipulation_types::JointState& joint_state,
    const Eigen::Affine3d& origin_to_base,
    const tmc_manipulation_types::AttachedObjectSeq& attached_objects,
    const RobotCollisionDetector::Ptr& robot_collision_detector,
    const tmc_manipulation_msgs::CollisionEnvironment& environment,
    tmc_manipulation_msgs::CollisionEnvironment& environment_out) {
  robot_collision_detector->SetRobotNamedAngle(joint_state);
  robot_collision_detector->SetRobotTransform(origin_to_base);
  environment_out = environment;
  AttachedObjectSeq::const_iterator attached_object;
  for (attached_object = attached_objects.begin();
       attached_object != attached_objects.end();
       ++attached_object) {
    for (uint32_t i = 0; i < environment_out.known_objects.size(); ++i) {
      string object_name;
      ObjectIdentifierMsgToObjectName(environment_out.known_objects[i].id,
                                      object_name);
      // If found in the environment, update its position
      if (object_name == attached_object->object_id) {
        Eigen::Affine3d attached_object_pose =
            robot_collision_detector->GetObjectTransform(object_name);
        tf::poseEigenToMsg(attached_object_pose,
                           environment_out.poses[i]);
      }
    }
  }
}

}  // namespace tmc_robot_rrt_planner_node

#endif  // TMC_ROBOT_RRT_PLANNER_NODE_DEBUG_UTILS_HPP_
