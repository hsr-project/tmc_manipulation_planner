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

#include <gtest/gtest.h>

#include <geometric_shapes/mesh_operations.h>
#include <rclcpp/rclcpp.hpp>

#include <tmc_manipulation_tests/configs.hpp>

#include "../src/robot_rrt_planner_node.hpp"

// Because Pinocchio makes an error "You Should Include Pinocchio Before The Boost Headers"
#include <geometric_shapes/shape_operations.h>  // NOLINT

namespace {
int32_t const kServiceTimeout = 5;

const char* const kPlanWithHandGoals = "/plan_with_hand_goals";
const char* const kPlanWithHandLine = "/plan_with_hand_line";
const char* const kPlanWithJointGoals = "/plan_with_joint_goals";

const std::vector<std::string> kUseJoints = {
    "CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R", "CARM/SHOULDER_P",
    "CARM/ELBOW_P", "CARM/WRIST_Y", "CARM/WRIST_R", "CARM/WRIST_P"};

void SpinSomeThread(const rclcpp::Node::SharedPtr node, std::function<bool()> interrupt) {
  while (!interrupt()) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template<typename ServiceT>
typename rclcpp::Client<ServiceT>::SharedPtr CreateClient(const rclcpp::Node::SharedPtr& node,
                                                          const std::string& name) {
  const auto client = node->create_client<ServiceT>(name);
  if (client->wait_for_service(std::chrono::seconds(1))) {
    return client;
  } else {
    return nullptr;
  }
}

geometry_msgs::msg::Pose CreateInitialPose() {
  geometry_msgs::msg::Pose pose_out;
  pose_out.orientation.w = 1.0;
  return pose_out;
}

sensor_msgs::msg::JointState CreateInitialJointState() {
  sensor_msgs::msg::JointState joint_state_out;
  joint_state_out.name = {"CARM/LINEAR", "CARM/SHOULDER_Y", "CARM/SHOULDER_R", "CARM/SHOULDER_P", "CARM/ELBOW_P",
                          "CARM/WRIST_Y", "CARM/WRIST_R", "CARM/WRIST_P", "CARM/HEAD/NECK_Y", "CARM/HEAD/NECK_P",
                          "CARM/HAND/JOINT_11", "CARM/HAND/JOINT_12", "CARM/HAND/JOINT_21", "CARM/HAND/JOINT_22",
                          "CARM/LINEAR_PASSIVE", "BASE/_BASE_X"};
  joint_state_out.position = {0.0, 0.0, 0.0, 0.0, 3.14, 1.57, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0, 1.0, -1.0, 0.0, 0.0};
  joint_state_out.velocity.resize(joint_state_out.position.size(), 0.0);
  joint_state_out.effort.resize(joint_state_out.position.size(), 0.0);
  return joint_state_out;
}

/*
/// @param[out] Map: Collision inspection map
void CreateCollisionMap(
    tmc_mapping_msgs::CollisionMap& collision_map_out) {
  for (int32_t i = 0; i < 10; ++i) {
    tmc_geometry_msgs::OrientedBoundingBox box;
    box.center.x = 1.0;
    box.center.y = 0.0;
    box.center.z = 0.1 * i;
    box.extents.x = 0.1;
    box.extents.y = 0.1;
    box.extents.z = 0.1;
    box.axis.x = 1.0;
    box.axis.y = 0.0;
    box.axis.z = 0.0;
    box.angle = 0.0;
    collision_map_out.boxes.push_back(box);
  }
}
*/

void CreateKnownObjects(
    moveit_msgs::msg::PlanningSceneWorld& environment_out) {
  environment_out.collision_objects.resize(2);
  environment_out.collision_objects[0].id = "ball";
  environment_out.collision_objects[0].primitives.resize(1);
  environment_out.collision_objects[0].primitives[0].type
      = shape_msgs::msg::SolidPrimitive::SPHERE;
  environment_out.collision_objects[0].primitives[0].dimensions.resize(1);
  environment_out.collision_objects[0].primitives[0].dimensions[0] = 0.2;
  environment_out.collision_objects[0].primitive_poses.resize(1);
  environment_out.collision_objects[0].primitive_poses[0].position.x = 0.0;
  environment_out.collision_objects[0].primitive_poses[0].position.y = 0.0;
  environment_out.collision_objects[0].primitive_poses[0].position.z = 0.0;
  environment_out.collision_objects[0].primitive_poses[0].orientation.x = 0.0;
  environment_out.collision_objects[0].primitive_poses[0].orientation.y = 0.0;
  environment_out.collision_objects[0].primitive_poses[0].orientation.z = 0.0;
  environment_out.collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  environment_out.collision_objects[0].pose.position.x = 0.8;
  environment_out.collision_objects[0].pose.position.y = 0.0;
  environment_out.collision_objects[0].pose.position.z = 0.4;
  environment_out.collision_objects[0].pose.orientation.x = 0.0;
  environment_out.collision_objects[0].pose.orientation.y = 0.0;
  environment_out.collision_objects[0].pose.orientation.z = 0.0;
  environment_out.collision_objects[0].pose.orientation.w = 1.0;
  environment_out.collision_objects[1].id = "powl";
  environment_out.collision_objects[1].primitives.resize(1);
  environment_out.collision_objects[1].primitives[0].type
      = shape_msgs::msg::SolidPrimitive::CYLINDER;
  environment_out.collision_objects[1].primitives[0].dimensions.resize(2);
  environment_out.collision_objects[1].primitives[0].dimensions[0] = 0.05;
  environment_out.collision_objects[1].primitives[0].dimensions[1] = 0.2;
  environment_out.collision_objects[1].primitive_poses.resize(1);
  environment_out.collision_objects[1].primitive_poses[0].position.x = 0.0;
  environment_out.collision_objects[1].primitive_poses[0].position.y = 0.0;
  environment_out.collision_objects[1].primitive_poses[0].position.z = 0.0;
  environment_out.collision_objects[1].primitive_poses[0].orientation.x = 0.0;
  environment_out.collision_objects[1].primitive_poses[0].orientation.y = 0.707;
  environment_out.collision_objects[1].primitive_poses[0].orientation.z = 0.0;
  environment_out.collision_objects[1].primitive_poses[0].orientation.w = 0.707;
  environment_out.collision_objects[1].pose.position.x = 0.8;
  environment_out.collision_objects[1].pose.position.y = 0.0;
  environment_out.collision_objects[1].pose.position.z = 0.4;
  environment_out.collision_objects[1].pose.orientation.x = 0.0;
  environment_out.collision_objects[1].pose.orientation.y = 0.0;
  environment_out.collision_objects[1].pose.orientation.z = 0.0;
  environment_out.collision_objects[1].pose.orientation.w = 1.0;
}

tmc_planning_msgs::srv::PlanWithHandGoals::Request::SharedPtr CreateHandGoalsRequest() {
  auto request = std::make_shared<tmc_planning_msgs::srv::PlanWithHandGoals::Request>();
  request->origin_to_basejoint = CreateInitialPose();
  request->initial_joint_state = CreateInitialJointState();
  request->use_joints = kUseJoints;
  request->origin_to_hand_goals.resize(1);
  request->origin_to_hand_goals[0].position.x = 0.395;
  request->origin_to_hand_goals[0].position.y = 0.190;
  request->origin_to_hand_goals[0].position.z = 0.456;
  request->origin_to_hand_goals[0].orientation.x = 0.707;
  request->origin_to_hand_goals[0].orientation.y = 0;
  request->origin_to_hand_goals[0].orientation.z = 0.707;
  request->origin_to_hand_goals[0].orientation.w = 0;
  request->ref_frame_id = "CARM/BASE_HAND";
  request->probability_goal_generate = 0.2;
  request->timeout.sec = 10;
  request->max_iteration = 1000;
  request->uniform_bound_sampling = true;
  return request;
}

tmc_planning_msgs::srv::PlanWithHandLine::Request::SharedPtr CreateHandLineRequest() {
  auto request = std::make_shared<tmc_planning_msgs::srv::PlanWithHandLine::Request>();
  request->origin_to_basejoint = CreateInitialPose();
  request->initial_joint_state = CreateInitialJointState();
  request->use_joints = kUseJoints;
  request->axis.x = 1.0;
  request->axis.y = 0.0;
  request->axis.z = 0.0;
  request->ref_frame_id = "CARM/BASE_HAND";
  request->goal_value = 0.2;
  request->probability_goal_generate = 0.2;
  request->timeout.sec = 10;
  request->max_iteration = 1000;
  request->uniform_bound_sampling = true;
  return request;
}

tmc_planning_msgs::srv::PlanWithJointGoals::Request::SharedPtr CreateJointGoalsRequest() {
  auto request = std::make_shared<tmc_planning_msgs::srv::PlanWithJointGoals::Request>();
  request->origin_to_basejoint = CreateInitialPose();
  request->initial_joint_state = CreateInitialJointState();
  request->use_joints = kUseJoints;
  request->goal_joint_states.resize(1);
  request->goal_joint_states[0].position = {0.2, 0.0, 0.0, 0.5, 2.0, 0.0, 0.0, 0.0};
  request->timeout.sec = 10;
  request->max_iteration = 1000;
  return request;
}

tmc_planning_msgs::srv::PlanWithJointGoals::Request::SharedPtr CreateJointGoalsWithCollisionMapRequest() {
  auto request = std::make_shared<tmc_planning_msgs::srv::PlanWithJointGoals::Request>();
  request->origin_to_basejoint = CreateInitialPose();
  request->initial_joint_state = CreateInitialJointState();
  request->use_joints = kUseJoints;
  request->goal_joint_states.resize(1);
  request->goal_joint_states[0].position = {0.2, 0.0, 0.0, 0.5, 2.0, 0.0, 0.0, 0.0};
  request->timeout.sec = 10;
  request->max_iteration = 1000;

  CreateKnownObjects(request->environment_before_planning);
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = std::string("CARM/HAND/SHAPE_11/collision/0");
  attached_object.object.pose.position.x = 0.0;
  attached_object.object.pose.position.y = 0.0;
  attached_object.object.pose.position.z = 0.05;
  attached_object.object.id = "powl";
  request->attached_objects.push_back(attached_object);

  return request;
}
}  // namespace

namespace tmc_robot_rrt_planner_node {

class RobotRrtPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override;
  void TearDown() override;

  bool Interrupt() const { return do_interrupt_; }

  std::shared_ptr<RobotRrtPlannerNode> planner_node_;
  rclcpp::Node::SharedPtr client_node_;

  bool do_interrupt_;
  std::thread planner_spinner_;
  std::thread client_spinner_;
};

void RobotRrtPlannerTest::SetUp() {
  rclcpp::NodeOptions options;
  options.parameter_overrides() = {
      rclcpp::Parameter("robot_description", tmc_manipulation_tests::hsra::GetUrdf()),
      rclcpp::Parameter("robot_collision_pair", tmc_manipulation_tests::hsra::GetCollisionConfig()),
      rclcpp::Parameter("delta", 0.2),
      rclcpp::Parameter("sub_delta", 0.01),
      rclcpp::Parameter("publish_debug_info", false),
      rclcpp::Parameter("step_mode", false),
      rclcpp::Parameter("save_request", true),
      rclcpp::Parameter("weight_names", std::vector<std::string>({"BASE/_BASE_X", "CARM/LINEAR"})),
      rclcpp::Parameter("weights", std::vector<double>({10.0, 10.0})),
      rclcpp::Parameter("ik_weight_names", std::vector<std::string>({"BASE/_BASE_X"})),
      rclcpp::Parameter("ik_weights", std::vector<double>({30.0}))};
  planner_node_ = std::make_shared<RobotRrtPlannerNode>(options);
  planner_node_->Init();

  client_node_ = rclcpp::Node::make_shared("test_node");

  do_interrupt_ = false;
  std::function<bool()> interrupt = std::bind(&RobotRrtPlannerTest::Interrupt, this);
  planner_spinner_ = std::thread(std::bind(SpinSomeThread, planner_node_, interrupt));
  client_spinner_ = std::thread(std::bind(SpinSomeThread, client_node_, interrupt));
}

void RobotRrtPlannerTest::TearDown() {
  do_interrupt_ = true;
  planner_spinner_.join();
  client_spinner_.join();
}

TEST_F(RobotRrtPlannerTest, TestHandPlanning) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithHandGoals>(client_node_, kPlanWithHandGoals);
  ASSERT_TRUE(client);

  const auto request = CreateHandGoalsRequest();

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestHandPlanningWithCollision) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithHandGoals>(client_node_, kPlanWithHandGoals);
  ASSERT_TRUE(client);

  std::string dir(__FILE__);
  dir = dir.substr(0, dir.find_last_of("/"));
  const auto sphere_shape = shapes::createMeshFromResource(std::string("file://") + dir + "/sphere.stl");
  shapes::ShapeMsg shape_msg;
  EXPECT_TRUE(shapes::constructMsgFromShape(sphere_shape, shape_msg));

  auto request = CreateHandGoalsRequest();
  request->timeout.sec = 1;
  request->environment_before_planning.collision_objects.resize(1);
  request->environment_before_planning.collision_objects[0].id = "sphere";
  request->environment_before_planning.collision_objects[0].pose = request->origin_to_hand_goals[0];
  request->environment_before_planning.collision_objects[0].meshes.push_back(
      boost::get<shape_msgs::msg::Mesh>(shape_msg));
  request->environment_before_planning.collision_objects[0].mesh_poses.resize(1);

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_NE(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestHandPlanningWithNormalSampling) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithHandGoals>(client_node_, kPlanWithHandGoals);
  ASSERT_TRUE(client);

  const auto request = CreateHandGoalsRequest();
  request->uniform_bound_sampling = false;
  request->deviation_for_bound_sampling = 0.1;

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestLinePlanningForward) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithHandLine>(client_node_, kPlanWithHandLine);
  ASSERT_TRUE(client);

  const auto request = CreateHandLineRequest();

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestLinePlanningBackward) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithHandLine>(client_node_, kPlanWithHandLine);
  ASSERT_TRUE(client);

  const auto request = CreateHandLineRequest();
  request->goal_value = -request->goal_value;
  request->axis.x = -request->axis.x;
  request->axis.y = -request->axis.y;
  request->axis.z = -request->axis.z;

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestLinePlanningWithNormalSampling) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithHandLine>(client_node_, kPlanWithHandLine);
  ASSERT_TRUE(client);

  const auto request = CreateHandLineRequest();
  request->uniform_bound_sampling = false;
  request->deviation_for_bound_sampling = 0.1;

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestSimplePlanning) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithJointGoals>(client_node_, kPlanWithJointGoals);
  ASSERT_TRUE(client);

  const auto request = CreateJointGoalsRequest();

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

TEST_F(RobotRrtPlannerTest, TestSimplePlanningWithCollisionMap) {
  const auto client = CreateClient<tmc_planning_msgs::srv::PlanWithJointGoals>(client_node_, kPlanWithJointGoals);
  ASSERT_TRUE(client);

  const auto request = CreateJointGoalsWithCollisionMapRequest();

  auto result = client->async_send_request(request);
  while (result.wait_for(std::chrono::milliseconds(1)) != std::future_status::ready) {}

  const auto response = result.get();
  EXPECT_EQ(response->error_code.val, moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
}

// TEST_F(GoogleTestSimplePlanning, TestSimplePlanningWithExtraConstraint) {
//   ros::service::waitForService(kServiceNamePlanWithJointGoals,
//                                ros::Duration(10.0));
//   ASSERT_TRUE(ros::service::exists(kServiceNamePlanWithJointGoals, false));

//   plan_.request.extra_constraints.push_back(
//       "tmc_robot_planner_plugins/ResetCounter");
//   EXPECT_TRUE(ros::service::call(kServiceNamePlanWithJointGoals, plan_));
//   EXPECT_EQ(plan_.response.error_code.val,
//             tmc_manipulation_msgs::ArmManipulationErrorCodes::SUCCESS);
//   int32_t reset_counter = -1;
//   EXPECT_TRUE(
//       ros::param::get("/reset_counter/num_of_reset_call", reset_counter));
//   EXPECT_EQ(1, reset_counter);

//   EXPECT_TRUE(ros::service::call(kServiceNamePlanWithJointGoals, plan_));
//   EXPECT_EQ(plan_.response.error_code.val,
//             tmc_manipulation_msgs::ArmManipulationErrorCodes::SUCCESS);
//   EXPECT_TRUE(
//       ros::param::get("/reset_counter/num_of_reset_call", reset_counter));
//   EXPECT_EQ(2, reset_counter);
// }
}  // namespace tmc_robot_rrt_planner_node

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
