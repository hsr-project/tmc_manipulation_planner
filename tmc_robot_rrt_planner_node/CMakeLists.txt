cmake_minimum_required(VERSION 3.5)
project(tmc_robot_rrt_planner_node)

# For using pinocchio library used in tmc_robot_kinematics_model
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,/opt/openrobots/lib")

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tmc_manipulation_types_bridge REQUIRED)
find_package(tmc_planning_msgs REQUIRED)
find_package(tmc_robot_planner REQUIRED)
find_package(tmc_utils REQUIRED)

find_package(Boost REQUIRED COMPONENTS date_time)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED src/robot_rrt_planner_node.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  tf2_eigen
  tf2_ros
  tmc_manipulation_types_bridge
  tmc_planning_msgs
  tmc_robot_planner
  tmc_utils
)

add_executable(robot_rrt_planner_node src/robot_rrt_planner_node_main.cpp)
target_link_libraries(robot_rrt_planner_node ${PROJECT_NAME})
ament_target_dependencies(robot_rrt_planner_node rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometric_shapes REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(robot_rrt_planner_node_test test/robot_rrt_planner_node-test.cpp)
  target_link_libraries(robot_rrt_planner_node_test ${PROJECT_NAME})
  ament_target_dependencies(robot_rrt_planner_node_test geometric_shapes rclcpp tmc_manipulation_tests)
endif()

install(TARGETS ${PROJECT_NAME} robot_rrt_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories()
ament_export_libraries()
ament_export_dependencies(pluginlib rclcpp tf2_eigen tf2_ros tmc_manipulation_types_bridge tmc_planning_msgs tmc_robot_planner tmc_utils)

ament_package()
