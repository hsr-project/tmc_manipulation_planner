cmake_minimum_required(VERSION 3.5)
project(tmc_robot_planner)

# For using pinocchio library used in tmc_robot_kinematics_model
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath -Wl,/opt/openrobots/lib")

find_package(ament_cmake REQUIRED)
find_package(tmc_eigen_utils REQUIRED)
find_package(tmc_manipulation_types REQUIRED)
find_package(tmc_robot_kinematics_model REQUIRED)
find_package(tmc_robot_collision_detector REQUIRED)
find_package(tmc_rplanner REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)

add_library(${PROJECT_NAME} SHARED src/robot_cbirrt_planner.cpp src/task_space_region.cpp)
target_link_libraries(${PROJECT_NAME} Boost::timer)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} tmc_eigen_utils tmc_manipulation_types tmc_robot_collision_detector tmc_robot_kinematics_model tmc_rplanner)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tmc_manipulation_tests REQUIRED)

  ament_add_gtest(test_tsr test/test_tsr.cpp)
  target_link_libraries(test_tsr ${PROJECT_NAME})

  ament_add_gtest(test_robot_planner test/test_robot_planner.cpp)
  target_link_libraries(test_robot_planner ${PROJECT_NAME})
  ament_target_dependencies(test_robot_planner tmc_manipulation_tests)

  ament_add_gtest(robot_cbirrt_planner-test test/robot_cbirrt_planner-test.cpp)
  target_link_libraries(robot_cbirrt_planner-test ${PROJECT_NAME})
  ament_target_dependencies(robot_cbirrt_planner-test tmc_manipulation_tests)
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(tmc_eigen_utils tmc_manipulation_types tmc_robot_collision_detector tmc_robot_kinematics_model tmc_rplanner)

ament_package()
