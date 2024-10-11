cmake_minimum_required(VERSION 3.5)
project(tmc_timeopt)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(console_bridge REQUIRED)
link_directories(${console_bridge_LIBRARY_DIRS})

add_library(trajectory_filter SHARED
  src/quick_trajectory_filter.cpp
  src/trajectory_filter_via_stop_state.cpp
  src/Path.cpp
  src/trajectory_filter_utils.cpp
  src/Trajectory.cpp)
target_include_directories(trajectory_filter PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})
target_link_libraries(trajectory_filter ${console_bridge_LIBRARIES})

install(TARGETS trajectory_filter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_quick_trajectory_filter test/quick_trajectory_filter-test.cpp)
  target_link_libraries(test_quick_trajectory_filter trajectory_filter)

  ament_add_gtest(test_trajectory_filter_via_stop_state test/trajectory_filter_via_stop_state-test.cpp)
  target_link_libraries(test_trajectory_filter_via_stop_state trajectory_filter)

  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_dynamics_accel test/test_dynamics_accel.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_dynamics_effort test/test_dynamics_effort.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_inverval test/test_interval.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_kinematics test/test_kinematics.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_nspline test/test_nspline.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_poly2 test/test_poly2.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_timeopt test/test_timeopt.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_total test/test_total.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
  ament_add_pytest_test(test_trajectory test/test_trajectory.py WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

ament_export_include_directories(include)
ament_export_libraries(trajectory_filter)
ament_export_dependencies()

ament_package()
