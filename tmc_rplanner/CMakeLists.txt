cmake_minimum_required(VERSION 3.5)
project(tmc_rplanner)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/configuration_space.cpp
  src/configuration_tree.cpp
  src/planner_common.cpp
  src/birrt_planner.cpp
  src/raund_robin_short_cutter.cpp
  src/random_optimizer.cpp
  src/rrt_planner.cpp
  src/multi_birrt_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_birrt2d examples/birrt2d.cpp)
target_include_directories(${PROJECT_NAME}_birrt2d PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_birrt2d ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_rrt2d examples/rrt2d.cpp)
target_include_directories(${PROJECT_NAME}_rrt2d PRIVATE include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_rrt2d ${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_configuration_space test/test_configuration_space.cpp)
  target_link_libraries(test_configuration_space ${PROJECT_NAME})

  ament_add_gtest(test_configuration_tree test/test_configuration_tree.cpp)
  target_link_libraries(test_configuration_tree ${PROJECT_NAME})

  ament_add_gtest(test_rrt_planner test/test_rrt_planner.cpp)
  target_link_libraries(test_rrt_planner ${PROJECT_NAME})

  ament_add_gtest(test_birrt_planner test/test_birrt_planner.cpp)
  target_link_libraries(test_birrt_planner ${PROJECT_NAME})

  ament_add_gtest(test_multi_birrt_planner test/test_multi_birrt_planner.cpp)
  target_link_libraries(test_multi_birrt_planner ${PROJECT_NAME})

  ament_add_gtest(test_raund_robin_short_cutter test/test_raund_robin_short_cutter.cpp)
  target_link_libraries(test_raund_robin_short_cutter ${PROJECT_NAME})

  ament_add_gtest(test_random_optimizer test/test_random_optimizer.cpp)
  target_link_libraries(test_random_optimizer ${PROJECT_NAME})
endif()

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_birrt2d ${PROJECT_NAME}_rrt2d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies()

ament_package()