cmake_minimum_required(VERSION 3.8)
project(behavior_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(tier4_planning_msgs REQUIRED)

include_directories(include)

add_executable(behavior_planner src/behaviorPlanner.cpp)

ament_target_dependencies(behavior_planner
  rclcpp
  crp_msgs
  tier4_planning_msgs
)

install(TARGETS
  behavior_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
