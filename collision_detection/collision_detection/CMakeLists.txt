cmake_minimum_required(VERSION 3.8)
project(collision_detection)

find_package(ament_cmake REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} src/collision_detection.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  crp_msgs autoware_planning_msgs autoware_perception_msgs rclcpp geometry_msgs)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)


# Add test executable
add_executable(test_collision_detection src/test_collision_detection.cpp)
target_link_libraries(test_collision_detection ${PROJECT_NAME})
ament_target_dependencies(test_collision_detection
  crp_msgs autoware_planning_msgs autoware_perception_msgs geometry_msgs)
install(TARGETS test_collision_detection DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()