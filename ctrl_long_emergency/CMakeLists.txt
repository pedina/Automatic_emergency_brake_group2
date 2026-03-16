cmake_minimum_required(VERSION 3.8)
project(ctrl_long_emergency)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)


include_directories(include)

add_executable(control_publisher src/controlPublisher.cpp)
ament_target_dependencies(control_publisher rclcpp autoware_control_msgs)

install(TARGETS
  control_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
