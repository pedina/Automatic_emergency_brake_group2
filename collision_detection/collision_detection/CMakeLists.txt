cmake_minimum_required(VERSION 3.8)
project(collision_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Függőségek megkeresése
find_package(ament_cmake REQUIRED)
find_package(crp_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
# Ha később ROS 2 Node-ot is írsz ide, a find_package(rclcpp REQUIRED) is kelleni fog!

# 2. A mag-logika könyvtárrá (library) alakítása
add_library(collision_detection_lib SHARED
        src/collision_detection.cpp
)
target_include_directories(collision_detection_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(collision_detection_lib
        crp_msgs
        autoware_planning_msgs
)

# 3. A teszt futtatható állomány létrehozása
add_executable(test_collision_detection
        src/test_collision_detection.cpp
)
target_link_libraries(test_collision_detection
        collision_detection_lib
)

# 4. Telepítési (Install) szabályok a ROS 2 számára
install(TARGETS
        collision_detection_lib
        test_collision_detection
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()