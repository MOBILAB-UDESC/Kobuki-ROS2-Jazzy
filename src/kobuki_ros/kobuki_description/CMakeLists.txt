cmake_minimum_required(VERSION 3.8)
project(kobuki_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(xacro REQUIRED)

install(
  DIRECTORY launch model config meshes worlds rviz src
  DESTINATION share/${PROJECT_NAME}
)

add_executable(velocity_control src/velocity_control.cpp)
ament_target_dependencies(velocity_control
  rclcpp
  geometry_msgs
)

add_executable(twist_to_twiststamped src/KobukiRemap.cpp)
ament_target_dependencies(twist_to_twiststamped
  rclcpp
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    velocity_control
    twist_to_twiststamped
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
