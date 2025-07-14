cmake_minimum_required(VERSION 3.8)
project(kobuki_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(kobuki_msgs REQUIRED)
# find_package(rosidl_default_runtime REQUIRED)

include_directories(
  include
  # ${kobuki_msgs_INCLUDE_DIRS}
)
set(SRC_FILES
  src/map.cpp
  src/Astar.cpp
  src/HybridAstar.cpp
  src/utils.cpp
  src/smooth.cpp
  src/dubins.c
)

install(
  DIRECTORY include src map
  DESTINATION share/${PROJECT_NAME}
)

add_executable(main2
    src/main2.cpp
    ${SRC_FILES}
)
add_executable(main
    src/main.cpp
    ${SRC_FILES}
)

# target_link_libraries(map
#   rclcpp
# )
ament_target_dependencies(main2
  rclcpp
  nav_msgs
  OpenCV
  geometry_msgs
  tf2_geometry_msgs
  kobuki_msgs
)
ament_target_dependencies(main
  rclcpp
  nav_msgs
  OpenCV
  geometry_msgs
  tf2_geometry_msgs
  kobuki_msgs
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    main2
    main
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
