cmake_minimum_required(VERSION 3.8)
project(no7)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add your executable
add_executable(spawn_turtles src/spawn_turtles.cpp)

ament_target_dependencies(spawn_turtles
  rclcpp
  std_msgs
  geometry_msgs
  turtlesim
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(broadcaster src/broadcaster.cpp)

ament_target_dependencies(broadcaster
  rclcpp
  std_msgs
  geometry_msgs
  turtlesim
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  spawn_turtles
  broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY                   
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
