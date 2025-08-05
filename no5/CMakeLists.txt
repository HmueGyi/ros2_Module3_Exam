cmake_minimum_required(VERSION 3.8)
project(no5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Hss.msg"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(custom_pub src/custom_publisher.cpp)
ament_target_dependencies(custom_pub rclcpp std_msgs geometry_msgs rosidl_default_runtime) 

rosidl_target_interfaces(custom_pub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(custom_sub src/custom_subscriber.cpp)
ament_target_dependencies(custom_sub rclcpp std_msgs geometry_msgs rosidl_default_runtime)

rosidl_target_interfaces(custom_sub
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  custom_pub
  custom_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
