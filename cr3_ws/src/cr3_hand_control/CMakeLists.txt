cmake_minimum_required(VERSION 3.8)
project(cr3_hand_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BasicCommand.msg"
  "msg/RobotStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Build basic robot controller executable
add_executable(basic_robot_controller src/basic_robot_controller.cpp)

ament_target_dependencies(basic_robot_controller
  rclcpp
  std_msgs
  builtin_interfaces
)

# Link the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(basic_robot_controller "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  basic_robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
