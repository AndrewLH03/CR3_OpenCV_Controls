cmake_minimum_required(VERSION 3.8)
project(cr3_hand_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BasicCommand.msg"
  "msg/RobotStatus.msg"
  "msg/HandPosition.msg"
  "msg/SafetyStatus.msg"
  "srv/EmergencyStop.srv"
  "srv/SetParameters.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs diagnostic_msgs
)

# Build basic robot controller executable
add_executable(basic_robot_controller src/basic_robot_controller.cpp)

ament_target_dependencies(basic_robot_controller
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
  diagnostic_msgs
  rcl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(basic_robot_controller "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  basic_robot_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/parameter_manager.py
  scripts/diagnostics_monitor.py
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
