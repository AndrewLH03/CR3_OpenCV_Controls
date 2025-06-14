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
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/robot/BasicCommand.msg"
  "msg/robot/RobotStatus.msg"
  "msg/perception/HandPosition.msg"
  "msg/safety/SafetyStatus.msg"
  "msg/safety/SafetyAlert.msg"
  "msg/coordination/CoordinateTransform.msg"
  "msg/PoseCoordinates.msg"
  "msg/PoseTrackingStatus.msg"
  "msg/DebugInfo.msg"
  "srv/safety/EmergencyStop.srv"
  "srv/robot/SetParameters.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs diagnostic_msgs sensor_msgs
)

# Build basic robot controller executable
add_executable(basic_robot_controller src/robot_control/basic_robot_controller.cpp)

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

# Build workspace validator executable
add_executable(workspace_validator src/safety/workspace_validator.cpp)

ament_target_dependencies(workspace_validator
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
)

# Build emergency stop handler executable
add_executable(emergency_stop_handler src/safety/emergency_stop_handler.cpp)

ament_target_dependencies(emergency_stop_handler
  rclcpp
  std_msgs
  geometry_msgs
  builtin_interfaces
)

# Build coordinate broadcaster executable
add_executable(coordinate_broadcaster src/coordination/coordinate_broadcaster.cpp)

ament_target_dependencies(coordinate_broadcaster
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  builtin_interfaces
)

# Link the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(basic_robot_controller "${cpp_typesupport_target}")
target_link_libraries(workspace_validator "${cpp_typesupport_target}")
target_link_libraries(emergency_stop_handler "${cpp_typesupport_target}")
target_link_libraries(coordinate_broadcaster "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  basic_robot_controller
  workspace_validator
  emergency_stop_handler
  coordinate_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/robot_control/parameter_manager.py
  scripts/robot_control/diagnostics_monitor.py
  scripts/safety/safety_monitor.py
  scripts/coordination/transform_manager.py
  scripts/coordination/calibration_node.py
  src/cr3_hand_control/pose_recognition_node.py
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
