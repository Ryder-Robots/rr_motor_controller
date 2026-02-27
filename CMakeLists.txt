cmake_minimum_required(VERSION 3.20)
project(rr_motor_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rr_common_base REQUIRED)
find_package(rr_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

add_library(rr_motor_controller 
  src/rr_motor_controller.cpp
  src/dc_linear.cpp
  src/motor.cpp
  src/encoder.cpp
  src/rr_motor_ecu_node.cpp
  src/differential.cpp)
add_library(rr_motor_controller::rr_motor_controller ALIAS rr_motor_controller)
target_compile_features(rr_motor_controller PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(rr_motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(
  rr_motor_controller PUBLIC
  ${rr_common_base_TARGETS}
  ${rr_interfaces_TARGETS}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${rclcpp_components_TARGETS}
  ${rclcpp_action_TARGETS}
  ${pluginlib_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${nav2_util_TARGETS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rr_motor_controller PRIVATE "RR_MOTOR_CONTROLLER_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS rr_motor_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Temporarily disable specific linters/tests that are noisy in CI
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_lint_cmake_FOUND TRUE)
  set(ament_uncrustify_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add Motor unit tests
  ament_add_gtest(test_motor test/test_motor.cpp)
  target_include_directories(test_motor PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_link_libraries(test_motor rr_motor_controller)

  # Add Encoder unit tests
  ament_add_gtest(test_encoder test/test_encoder.cpp)
  target_include_directories(test_encoder PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_link_libraries(test_encoder rr_motor_controller)

  # Add DutyConvertorLinearRegression unit tests
  ament_add_gtest(test_dc_linear test/test_dc_linear.cpp)
  target_include_directories(test_dc_linear PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_link_libraries(test_dc_linear rr_motor_controller)

  # Add RrMotorController unit tests
  ament_add_gtest(test_rr_motor_controller test/test_rr_motor_controller.cpp)
  target_include_directories(test_rr_motor_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_link_libraries(test_rr_motor_controller rr_motor_controller)

  # Add DifferentialCmdProc unit tests
  ament_add_gtest(test_differential test/test_differential.cpp)
  target_include_directories(test_differential PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
  target_link_libraries(test_differential rr_motor_controller)
endif()

ament_export_dependencies(
  nav_msgs
  nav2_util
  tf2
  tf2_geometry_msgs
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  rclcpp_action
  pluginlib
  rr_common_base
  rr_interfaces
)

rclcpp_components_register_nodes(rr_motor_controller
  "rr_motor_controller::RrECU"
)

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  rr_motor_controller
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
