cmake_minimum_required(VERSION 3.8)
project(craft_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add rudder_servo library related compile commands
generate_parameter_library(rudder_servo_parameters
  src/rudder_servo.yaml
  include/craft_control/validate_rudder_servo_parameters.hpp
)
add_library(
  rudder_servo
  SHARED
  src/rudder_servo.cpp
)
target_include_directories(rudder_servo PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(rudder_servo rudder_servo_parameters)
ament_target_dependencies(rudder_servo ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(rudder_servo PRIVATE "RUDDER_SERVO_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface craft_control.xml)

install(
  TARGETS
  rudder_servo
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_rudder_servo test/test_load_rudder_servo.cpp)
  target_include_directories(test_load_rudder_servo PRIVATE include)
  ament_target_dependencies(
    test_load_rudder_servo
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  add_rostest_with_parameters_gmock(test_rudder_servo test/test_rudder_servo.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/rudder_servo_params.yaml)
  target_include_directories(test_rudder_servo PRIVATE include)
  target_link_libraries(test_rudder_servo rudder_servo)
  ament_target_dependencies(
    test_rudder_servo
    controller_interface
    hardware_interface
  )

  add_rostest_with_parameters_gmock(test_rudder_servo_preceeding test/test_rudder_servo_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/rudder_servo_preceeding_params.yaml)
  target_include_directories(test_rudder_servo_preceeding PRIVATE include)
  target_link_libraries(test_rudder_servo_preceeding rudder_servo)
  ament_target_dependencies(
    test_rudder_servo_preceeding
    controller_interface
    hardware_interface
  )
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_export_libraries(
  rudder_servo
)

ament_package()
