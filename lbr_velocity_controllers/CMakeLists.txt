cmake_minimum_required(VERSION 3.8)
project(lbr_velocity_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

add_library(
  lbr_velocity_controllers
  SHARED
    src/lbr_velocity_controller.cpp
)

target_include_directories(
  lbr_velocity_controllers
  PRIVATE include
)

set(lbr_velocity_controllers_dependencies
  control_toolbox
  controller_interface
  hardware_interface
  lbr_fri_ros2
  rcl_interfaces
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

ament_target_dependencies(
  lbr_velocity_controllers
  ${lbr_velocity_controllers_dependencies}
)

pluginlib_export_plugin_description_file(controller_interface lbr_velocity_controllers.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_lbr_velocity_controller
    test/test_load_lbr_velocity_controller.cpp
  )
  target_link_libraries(test_load_lbr_velocity_controller
    lbr_velocity_controllers
  )
  ament_target_dependencies(test_load_lbr_velocity_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS lbr_velocity_controllers
  EXPORT export_lbr_velocity_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_lbr_velocity_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  lbr_velocity_controllers
  ${lbr_velocity_controllers_dependencies}
)

ament_package()
