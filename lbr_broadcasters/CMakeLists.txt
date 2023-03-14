cmake_minimum_required(VERSION 3.8)
project(lbr_broadcasters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(lbr_fri_ros2 REQUIRED)
find_package(lbr_hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

set(lbr_broadcaster_dependencies
  controller_interface
  Eigen3
  geometry_msgs
  hardware_interface
  kinematics_interface
  lbr_fri_ros2
  lbr_hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

add_library(lbr_broadcasters
  SHARED
    src/lbr_virtual_force_torque_broadcaster.cpp
)

target_include_directories(lbr_broadcasters
  PRIVATE include
)

ament_target_dependencies(lbr_broadcasters
  ${lbr_broadcaster_dependencies}
)

pluginlib_export_plugin_description_file(controller_interface lbr_broadcasters.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_lbr_virtual_force_torque_broadcaster
    test/test_load_lbr_virtual_force_torque_broadcaster.cpp
  )
  target_link_libraries(test_load_lbr_virtual_force_torque_broadcaster
    lbr_broadcasters
  )
  ament_target_dependencies(test_load_lbr_virtual_force_torque_broadcaster
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
  TARGETS lbr_broadcasters
  EXPORT export_lbr_broadcasters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_lbr_broadcasters HAS_LIBRARY_TARGET)
ament_export_dependencies(${lbr_broadcaster_dependencies})

ament_package()
