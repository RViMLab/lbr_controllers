cmake_minimum_required(VERSION 3.5)
project(rvim_position_controllers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

##
add_library(rvim_position_controllers
  SHARED
  src/hand_guide_position_controller.cpp
)

target_include_directories(
  rvim_position_controllers 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src    
)

ament_target_dependencies(rvim_position_controllers
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  orocos_kdl
  kdl_parser
  urdf
  Eigen3
  realtime_tools
  geometry_msgs
  std_msgs
)

target_link_libraries(rvim_position_controllers OsqpEigen::OsqpEigen)

pluginlib_export_plugin_description_file(controller_interface rvim_position_controllers.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS rvim_position_controllers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if (BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(
    test_load_controllers
    test/test_load_hand_guide_position_controller.cpp
  )
  target_include_directories(test_load_controllers PRIVATE include)
  ament_target_dependencies(test_load_controllers
    rclcpp
    hardware_interface
    controller_manager
    ros2_control_test_assets
  )
endif ()

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  pluginlib
  orocos_kdl
  kdl_parser
  urdf
  realtime_tools
  geometry_msgs
  std_msgs
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  rvim_position_controllers
)

ament_package()
