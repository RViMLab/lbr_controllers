#include "lbr_broadcasters/lbr_virtual_force_torque_broadcaster.hpp"

namespace lbr_broadcasters {

LBRVirtualForceTorqueBroadcaster::LBRVirtualForceTorqueBroadcaster()
    : virtual_force_torque_publisher_(nullptr), virtual_force_torque_realtime_publisher_(nullptr),
      state_interface_names_{lbr_hardware_interface::HW_IF_EXTERNAL_TORQUE,
                             hardware_interface::HW_IF_POSITION} {}

controller_interface::InterfaceConfiguration
LBRVirtualForceTorqueBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
LBRVirtualForceTorqueBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    for (const auto &state_interface_name : state_interface_names_) {
      interface_configuration.names.push_back(joint_name + "/" + state_interface_name);
    }
  }
  return interface_configuration;
}

controller_interface::CallbackReturn LBRVirtualForceTorqueBroadcaster::on_init() {}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state) {
  if (!read_parameters_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_activate(const rclcpp_lifecycle::State &previous_state) {
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRVirtualForceTorqueBroadcaster::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
  // compute j pseudo inverse times ext
  RCLCPP_INFO(get_node()->get_logger(), "external torques: %f, %f, %f, %f, %f, %f, %f",
              external_torque_state_interfaces_[0].get().get_value(),
              external_torque_state_interfaces_[1].get().get_value(),
              external_torque_state_interfaces_[2].get().get_value(),
              external_torque_state_interfaces_[3].get().get_value(),
              external_torque_state_interfaces_[4].get().get_value(),
              external_torque_state_interfaces_[5].get().get_value(),
              external_torque_state_interfaces_[6].get().get_value());
  return controller_interface::return_type::OK;
}

bool LBRVirtualForceTorqueBroadcaster::read_parameters_() {
  try {
    if (!get_node()->get_parameter("joints", joint_names_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve joint names parameter.");
      return false;
    }
    if (joint_names_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Empty joint parameter provided.");
      return false;
    }
    if (joint_names_.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %lu joint names, got %d.",
                   joint_names_.size(), lbr_fri_ros2::LBR::JOINT_DOF);
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBRVirtualForceTorqueBroadcaster::reference_state_interfaces_() {
  try {
    for (auto &state_interface : state_interfaces_) {
      if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        position_state_interfaces_.emplace_back(std::ref(state_interface));
      }
      if (state_interface.get_interface_name() == lbr_hardware_interface::HW_IF_EXTERNAL_TORQUE) {
        external_torque_state_interfaces_.emplace_back(std::ref(state_interface));
      }
    }
    if (position_state_interfaces_.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d position state interfaces, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, position_state_interfaces_.size());
      return false;
    }
    if (external_torque_state_interfaces_.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Expected %d external torque state interfaces, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, external_torque_state_interfaces_.size());
      return false;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Exception thrown while referencing state interfaces.\n%s", e.what());
    return false;
  }
  return true;
}

bool LBRVirtualForceTorqueBroadcaster::clear_state_interfaces_() {
  position_state_interfaces_.clear();
  external_torque_state_interfaces_.clear();
  return true;
}

} // end of namespace lbr_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_broadcasters::LBRVirtualForceTorqueBroadcaster,
                       controller_interface::ControllerInterface)
