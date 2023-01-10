#include "lbr_velocity_controllers/lbr_velocity_controller.hpp"

namespace lbr_velocity_controllers {
LBRVelocityController::LBRVelocityController()
    : joint_velocity_command_rt_buffer_(nullptr), joint_velocity_command_subscription_(nullptr),
      command_interface_names_{hardware_interface::HW_IF_POSITION},
      state_interface_names_{hardware_interface::HW_IF_POSITION,
                             hardware_interface::HW_IF_VELOCITY} {}

controller_interface::InterfaceConfiguration
LBRVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration{
      controller_interface::interface_configuration_type::INDIVIDUAL};
  for (const auto &joint_name : joint_names_) {
    for (const auto &command_interface_name : command_interface_names_) {
      interface_configuration.names.push_back(joint_name + "/" + command_interface_name);
    }
  }
  return interface_configuration;
}
controller_interface::InterfaceConfiguration
LBRVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration{
      controller_interface::interface_configuration_type::INDIVIDUAL};
  for (const auto &joint_name : joint_names_) {
    for (const auto &state_interface_name : state_interface_names_) {
      interface_configuration.names.push_back(joint_name + "/" + state_interface_name);
    }
  }
  return interface_configuration;
}
controller_interface::CallbackReturn LBRVelocityController::on_init() {
  if (!declare_parameters_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVelocityController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!read_parameters_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_velocity_command_subscription_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "~/joint_velocity_command", rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr joint_velocity_command) {
            if (joint_velocity_command->data.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
              RCLCPP_ERROR(get_node()->get_logger(),
                           "Received velocity command of size %lu, expected %d.",
                           joint_velocity_command->data.size(), lbr_fri_ros2::LBR::JOINT_DOF);
              return;
            }
            joint_velocity_command_rt_buffer_->writeFromNonRT(joint_velocity_command);
          });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVelocityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!clear_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_velocity_command_rt_buffer_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!clear_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRVelocityController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  return controller_interface::return_type::OK;
}

bool LBRVelocityController::declare_parameters_() {
  try {
    if (!get_node()->has_parameter("joints")) {
      get_node()->declare_parameter<std::vector<std::string>>("joints");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBRVelocityController::read_parameters_() {
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

bool LBRVelocityController::reference_command_interfaces_() {
  try {
    for (auto &command_interface : command_interfaces_) {
      if (command_interface.get_name() == hardware_interface::HW_IF_POSITION) {
        position_command_interfaces_.emplace_back(std::ref(command_interface));
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reference command interfaces.\n%s.",
                 e.what());
    return false;
  }
  return true;
}

bool LBRVelocityController::reference_state_interfaces_() {
  try {
    for (auto &state_interface : state_interfaces_) {
      if (state_interface.get_name() == hardware_interface::HW_IF_POSITION) {
        position_state_interfaces_.emplace_back(std::ref(state_interface));
      }
      if (state_interface.get_name() == hardware_interface::HW_IF_VELOCITY) {
        velocity_state_interfaces_.emplace_back(std::ref(state_interface));
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reference state interfaces.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBRVelocityController::clear_command_interfaces_() {
  position_command_interfaces_.clear();
  return true;
}

bool LBRVelocityController::clear_state_interfaces_() {
  position_state_interfaces_.clear();
  velocity_state_interfaces_.clear();
  return true;
}
} // end of namespace lbr_velocity_controllers

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(lbr_velocity_controllers::LBRVelocityController,
                       controller_interface::ControllerInterface)
