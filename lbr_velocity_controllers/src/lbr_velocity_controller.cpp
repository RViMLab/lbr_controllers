#include "lbr_velocity_controllers/lbr_velocity_controller.hpp"

namespace lbr_velocity_controllers {
LBRVelocityController::LBRVelocityController()
    : on_set_parameter_callback_handle_ptr_(nullptr),
      velocity_control_rule_(std::make_unique<VelocityControlRule>(lbr_fri_ros2::LBR::JOINT_DOF)),
      joint_velocity_command_rt_buffer_(nullptr), joint_velocity_command_subscription_(nullptr),
      command_interface_names_{hardware_interface::HW_IF_POSITION},
      state_interface_names_{hardware_interface::HW_IF_POSITION,
                             hardware_interface::HW_IF_VELOCITY},
      current_position_(lbr_fri_ros2::LBR::JOINT_DOF, std::numeric_limits<double>::quiet_NaN()),
      current_velocity_(lbr_fri_ros2::LBR::JOINT_DOF, std::numeric_limits<double>::quiet_NaN()),
      desired_velocity_(lbr_fri_ros2::LBR::JOINT_DOF, std::numeric_limits<double>::quiet_NaN()),
      position_command_(lbr_fri_ros2::LBR::JOINT_DOF, std::numeric_limits<double>::quiet_NaN()) {}

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
          "~/command", rclcpp::SystemDefaultsQoS(),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr joint_velocity_command) {
            if (joint_velocity_command->data.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
              RCLCPP_ERROR(get_node()->get_logger(),
                           "Expected %d elements in joint velocity command, got %lu.",
                           lbr_fri_ros2::LBR::JOINT_DOF, joint_velocity_command->data.size());
              return;
            }
            joint_velocity_command_rt_buffer_.writeFromNonRT(joint_velocity_command);
          });

  on_set_parameter_callback_handle_ptr_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        for (const auto &param : parameters) {
          if (param.get_name() == "desired_velocity_exp_smooth") {
            velocity_control_rule_->param().desired_velocity_exp_smooth = param.as_double();
            RCLCPP_INFO(get_node()->get_logger(), "Set desired_velocity_exp_smooth to %f.",
                        velocity_control_rule_->param().desired_velocity_exp_smooth);
          } else if (param.get_name() == "period_exp_smooth") {
            velocity_control_rule_->param().period_exp_smooth = param.as_double();
            RCLCPP_INFO(get_node()->get_logger(), "Set period_exp_smooth to %f.",
                        velocity_control_rule_->param().period_exp_smooth);
          } else if (param.get_name() == "position_exp_smooth") {
            velocity_control_rule_->param().position_exp_smooth = param.as_double();
            RCLCPP_INFO(get_node()->get_logger(), "Set position_exp_smooth to %f.",
                        velocity_control_rule_->param().position_exp_smooth);
          } else if (param.get_name() == "velocity_exp_smooth") {
            velocity_control_rule_->param().velocity_exp_smooth = param.as_double();
            RCLCPP_INFO(get_node()->get_logger(), "Set velocity_exp_smooth to %f.",
                        velocity_control_rule_->param().velocity_exp_smooth);
          } else {
            result.successful = false;
            result.reason = "Skipping unhandled parameter.";
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set parameter: %s.",
                         result.reason.c_str());
          }
        }
        return result;
      });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVelocityController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!init_rt_buffer_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
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
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVelocityController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!init_rt_buffer_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!clear_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LBRVelocityController::update(const rclcpp::Time & /*time*/,
                                                                const rclcpp::Duration &period) {
  auto joint_velocity_command = joint_velocity_command_rt_buffer_.readFromRT();
  if (!joint_velocity_command || !(*joint_velocity_command)) {
    return controller_interface::return_type::OK;
  }
  if ((*joint_velocity_command)->data.size() != desired_velocity_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Expected velocity command of size %lu, got %lu.",
                 desired_velocity_.size(), (*joint_velocity_command)->data.size());
    return controller_interface::return_type::ERROR;
  }
  for (uint8_t i = 0; i < lbr_fri_ros2::LBR::JOINT_DOF; ++i) {
    current_position_[i] = position_state_interfaces_[i].get().get_value();
    current_velocity_[i] = velocity_state_interfaces_[i].get().get_value();
    desired_velocity_[i] = (*joint_velocity_command)->data[i];
  }
  velocity_control_rule_->compute(current_position_, current_velocity_, desired_velocity_,
                                  period.seconds(), position_command_);
  for (uint8_t i = 0; i < lbr_fri_ros2::LBR::JOINT_DOF; ++i) {
    position_command_interfaces_[i].get().set_value(position_command_[i]);
  }
  return controller_interface::return_type::OK;
}

bool LBRVelocityController::declare_parameters_() {
  try {
    if (!get_node()->has_parameter("joints")) {
      get_node()->declare_parameter<std::vector<std::string>>("joints");
    }
    if (!get_node()->has_parameter("desired_velocity_exp_smooth")) {
      get_node()->declare_parameter("desired_velocity_exp_smooth", 0.02);
    }
    if (!get_node()->has_parameter("period_exp_smooth")) {
      get_node()->declare_parameter("period_exp_smooth", 0.02);
    }
    if (!get_node()->has_parameter("position_exp_smooth")) {
      get_node()->declare_parameter("position_exp_smooth", 0.02);
    }
    if (!get_node()->has_parameter("velocity_exp_smooth")) {
      get_node()->declare_parameter("velocity_exp_smooth", 0.02);
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

bool LBRVelocityController::init_rt_buffer_() {
  try {
    joint_velocity_command_rt_buffer_ =
        realtime_tools::RealtimeBuffer<std_msgs::msg::Float64MultiArray::SharedPtr>(nullptr);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize rt buffer.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBRVelocityController::reference_command_interfaces_() {
  try {
    for (auto &command_interface : command_interfaces_) {
      if (command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        position_command_interfaces_.emplace_back(std::ref(command_interface));
      }
    }
    if (position_command_interfaces_.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d position command interfaces, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, position_command_interfaces_.size());
      return false;
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
      if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        position_state_interfaces_.emplace_back(std::ref(state_interface));
      }
      if (state_interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY) {
        velocity_state_interfaces_.emplace_back(std::ref(state_interface));
      }
    }
    if (position_state_interfaces_.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d position state interfaces, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, position_state_interfaces_.size());
      return false;
    }
    if (velocity_state_interfaces_.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d velocity state interfaces, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, velocity_state_interfaces_.size());
      return false;
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

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_velocity_controllers::LBRVelocityController,
                       controller_interface::ControllerInterface)
