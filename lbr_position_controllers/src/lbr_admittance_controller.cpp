#include "lbr_position_controllers/lbr_admittance_controller.hpp"

namespace lbr_position_controllers {

LBRAdmittanceController::LBRAdmittanceController()
    : force_torque_publisher_(nullptr), force_torque_realtime_publisher_(nullptr),
      state_interface_names_{lbr_hardware_interface::HW_IF_EXTERNAL_TORQUE,
                             hardware_interface::HW_IF_POSITION},
      sensitivity_offset_(lbr_fri_ros2::LBR::CARTESIAN_DOF,
                          std::numeric_limits<double>::infinity()),
      position_alpha_(0.), external_torque_alpha_(0.),
      singular_damping_(std::numeric_limits<double>::infinity()),
      jacobian_(lbr_fri_ros2::LBR::CARTESIAN_DOF, lbr_fri_ros2::LBR::JOINT_DOF),
      jacobian_inv_(lbr_fri_ros2::LBR::JOINT_DOF, lbr_fri_ros2::LBR::CARTESIAN_DOF) {
  positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
  external_torques_.setConstant(std::numeric_limits<double>::quiet_NaN());
  force_torque_.setZero();
  desired_velocity_.setZero();
  position_increment_.setZero();
  cartesian_gains_.setZero();
  joint_gains_.setZero();
}

controller_interface::InterfaceConfiguration
LBRAdmittanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    interface_configuration.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }
  return interface_configuration;
}

controller_interface::InterfaceConfiguration
LBRAdmittanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration interface_configuration;
  interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &joint_name : joint_names_) {
    for (const auto &state_interface_name : state_interface_names_) {
      interface_configuration.names.push_back(joint_name + "/" + state_interface_name);
    }
  }
  interface_configuration.names.push_back(std::string("lbr_fri_sensor") +
                                          "/" + // TODO: fix this hard-coded sensor name
                                          lbr_hardware_interface::HW_IF_TIME_STAMP_SEC);
  interface_configuration.names.push_back(std::string("lbr_fri_sensor") + "/" +
                                          lbr_hardware_interface::HW_IF_TIME_STAMP_NANO_SEC);
  return interface_configuration;
}

controller_interface::CallbackReturn LBRAdmittanceController::on_init() {
  force_torque_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/force_torque", rclcpp::SystemDefaultsQoS());
  force_torque_realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
          force_torque_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRAdmittanceController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!read_parameters_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!initialize_kinematics_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRAdmittanceController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!clear_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRAdmittanceController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!clear_command_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type LBRAdmittanceController::update(const rclcpp::Time & /*time*/,
                                                                  const rclcpp::Duration &period) {
  for (std::size_t i = 0; i < lbr_fri_ros2::LBR::JOINT_DOF; ++i) {
    if (std::isnan(positions_[i])) {
      positions_[i] = position_state_interfaces_[i].get().get_value();
    } else {
      positions_[i] = filters::exponentialSmoothing(position_state_interfaces_[i].get().get_value(),
                                                    positions_[i], position_alpha_);
    }
    if (std::isnan(external_torques_[i])) {
      external_torques_[i] = external_torque_state_interfaces_[i].get().get_value();
    } else {
      external_torques_[i] =
          filters::exponentialSmoothing(external_torque_state_interfaces_[i].get().get_value(),
                                        external_torques_[i], external_torque_alpha_);
    }
  }

  // calculate the Jacobian
  kinematics_->calculate_jacobian(positions_, end_effector_name_, jacobian_);

  // calculate the force torque
  jacobian_inv_ = damped_leaste_squares(jacobian_, singular_damping_);
  force_torque_ = jacobian_inv_.transpose() * external_torques_;
  force_torque_ = force_torque_.NullaryExpr([this](const Eigen::Index &i) {
    double sign = std::signbit(force_torque_[i]) ? -1.0 : 1.0;
    return std::abs(force_torque_[i]) < sensitivity_offset_[i]
               ? 0.0
               : sign * (std::abs(force_torque_[i]) - sensitivity_offset_[i]);
  });

  // publish force torque
  if (force_torque_realtime_publisher_->trylock()) {
    force_torque_msg_.header.frame_id = end_effector_name_;
    force_torque_msg_.header.stamp.sec = static_cast<int32_t>(
        time_interface_map_.at(lbr_hardware_interface::HW_IF_TIME_STAMP_SEC).get().get_value());
    force_torque_msg_.header.stamp.nanosec = static_cast<uint32_t>(
        time_interface_map_.at(lbr_hardware_interface::HW_IF_TIME_STAMP_NANO_SEC)
            .get()
            .get_value());
    force_torque_msg_.wrench.force.x = force_torque_[0];
    force_torque_msg_.wrench.force.y = force_torque_[1];
    force_torque_msg_.wrench.force.z = force_torque_[2];
    force_torque_msg_.wrench.torque.x = force_torque_[3];
    force_torque_msg_.wrench.torque.y = force_torque_[4];
    force_torque_msg_.wrench.torque.z = force_torque_[5];
    force_torque_realtime_publisher_->msg_ = force_torque_msg_;
    force_torque_realtime_publisher_->unlockAndPublish();
  }

  // compute position update
  desired_velocity_ =
      joint_gains_.asDiagonal() * jacobian_inv_ * cartesian_gains_.asDiagonal() * force_torque_;

  // smooth update
  for (uint8_t i = 0; i < lbr_fri_ros2::LBR::JOINT_DOF; ++i) {
    position_increment_[i] = filters::exponentialSmoothing(desired_velocity_[i] * period.seconds(),
                                                           position_increment_[i], position_alpha_);
    position_command_interfaces_[i].get().set_value(positions_[i] + position_increment_[i]);
  }

  return controller_interface::return_type::OK;
}

bool LBRAdmittanceController::read_parameters_() {
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
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d joint names, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, joint_names_.size());
      return false;
    }
    if (!get_node()->get_parameter("kinematics_plugin", kinematics_plugin_name_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve kinematics plugin parameter.");
      return false;
    }
    if (!get_node()->get_parameter("end_effector_name", end_effector_name_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve end effector name parameter.");
      return false;
    }
    if (!get_node()->get_parameter("sensitivity_offset", sensitivity_offset_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve sensitivity offset parameter.");
      return false;
    }
    if (sensitivity_offset_.size() != lbr_fri_ros2::LBR::CARTESIAN_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d sensitivity offset values, got %lu.",
                   lbr_fri_ros2::LBR::CARTESIAN_DOF, sensitivity_offset_.size());
      return false;
    }
    if (!get_node()->get_parameter("position_alpha", position_alpha_)) {
      position_alpha_ = 0.;
      RCLCPP_WARN(get_node()->get_logger(),
                  "Failed to retrieve position alpha parameter. Using default value (0.0).");
      return false;
    }
    if (!get_node()->get_parameter("external_torque_alpha", external_torque_alpha_)) {
      external_torque_alpha_ = 0.;
      RCLCPP_WARN(get_node()->get_logger(),
                  "Failed to retrieve external torque alpha parameter. Using default value (0.0).");
      return false;
    }
    if (!get_node()->get_parameter("singular_damping", singular_damping_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve singular damping parameter.");
      return false;
    }
    std::vector<double> cartesian_gains(lbr_fri_ros2::LBR::CARTESIAN_DOF, 0.);
    if (!get_node()->get_parameter("cartesian_gains", cartesian_gains)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve cartesian gains parameter.");
      return false;
    }
    if (cartesian_gains_.size() != lbr_fri_ros2::LBR::CARTESIAN_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d cartesian gains, got %lu.",
                   lbr_fri_ros2::LBR::CARTESIAN_DOF, cartesian_gains_.size());
      return false;
    }
    cartesian_gains_ =
        Eigen::Vector<double, lbr_fri_ros2::LBR::CARTESIAN_DOF>::Map(cartesian_gains.data());
    std::vector<double> joint_gains(lbr_fri_ros2::LBR::JOINT_DOF, 0.);
    if (!get_node()->get_parameter("joint_gains", joint_gains)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve joint gains parameter.");
      return false;
    }
    if (joint_gains.size() != lbr_fri_ros2::LBR::JOINT_DOF) {
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %d joint gains, got %lu.",
                   lbr_fri_ros2::LBR::JOINT_DOF, joint_gains.size());
      return false;
    }
    joint_gains_ = Eigen::Vector<double, lbr_fri_ros2::LBR::JOINT_DOF>::Map(joint_gains.data());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read parameters.\n%s.", e.what());
    return false;
  }
  return true;
}

bool LBRAdmittanceController::reference_command_interfaces_() {
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

bool LBRAdmittanceController::reference_state_interfaces_() {
  try {
    for (auto &state_interface : state_interfaces_) {
      if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
        position_state_interfaces_.emplace_back(std::ref(state_interface));
      }
      if (state_interface.get_interface_name() == lbr_hardware_interface::HW_IF_EXTERNAL_TORQUE) {
        external_torque_state_interfaces_.emplace_back(std::ref(state_interface));
      }
      if (state_interface.get_interface_name() == lbr_hardware_interface::HW_IF_TIME_STAMP_SEC ||
          state_interface.get_interface_name() ==
              lbr_hardware_interface::HW_IF_TIME_STAMP_NANO_SEC) {
        time_interface_map_.emplace(state_interface.get_interface_name(),
                                    std::ref(state_interface));
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

bool LBRAdmittanceController::clear_command_interfaces_() {
  position_command_interfaces_.clear();
  return true;
}

bool LBRAdmittanceController::clear_state_interfaces_() {
  position_state_interfaces_.clear();
  external_torque_state_interfaces_.clear();
  time_interface_map_.clear();
  positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
  external_torques_.setConstant(std::numeric_limits<double>::quiet_NaN());
  force_torque_.setZero();
  return true;
}

bool LBRAdmittanceController::initialize_kinematics_() {
  try {
    kinematics_loader_ =
        std::make_shared<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>(
            "kinematics_interface", "kinematics_interface::KinematicsInterface");
    kinematics_ = std::unique_ptr<kinematics_interface::KinematicsInterface>(
        kinematics_loader_->createUnmanagedInstance(kinematics_plugin_name_));
    kinematics_->initialize(get_node()->get_node_parameters_interface(), end_effector_name_);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize kinematics.\n%s", e.what());
    return false;
  }
  return true;
}

} // end of namespace lbr_position_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_position_controllers::LBRAdmittanceController,
                       controller_interface::ControllerInterface)
