#include "lbr_broadcasters/lbr_virtual_force_torque_broadcaster.hpp"

namespace lbr_broadcasters {

LBRVirtualForceTorqueBroadcaster::LBRVirtualForceTorqueBroadcaster()
    : force_torque_publisher_(nullptr), force_torque_realtime_publisher_(nullptr),
      state_interface_names_{lbr_hardware_interface::HW_IF_EXTERNAL_TORQUE,
                             hardware_interface::HW_IF_POSITION},
      sensitivity_offset_(lbr_fri_ros2::LBR::CARTESIAN_DOF,
                          std::numeric_limits<double>::infinity()),
      singular_damping_(std::numeric_limits<double>::infinity()),
      jacobian_(lbr_fri_ros2::LBR::CARTESIAN_DOF, lbr_fri_ros2::LBR::JOINT_DOF) {
  positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
  external_torques_.setConstant(std::numeric_limits<double>::quiet_NaN());
  force_torque_.setConstant(std::numeric_limits<double>::quiet_NaN());
}

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

controller_interface::CallbackReturn LBRVirtualForceTorqueBroadcaster::on_init() {
  force_torque_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/force_torque", rclcpp::SystemDefaultsQoS());
  force_torque_realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>(
          force_torque_publisher_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!read_parameters_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!initialize_kinematics_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!reference_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn LBRVirtualForceTorqueBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  if (!clear_state_interfaces_()) {
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
LBRVirtualForceTorqueBroadcaster::update(const rclcpp::Time & /*time*/,
                                         const rclcpp::Duration & /*period*/) {
  for (std::size_t i = 0; i < lbr_fri_ros2::LBR::JOINT_DOF; ++i) {
    positions_[i] = position_state_interfaces_[i].get().get_value();
    external_torques_[i] = external_torque_state_interfaces_[i].get().get_value();
  }

  // calculate the Jacobian
  kinematics_->calculate_jacobian(positions_, end_effector_name_, jacobian_);

  // calculate the force torque
  force_torque_ =
      damped_leaste_squares(jacobian_, singular_damping_).transpose() * external_torques_;

  force_torque_ = force_torque_.NullaryExpr([this](const Eigen::Index &i) {
    double sign = std::signbit(force_torque_[i]) ? -1.0 : 1.0;
    return std::abs(force_torque_[i]) < sensitivity_offset_[i]
               ? 0.0
               : sign * (std::abs(force_torque_[i]) - sensitivity_offset_[i]);
  });

  // publish force torque
  if (force_torque_realtime_publisher_->trylock()) {
    force_torque_msg_.header.frame_id = end_effector_name_;
    // force_torque_msg_.header.stamp.
    force_torque_msg_.wrench.force.x = force_torque_[0];
    force_torque_msg_.wrench.force.y = force_torque_[1];
    force_torque_msg_.wrench.force.z = force_torque_[2];
    force_torque_msg_.wrench.torque.x = force_torque_[3];
    force_torque_msg_.wrench.torque.y = force_torque_[4];
    force_torque_msg_.wrench.torque.z = force_torque_[5];
    force_torque_realtime_publisher_->msg_ = force_torque_msg_;
    force_torque_realtime_publisher_->unlockAndPublish();
  }

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
      RCLCPP_ERROR(get_node()->get_logger(), "Expected %lu sensitivity offset values, got %d.",
                   sensitivity_offset_.size(), lbr_fri_ros2::LBR::CARTESIAN_DOF);
      return false;
    }
    if (!get_node()->get_parameter("singular_damping", singular_damping_)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to retrieve singular damping parameter.");
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
  positions_.setConstant(std::numeric_limits<double>::quiet_NaN());
  external_torques_.setConstant(std::numeric_limits<double>::quiet_NaN());
  force_torque_.setConstant(std::numeric_limits<double>::quiet_NaN());
  return true;
}

bool LBRVirtualForceTorqueBroadcaster::initialize_kinematics_() {
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

} // end of namespace lbr_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_broadcasters::LBRVirtualForceTorqueBroadcaster,
                       controller_interface::ControllerInterface)
