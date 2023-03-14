#ifndef LBR_POSITION_CONTROLLERS__LBR_SWITCH_CONTROLLER_HPP_
#define LBR_POSITION_CONTROLLERS__LBR_SWITCH_CONTROLLER_HPP_

#include <Eigen/Core>
#include <array>
#include <atomic>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_toolbox/filters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kinematics_interface/kinematics_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "lbr_fri_ros2/lbr.hpp"
#include "lbr_hardware_interface/lbr_hardware_interface_type_values.hpp"

#include "lbr_position_controllers/pseudo_inverse.hpp"

#include "lbr_controllers_msgs/srv/control_mode.hpp"

namespace lbr_position_controllers {

class LBRSwitchController : public controller_interface::ControllerInterface {
  enum CONTROL_MODE {
    DISABLED,
    ADMITTANCE,
    CONFIGURE,
  };

public:
  LBRSwitchController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

protected:
  bool read_parameters_();
  bool reference_command_interfaces_();
  bool reference_state_interfaces_();
  bool clear_command_interfaces_();
  bool clear_state_interfaces_();
  bool initialize_kinematics_();

  bool admittance_control_(const double &dt);
  bool configure_control_(const double &dt);

  void control_mode_service_callback_(
      const lbr_controllers_msgs::srv::ControlMode::Request::SharedPtr request,
      lbr_controllers_msgs::srv::ControlMode::Response::SharedPtr response);

  rclcpp::Service<lbr_controllers_msgs::srv::ControlMode>::SharedPtr control_mode_service_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cartesian_gain_cb_handle_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_torque_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      force_torque_realtime_publisher_;
  std::array<std::string, 2> state_interface_names_;

  std::vector<std::string> joint_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      external_torque_state_interfaces_;
  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      time_interface_map_;

  std::string kinematics_plugin_name_, end_effector_name_;
  std::shared_ptr<pluginlib::ClassLoader<kinematics_interface::KinematicsInterface>>
      kinematics_loader_;
  std::unique_ptr<kinematics_interface::KinematicsInterface> kinematics_;
  Eigen::Vector<double, lbr_fri_ros2::LBR::JOINT_DOF> positions_, external_torques_,
      desired_velocity_, position_increment_;
  std::vector<double> cartesian_sensitivity_offset_, joint_sensitivty_offset_;
  Eigen::Vector<double, lbr_fri_ros2::LBR::CARTESIAN_DOF> force_torque_;
  geometry_msgs::msg::WrenchStamped force_torque_msg_;
  double position_alpha_, cartesian_external_torque_alpha_, joint_external_torque_alpha_,
      singular_damping_;
  Eigen::Vector<double, lbr_fri_ros2::LBR::CARTESIAN_DOF> cartesian_gains_;
  Eigen::Vector<double, lbr_fri_ros2::LBR::JOINT_DOF> cartesian_joint_gains_;
  std::vector<double> joint_joint_gains_;
  Eigen::Matrix<double, lbr_fri_ros2::LBR::CARTESIAN_DOF, Eigen::Dynamic> jacobian_;
  Eigen::Matrix<double, Eigen::Dynamic, lbr_fri_ros2::LBR::CARTESIAN_DOF> jacobian_inv_;

  std::atomic<uint8_t> control_mode_;
};

} // end of namespace lbr_position_controllers
#endif // LBR_POSITION_CONTROLLERS__LBR_SWITCH_CONTROLLER_HPP_
