#ifndef TMP_NAME_H_
#define TMP_NAME_H_

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "control_toolbox/filters.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lbr_fri_ros2/lbr.hpp"

namespace lbr_velocity_controllers {
class LBRVelocityController : public controller_interface::ControllerInterface {
public:
  LBRVelocityController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previoud_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time,
                                           const rclcpp::Duration &period) override;

protected:
  bool declare_parameters_();
  bool read_parameters_();
  bool reference_command_interfaces_();
  bool reference_state_interfaces_();
  bool clear_command_interfaces_();
  bool clear_state_interfaces_();

  std::shared_ptr<realtime_tools::RealtimeBuffer<std_msgs::msg::Float64MultiArray::SharedPtr>>
      joint_velocity_command_rt_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      joint_velocity_command_subscription_;

  std::vector<std::string> joint_names_;
  const std::array<std::string, 1> command_interface_names_;
  const std::array<std::string, 2> state_interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
      position_command_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      velocity_state_interfaces_;
};
} // end of namespace lbr_velocity_controllers
#endif
