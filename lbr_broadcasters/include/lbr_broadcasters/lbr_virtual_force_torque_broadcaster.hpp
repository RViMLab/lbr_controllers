#ifndef LBR_BROADCASTERS__LBR_VIRTUAL_FORCE_TORQUE_BROADCASTER_HPP_
#define LBR_BROADCASTERS__LBR_VIRTUAL_FORCE_TORQUE_BROADCASTER_HPP_

#include <array>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include "lbr_fri_ros2/lbr.hpp"
#include "lbr_hardware_interface/lbr_hardware_interface_type_values.hpp"

namespace lbr_broadcasters {

class LBRVirtualForceTorqueBroadcaster : public controller_interface::ControllerInterface {
public:
  LBRVirtualForceTorqueBroadcaster();

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
  bool reference_state_interfaces_();
  bool clear_state_interfaces_();

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr virtual_force_torque_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>>
      virtual_force_torque_realtime_publisher_;
  std::array<std::string, 2> state_interface_names_;
  std::vector<std::string> joint_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      position_state_interfaces_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
      external_torque_state_interfaces_;
};

} // end of namespace lbr_broadcasters
#endif
