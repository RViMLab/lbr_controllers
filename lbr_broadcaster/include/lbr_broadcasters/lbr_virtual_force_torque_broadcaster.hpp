#ifndef LBR_BROADCASTERS__LBR_VIRTUAL_FORCE_TORQUE_BROADCASTER_HPP_
#define LBR_BROADCASTERS__LBR_VIRTUAL_FORCE_TORQUE_BROADCASTER_HPP_

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

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
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr virtual_force_torque_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Wrench>>
      virtual_force_torque_realtime_publisher_;
};

} // end of namespace lbr_broadcasters
#endif
