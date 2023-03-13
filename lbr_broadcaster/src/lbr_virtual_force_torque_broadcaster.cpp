#include "lbr_broadcasters/lbr_virtual_force_torque_broadcaster.hpp"

namespace lbr_broadcasters {

LBRVirtualForceTorqueBroadcaster::LBRVirtualForceTorqueBroadcaster()
    : virtual_force_torque_publisher_(nullptr), virtual_force_torque_realtime_publisher_(nullptr)
{}

controller_interface::InterfaceConfiguration
LBRVirtualForceTorqueBroadcaster::command_interface_configuration() const {}

controller_interface::InterfaceConfiguration
LBRVirtualForceTorqueBroadcaster::state_interface_configuration() const {}

controller_interface::CallbackReturn LBRVirtualForceTorqueBroadcaster::on_init() {}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_configure(const rclcpp_lifecycle::State &previous_state) {}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_activate(const rclcpp_lifecycle::State &previous_state) {}

controller_interface::CallbackReturn
LBRVirtualForceTorqueBroadcaster::on_deactivate(const rclcpp_lifecycle::State &previous_state) {}

controller_interface::return_type
LBRVirtualForceTorqueBroadcaster::update(const rclcpp::Time &time, const rclcpp::Duration &period) {
}

} // end of namespace lbr_broadcasters

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(lbr_broadcasters::LBRVirtualForceTorqueBroadcaster,
                       controller_interface::ControllerInterface)
