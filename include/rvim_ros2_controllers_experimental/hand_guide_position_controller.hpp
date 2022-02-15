#include <vector>
#include <string>

#include <controller_interface/controller_interface.hpp>


// follow http://control.ros.org/ros2_controllers/doc/writing_new_controller.html
// see forward command controller for an example: https://github.com/ros-controls/ros2_controllers/blob/foxy/forward_command_controller/include/forward_command_controller/forward_command_controller.hpp

namespace rvim_ros2_controllers_experimental {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HandGuidePositionController : public controller_interface::ControllerInterface {
        public:
            HandGuidePositionController();


            controller_interface::return_type init(const std::string& controller_name) override;
            controller_interface::InterfaceConfiguration command_interface_configuration() override;
            controller_interface::InterfaceConfiguration state_interface_configuration() override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            controller_interface::return_type update() override;

        protected:
            std::vector<std::string> joint_names_;
            std::vector<std::string> interface_names_;  // for parsing in yaml file (external_torque & position required)
    };

}  // end of namespace rvim_ros2_controllers_experimental

