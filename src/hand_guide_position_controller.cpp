#include <rvim_ros2_controllers_experimental/hand_guide_position_controller.hpp>

namespace rvim_ros2_controllers_experimental {

    controller_interface::return_type HandGuidePositionController::init(const std::string& controller_name) {
        auto ret = ControllerInterface::init(controller_name);  // required!
        if (ret != controller_interface::return_type::OK) {
            return ret;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration HandGuidePositionController::command_interface_configuration() const {
        // RCLCPP_INFO(this->get_node()->get_logger(), "Command interface configure called.");
        return controller_interface::InterfaceConfiguration();
    }

    controller_interface::InterfaceConfiguration HandGuidePositionController::state_interface_configuration() const {
        // RCLCPP_INFO(this->get_node()->get_logger(), "State interface configure called.");
        return controller_interface::InterfaceConfiguration();
    }

    CallbackReturn HandGuidePositionController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // RCLCPP_INFO(this->get_node()->get_logger(), "On configure called.");
        // print robot description?
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HandGuidePositionController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_node()->get_logger(), "On activate called.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HandGuidePositionController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(this->get_node()->get_logger(), "On deactivate called.");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type HandGuidePositionController::update() {
        RCLCPP_INFO(this->get_node()->get_logger(), "Update called.");
        return controller_interface::return_type::OK;
    }

}  // end of namespace rvim_ros2_controllers_experimental

// create controller, load controller, print hello world


// J_ee tau = f_ext ~ dx, if f > f_th... J_ee# J_ee tau, simply -tau?
// J_cam H = dx, if f < f_th, view ~ f_ext
// J_ns tau
// J \in 6x7, J_nx = (1-J#J) \in 7x7
//
// control rotation via homography in translational nullspace? ie send omega via pub sub

// J      dq = dx
// J^T f_ext = tau
// ->
// dq = (J^T#K)#Gh
// K stiffness, G gain, h wrench == 0
// see https://mediatum.ub.tum.de/doc/1244171/677139536966.pdf
// and https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6696601

// joint limits
// omega
// end-effector force

//

// min dq,    s.t. J   dq    = dx
//     f_ext       J^t f_ext = 0 

// how to obtain Jacobian?
// how to link qpoases?

// input:
// tau_ext (state interface), dx (homography in vision node)

// control (joint impedance control mode with target position):
// qp with hot start
// q_i-1 + dt dq = q_i (target)



#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rvim_ros2_controllers_experimental::HandGuidePositionController,
    controller_interface::ControllerInterface
)
