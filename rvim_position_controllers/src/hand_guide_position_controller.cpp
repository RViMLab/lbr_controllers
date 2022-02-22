#include <rvim_position_controllers/hand_guide_position_controller.hpp>

namespace rvim_position_controllers {

    controller_interface::return_type HandGuidePositionController::init(const std::string& controller_name) {
        auto ret = ControllerInterface::init(controller_name);
        if (ret != controller_interface::return_type::OK) {
            return ret;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration HandGuidePositionController::command_interface_configuration() const {
        RCLCPP_INFO(node_->get_logger(), "Command interface configure called.");
        controller_interface::InterfaceConfiguration command_interface_configuration;
        command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joint_name: joint_names_) {
            command_interface_configuration.names.push_back(joint_name + "/" + command_interface_name_);
        }
        return command_interface_configuration;
    }

    controller_interface::InterfaceConfiguration HandGuidePositionController::state_interface_configuration() const {
        RCLCPP_INFO(node_->get_logger(), "State interface configure called.");
        controller_interface::InterfaceConfiguration state_interface_configuration;
        state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joint_name: joint_names_) {
            for (const auto& state_interface: state_interface_names_) {
                state_interface_configuration.names.push_back(joint_name + "/" + state_interface);
            }
        }
        return state_interface_configuration;
    }

    CallbackReturn HandGuidePositionController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // configure interfaces
        if (!node_->get_parameter("command_interface", command_interface_name_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load command_interface parameter");
            return CallbackReturn::ERROR;
        };
        if (command_interface_name_ != hardware_interface::HW_IF_POSITION) {
            RCLCPP_ERROR(node_->get_logger(), "Expected '%s' command interface, got '%s'.", hardware_interface::HW_IF_POSITION, command_interface_name_.c_str());
            return CallbackReturn::ERROR;
        }
        if (!node_->get_parameter("state_interfaces", state_interface_names_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load state_interfaces parameter.");
            return CallbackReturn::ERROR;
        };
        if (!(
            (std::find(state_interface_names_.begin(), state_interface_names_.end(), hardware_interface::HW_IF_POSITION) != state_interface_names_.end()) &&
            (std::find(state_interface_names_.begin(), state_interface_names_.end(), HW_IF_EXTERNAL_TORQUE) != state_interface_names_.end())
        )) {
            RCLCPP_ERROR(node_->get_logger(), "Did not find expected '%s' and '%s' state interfaces.", hardware_interface::HW_IF_POSITION, HW_IF_EXTERNAL_TORQUE);
            return CallbackReturn::ERROR;
        }

        // get joints
        joint_names_ = node_->get_parameter("joints").as_string_array();
        if (joint_names_.empty()) {
            RCLCPP_ERROR(node_->get_logger(), "The joints parameter was empty.");
            return CallbackReturn::ERROR;
        }

        // load robot description, see http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
        std::string robot_description_string = node_->get_parameter("robot_description").as_string();
        if (!kdl_parser::treeFromString(robot_description_string, tree_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree from robot description.");
            return CallbackReturn::ERROR;
        }

        // configure kdl chain by defining root and tip from tree
        if (!node_->get_parameter("chain_root", chain_root_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load chain_root parameter, got '%s'.", chain_root_.c_str());
            return CallbackReturn::ERROR;
        };
        if (!node_->get_parameter("chain_tip", chain_tip_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load chain_tip parameter, got '%s'.", chain_tip_.c_str());
            return CallbackReturn::ERROR;
        };

        if (!tree_.getChain(chain_root_, chain_tip_, kdl_chain_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to extract kdl chain with root '%s' and tip '%s'.", chain_root_.c_str(), chain_tip_.c_str());
            return CallbackReturn::ERROR;
        };

        // allocate joint angles and Jacobian
        q_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());

        // create Jacobian solver from kdl chain
        jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdl_chain_);

        // create quadratic problem
        RCLCPP_ERROR(node_->get_logger(), "Building QP...");
        qp_ = std::make_unique<qpOASES::SQProblem>(kdl_chain_.getNrOfJoints(), 6, qpOASES::HST_IDENTITY);
        RCLCPP_ERROR(node_->get_logger(), "Built QP...");

        // H_ = RowMajorMatrixXd::Identity(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
        // g_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
        // lb_ = Eigen::VectorXd::Constant(kdl_chain_.getNrOfJoints(), std::numeric_limits<double>::min());
        // ub_ = Eigen::VectorXd::Constant(kdl_chain_.getNrOfJoints(), std::numeric_limits<double>::max());
        // lba_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
        // uba_ = Eigen::Ref<Eigen::VectorXd>(lba_);  // lba_ = uba_ (equality constraints)

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HandGuidePositionController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // get a sorted reference
        for (auto& state_interface: state_interfaces_) {
            if (state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION) {
                position_interfaces_.emplace_back(std::ref(state_interface));
            } else if (state_interface.get_interface_name() == HW_IF_EXTERNAL_TORQUE) {
                external_torque_interfaces_.emplace_back(std::ref(state_interface));
            }
            else {
                RCLCPP_ERROR(node_->get_logger(), "Provided with wrong state interface '%s' for joint %s.", state_interface.get_interface_name().c_str(), state_interface.get_name().c_str());
                return CallbackReturn::ERROR;
            }
        }

        // check size of sorted references
        if (joint_names_.size() != position_interfaces_.size()) {
            RCLCPP_ERROR(node_->get_logger(), "Expected %zu %s interfaces, got %zu.", joint_names_.size(), hardware_interface::HW_IF_POSITION, position_interfaces_.size());
            return CallbackReturn::ERROR;
        }
        if (joint_names_.size() != external_torque_interfaces_.size()) {
            RCLCPP_ERROR(node_->get_logger(), "Expected %zu %s interfaces, got %zu", joint_names_.size(), HW_IF_EXTERNAL_TORQUE, external_torque_interfaces_.size());
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn HandGuidePositionController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(node_->get_logger(), "On deactivate called.");
        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type HandGuidePositionController::update() {

        // auto f_ext = J_pseudo_inv.transpose()*;

        // min x
        // s.t. Ax = b (equality constraint)

        // s.t. J^#^T tau ~ f_ext



        // f_ext = J^#^T tau
        // avoid f_ext via dx
        // dq = J^# dx

        // J^# A_w(w* - w) = dq -> (J^# A_w)^# dq = (w* - w) = J^#^T tau_ext
        // where A_w (w* - w) = dx with w wrench, w* = 0, w = J^#^T tau_ext
        


        // ********************************************************
        // ******************************************qp formulation
        // ********************************************************
        // pseudo inverse rules https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
        // min dq
        // s.t. (J^# A_w)^# dq = (w* - w) = J^#^T tau_ext
        //       J K dq = J^#^T tau_ext




        // build qp
        // H = identity
        // g = 0
        // A ~ Jac
        // lb, ub, -inf, +inf
        // lbA, upA, lbA = ubA (equality)

        // lba_[0] += 1;

        // if (lba_ != uba_) {
        //     RCLCPP_ERROR(node_->get_logger(), "Found different values for lower and upper bounds lbA, and ubA.");
        //     return controller_interface::return_type::ERROR;
        // }

        // find solution given cpu constraints
        if (!qp_init_) {
            // qp_->init(

            // );
            qp_init_ = true;
        } else {
            // qp_->hotstart(

            // );
        }

        // update joint states

        // set update
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            command_interfaces_[i].set_value(position_interfaces_[i].get().get_value());
        }


        

        // ********************************************************
        // ******************************************qp formulation
        // ********************************************************






        // ********************************************************
        // ********************************wrench avoidance control
        // ********************************************************

        // // compute jacobian
        // for (std::size_t i = 0; i < joint_names_.size(); i++) {
        //     q_.data[i] = position_interfaces_[i].get().get_value();
        // }

        // jac_solver_->JntToJac(q_, J_);
        // auto J = J_.data;
        // auto J_pseudo_inv = dampedLeastSquares(J);
        // Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(J.cols());
        // Eigen::VectorXd dx = Eigen::VectorXd::Zero(J.rows());
        // for (std::size_t i = 0; i < joint_names_.size(); i++) {
        //     tau_ext[i] = external_torque_interfaces_[i].get().get_value();
        // }
        // RCLCPP_INFO(node_->get_logger(), "text: %f, %f, %f, %f, %f, %f, %f", tau_ext[0], tau_ext[1], tau_ext[2], tau_ext[3], tau_ext[4], tau_ext[5], tau_ext[6]);
        // auto f_ext = J_pseudo_inv.transpose()*tau_ext;

        // // determine desired update
        // int j = 0;
        // for (int i = 0; i < 3; i++) {
        //     if (std::abs(f_ext[j]) > th_f_) (f_ext[j] > 0. ? dx[j] = 0.1 : dx[j] = -0.1);
        //     j++;
        // }
        // for (int i = 0; i < 3; i++) {
        //     if (std::abs(f_ext[j]) > th_d_) (f_ext[j] > 0. ? dx[j] = 1.0 : dx[j] = -1.0);
        //     j++;
        // }
        // RCLCPP_INFO(node_->get_logger(), "fext: %f, %f, %f, %f, %f, %f", f_ext[0], f_ext[1], f_ext[2], f_ext[3], f_ext[4], f_ext[5]);

        // auto dq = 0.01*J_pseudo_inv*dx;

        // // set update
        // for (std::size_t i = 0; i < joint_names_.size(); i++) {
        //     dq_[i] = alpha_*dq_[i] + (1-alpha_)*dq[i];
        //     command_interfaces_[i].set_value(position_interfaces_[i].get().get_value() + dq_[i]);
        // }

        // ********************************************************
        // ********************************wrench avoidance control
        // ********************************************************

        





        // ********************************************************
        // ******************************* torque avoidance control
        // ********************************************************

        // // noise filter!
        // // xi+1 = alpha*xi-1 + (1-alpha)*xi;

        // // safety set 0!
        // std::vector<double> update(prev_update_.size(), 0.);
        // for (std::size_t i = 0; i<command_interfaces_.size(); i++) {
        //     const auto& et = external_torque_interfaces_[i].get().get_value();

        //     double sign = 0.;
        //     if (std::abs(et) > 2.) {
        //         sign = std::signbit(et) ? -1. : 1.;
        //     } 

        //     update[i] = alpha_*prev_update_[i] + (1.-alpha_)*sign*0.002;
        //     prev_update_[i] = update[i];

        //     command_interfaces_[i].set_value(position_interfaces_[i].get().get_value() + update[i]);
        // }
        // ********************************************************
        // ******************************* torque avoidance control
        // ********************************************************

        // RCLCPP_INFO(node_->get_logger(), "position: %f, external torque: %f", position_interfaces_[0].get().get_value(), external_torque_interfaces_[0].get().get_value());

        return controller_interface::return_type::OK;
    }

}  // end of namespace rvim_position_controllers

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
    rvim_position_controllers::HandGuidePositionController,
    controller_interface::ControllerInterface
)
