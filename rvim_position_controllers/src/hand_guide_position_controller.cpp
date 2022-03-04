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

        // // create quadratic problem
        // options_.printLevel = qpOASES::PL_LOW;
        // qp_ = std::make_unique<qpOASES::SQProblem>(kdl_chain_.getNrOfJoints(), 6, qpOASES::HST_IDENTITY);
        // qp_->setOptions(options_);

        // H_ = RowMajorMatrixXd::Identity(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints());
        // A_ = RowMajorMatrixXd::Zero(J_.data.rows(), J_.data.cols());
        // g_ = Eigen::RowVectorXd::Zero(kdl_chain_.getNrOfJoints());
        // lb_ = Eigen::RowVectorXd::Constant(kdl_chain_.getNrOfJoints(), std::numeric_limits<double>::lowest());
        // ub_ = Eigen::RowVectorXd::Constant(kdl_chain_.getNrOfJoints(), std::numeric_limits<double>::max());
        // lba_ = Eigen::RowVectorXd::Constant(6, std::numeric_limits<double>::lowest());
        // uba_ = Eigen::RowVectorXd::Constant(6, std::numeric_limits<double>::max());

        // 

        nwsr_ = std::numeric_limits<int>::max();
        cputime_ = 0.005;  // 100 hz

        // dq_ = Eigen::RowVectorXd::Zero(kdl_chain_.getNrOfJoints());
        std::cout << "dq: " << dq_ << std::endl;


        // OSQP
        H_osqp_.resize(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints()); H_osqp_.setIdentity();
        A_osqp_.resize(J_.data.rows() + J_.data.cols(), J_.data.cols()); A_osqp_.setZero();
        g_osqp_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
        lb_osqp_ = Eigen::VectorXd::Constant(kdl_chain_.getNrOfJoints() + J_.data.cols(), std::numeric_limits<double>::lowest());
        ub_osqp_ = Eigen::VectorXd::Constant(kdl_chain_.getNrOfJoints() + J_.data.cols(), std::numeric_limits<double>::max());

        // identity for bounds on dq_osqp_
        // sparse matrices: https://eigen.tuxfamily.org/dox/group__SparseQuickRefPage.html
        // manipulation: https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
        for (int i = 0; i < J_.data.cols(); i ++) {
            A_osqp_.insert(J_.data.rows()+i, i) = 1.;  // sparse matrix mostly empty, hence inserion required
        }

        dq_osqp_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());

        qp_osqp_ = std::make_unique<OsqpEigen::Solver>();
        qp_osqp_->settings()->setWarmStart(true);
        qp_osqp_->data()->setNumberOfVariables(kdl_chain_.getNrOfJoints());
        qp_osqp_->data()->setNumberOfConstraints(6);

        qp_osqp_->data()->setHessianMatrix(H_osqp_);
        qp_osqp_->data()->setLinearConstraintsMatrix(A_osqp_);
        qp_osqp_->data()->setGradient(g_osqp_);
        qp_osqp_->data()->setLowerBound(lb_osqp_);
        qp_osqp_->data()->setUpperBound(ub_osqp_);

        qp_osqp_->settings()->setMaxIteration(nwsr_);
        qp_osqp_->settings()->setTimeLimit(cputime_);

        dq_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
        std::cout << "dq: " << dq_ << std::endl;

        if (!qp_osqp_->initSolver()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialized solver.");
            return CallbackReturn::ERROR;
        };

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

        // J^# A_w(w* - w) = dq -> (J^# A_w)^# dq = (w* - w) = - J^#^T tau_ext
        // where A_w (w* - w) = dx with w wrench, w* = 0, w = J^#^T tau_ext
        


        // ********************************************************
        // ******************************************qp formulation
        // ********************************************************
        // pseudo inverse rules https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
        // min ||dq||^2_2
        // s.t. (J^# A_w)^# dq = (w* - w) = - J^#^T tau_ext
        //       J K dq = - J^#^T tau_ext = ba_




        // // build qp
        // // H = identity
        // // g = 0
        // // A ~ Jac
        // // lb, ub, -inf, +inf
        // // lbA, upA, lbA = ubA (equality)


        // // qpOASES
        // // compute jacobian
        // for (std::size_t i = 0; i < joint_names_.size(); i++) {
        //     q_.data[i] = position_interfaces_[i].get().get_value();
        // }
        // jac_solver_->JntToJac(q_, J_);

        // auto J = J_.data;
        // auto J_pseudo_inv = dampedLeastSquares(J);
        // Eigen::RowVectorXd tau_ext = Eigen::VectorXd::Zero(J.cols());

        // for (std::size_t i = 0; i < joint_names_.size(); i++) {
        //     tau_ext[i] = external_torque_interfaces_[i].get().get_value();
        // }

        // // threshold tau_ext? 

        // A_ = J;  // 0.01 = K
        // A_.topRows(3) *= 2500.;
        // A_.bottomRows(3) *= 500.;
        // Eigen::RowVectorXd ba = tau_ext*J_pseudo_inv;

        // // threshold noise
        // ba.head(3) = ba.head(3).unaryExpr([](double d){
        //     if (std::abs(d) > 2.) {
        //         // double sign = std::signbit(d) ? -1.:1.;
        //         return d;
        //     } else {
        //         return 0.;
        //     }
        // });
        // ba.tail(3) = ba.tail(3).unaryExpr([](double d){
        //     if (std::abs(d) > 0.5) {
        //         // double sign = std::signbit(d) ? -0.5:0.5;
        //         return d;
        //     } else {
        //         return 0.;
        //     }
        // });

        // std::cout << "ba: " << ba << std::endl;
        // lba_.head(3) = ba.head(3).array() - 1.;  // 1N, 1Nm
        // lba_.tail(3) = ba.tail(3).array() - 0.25;  // 1N, 1Nm
        // uba_.head(3) = ba.head(3).array() + 1.;
        // uba_.tail(3) = ba.tail(3).array() + 0.25;


        // // uba_.setConstant(std::numeric_limits<double>::max()); lba_.setConstant(std::numeric_limits<double>::lowest());
        // // std::cout << "ba:\n " << ba << std::endl;
        // // std::cout << "lba:\n" << lba_ << std::endl;
        // // std::cout << "uba:\n" << uba_ << std::endl;

        // // // std::cout << "A: \n" << A_ << std::endl;
        // // // std::cout << "g: \n" << g_ << std::endl;
        // // // std::cout << "ba:\n" << ba_ << std::endl;
        // // std::cout << "lb:\n" << lb_ << std::endl;
        // // std::cout << "ub:\n" << ub_ << std::endl;
        // // std::cout << "nwsr: " << nwsr_ << std::endl;

        // // find solution given cpu constraints
        // int nwsr_tmp_ = nwsr_;
        // if (!qp_init_) {
        //     auto ret = qp_->init(
        //         nullptr,  // trivial qp
        //         g_.data(), 
        //         A_.data(), 
        //         lb_.data(), 
        //         ub_.data(), 
        //         lba_.data(), 
        //         uba_.data(), 
        //         nwsr_tmp_, 
        //         &cputime_
        //     );
        //     if (ret != qpOASES::SUCCESSFUL_RETURN) {
        //         RCLCPP_ERROR(node_->get_logger(), "Failed init solve SQP: qpOASES::returnValue::%d.", ret);
        //         // dq_.setZero();
        //         return controller_interface::return_type::ERROR; 
        //     }

        //     // qp_osqp_->data()


        //     // // OSQP
        //     // auto ret = qp_osqp_->solveProblem();
        //     // if (ret != OsqpEigen::ErrorExitFlag::NoError) {
        //     //     RCLCPP_ERROR(node_->get_logger(), "Failed init solve QP.");
        //     //     return controller_interface::return_type::ERROR;
        //     // };

        //     qp_init_ = true;
        // } else {
        //     auto ret = qp_->hotstart(
        //         nullptr,  // trivial qp
        //         g_.data(), 
        //         A_.data(), 
        //         lb_.data(), 
        //         ub_.data(), 
        //         lba_.data(), 
        //         uba_.data(), 
        //         nwsr_tmp_, 
        //         &cputime_
        //     );
        //     // if (ret != qpOASES::SUCCESSFUL_RETURN) {
        //     //     RCLCPP_ERROR(node_->get_logger(), "Failed hotstart solve SQP: qpOASES::returnValue::%d.", ret);
        //     //     // dq_.setZero();
        //     //     return controller_interface::return_type::ERROR; 
        //     // }
        // }

        // // get solution
        // Eigen::RowVectorXd dq = Eigen::RowVectorXd::Zero(dq_.size());
        // qp_->getPrimalSolution(dq.data());
        // std::cout << "dq: " << dq << std::endl;

        // // qpOASES





        // OSQP
        // compute jacobian
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            q_.data[i] = position_interfaces_[i].get().get_value();
        }
        jac_solver_->JntToJac(q_, J_);

        auto J = J_.data;
        auto J_pseudo_inv = dampedLeastSquares(J);
        Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(J.cols());

        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            tau_ext[i] = external_torque_interfaces_[i].get().get_value();
        }

        // threshold tau_ext?
        for (int i = 0; i < J.rows(); i++) {
            for (int j = 0; j < J.rows(); j++) {
                if (i < 3) {
                    A_osqp_.coeffRef(i, j) = 2500.*J(i, j);
                } else {
                    A_osqp_.coeffRef(i, j) = 500.*J(i, j);
                }
            }
        }

        Eigen::VectorXd ba = J_pseudo_inv.transpose()*tau_ext;

        // threshold noise
        ba.head(3) = ba.head(3).unaryExpr([](double d){
            if (std::abs(d) > 2.) {
                // double sign = std::signbit(d) ? -1.:1.;
                return d;
            } else {
                return 0.;
            }
        });
        ba.tail(3) = ba.tail(3).unaryExpr([](double d){
            if (std::abs(d) > 0.5) {
                // double sign = std::signbit(d) ? -0.5:0.5;
                return d;
            } else {
                return 0.;
            }
        });

        lb_osqp_.segment(lb_osqp_.size() - 7, lb_osqp_.size() - 4) = ba.head(3).array() - 1.;  // 1N, 1Nm
        lb_osqp_.tail(3) = ba.tail(3).array() - 0.25;  // 1N, 1Nm
        ub_osqp_.segment(ub_osqp_.size() - 7, ub_osqp_.size() - 4) = ba.head(3).array() + 1.;
        ub_osqp_.tail(3) = ba.tail(3).array() + 0.25;

        // update QP
        qp_osqp_->updateHessianMatrix(A_osqp_);
        qp_osqp_->updateLowerBound(lb_osqp_);
        qp_osqp_->updateUpperBound(ub_osqp_);

        // compute solution
        auto ret = qp_osqp_->solveProblem();
        if (ret != OsqpEigen::ErrorExitFlag::NoError) {
            RCLCPP_ERROR(node_->get_logger(), "Failed init solve QP.");
            return controller_interface::return_type::ERROR;
        };

        // get solution
        Eigen::VectorXd dq = qp_osqp_->getSolution(); // very simple!

        // OSQP

        // execute solution
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            dq_[i] = alpha_*dq_[i] + (1-alpha_)*dq[i];
            command_interfaces_[i].set_value(position_interfaces_[i].get().get_value() + dq_[i]);
        }


        

        // ********************************************************
        // ******************************************qp formulation
        // ********************************************************






        // ba_ = Eigen::VectorXd::Zero(kdl_chain_.getNrOfJoints());
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
