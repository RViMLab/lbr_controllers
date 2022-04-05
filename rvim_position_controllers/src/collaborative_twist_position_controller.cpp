#include <rvim_position_controllers/collaborative_twist_position_controller.hpp>

namespace rvim_position_controllers {

    controller_interface::return_type CollaborativeTwistPositionController::init(const std::string& controller_name) {
        auto ret = ControllerInterface::init(controller_name);
        if (ret != controller_interface::return_type::OK) {
            return ret;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::InterfaceConfiguration CollaborativeTwistPositionController::command_interface_configuration() const {
        RCLCPP_INFO(node_->get_logger(), "command_interface_configuration");
        controller_interface::InterfaceConfiguration command_interface_configuration;
        command_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joint_name: joint_names_) {
            command_interface_configuration.names.push_back(joint_name + "/" + command_interface_name_);
        }
        return command_interface_configuration;
    }

    controller_interface::InterfaceConfiguration CollaborativeTwistPositionController::state_interface_configuration() const {
        RCLCPP_INFO(node_->get_logger(), "state_interface_configuration");
        controller_interface::InterfaceConfiguration state_interface_configuration;
        state_interface_configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for (const auto& joint_name: joint_names_) {
            for (const auto& state_interface: state_interface_names_) {
                state_interface_configuration.names.push_back(joint_name + "/" + state_interface);
            }
        }
        return state_interface_configuration;
    }

    CallbackReturn CollaborativeTwistPositionController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(node_->get_logger(), "on_configure");
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

        // rt middleware buffers
        twist_command_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "~/twist", rclcpp::SystemDefaultsQoS(), [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                this->rt_twist_command_ptr_.writeFromNonRT(msg);
            }
        );

        wrench_state_pub_ = node_->create_publisher<geometry_msgs::msg::Wrench>(
            "~/wrench", rclcpp::SystemDefaultsQoS()
        );

        rt_wrench_state_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Wrench>>(
            wrench_state_pub_
        );

        // load robot description, see http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
        std::string robot_description_string = node_->get_parameter("robot_description").as_string();

        // read urdf
        if (!urdf_.initString(robot_description_string)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialize urdf from robot description.");
            return CallbackReturn::ERROR;
        };

        if (!kdl_parser::treeFromUrdfModel(urdf_, tree_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to construct kdl tree from urdf.");
            return CallbackReturn::ERROR;
        }

        // configure kdl chain by defining root and tip from tree
        if (!node_->get_parameter("base_link", base_link_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load base_link parameter, got '%s'.", base_link_.c_str());
            return CallbackReturn::ERROR;
        };
        if (!node_->get_parameter("hand_guide_link", hand_guide_link_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load hand_guide_link parameter, got '%s'.", hand_guide_link_.c_str());
            return CallbackReturn::ERROR;
        };

        if (!tree_.getChain(base_link_, hand_guide_link_, hand_guide_chain_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to extract kdl chain with root '%s' and tip '%s'.", base_link_.c_str(), hand_guide_link_.c_str());
            return CallbackReturn::ERROR;
        };

        if (!tree_.getChain(base_link_, camera_link_, camera_chain_)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to extract kdl chain with root '%s' and tip '%s'.", base_link_.c_str(), camera_link_.c_str());
            return CallbackReturn::ERROR;
        };


        // lb_q_ = Eigen::VectorXd::Zero(urdf_.joints_.size());
        // ub_q_ = Eigen::VectorXd::Zero(urdf_.joints_.size());
        // for (auto& joint: urdf_.joints_) {
        //     // lb_q_[i] = joint
        //     // joint.second->limits->lower
        //     // joint.second->limits->upper
        // }

        // other objective:
        // dx (body velocity) = Jdq, J^# dx = dq, as constraint
        // 
        // Joint limit conversion: max(dq, mu(q_max-q_i)), see https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.715.4291&rep=rep1&type=pdf section III.B
        // hierarchy of equality constraints: https://journals.sagepub.com/doi/pdf/10.1177/0278364914521306 section 1.3


        // allocate joint angles and Jacobian
        q_.resize(hand_guide_chain_.getNrOfJoints());
        J_.resize(hand_guide_chain_.getNrOfJoints());

        // create Jacobian solver from kdl chain
        hand_guide_jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(hand_guide_chain_);
        camera_jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(camera_chain_);

        nwsr_ = std::numeric_limits<int>::max();
        cputime_ = 0.005;  // 100 hz

        // build QP, update sizes
        H_osqp_.resize(hand_guide_chain_.getNrOfJoints(), hand_guide_chain_.getNrOfJoints()); H_osqp_.setIdentity();  // q^T H q
        A_osqp_.resize(J_.data.rows() + hand_guide_chain_.getNrOfJoints(), hand_guide_chain_.getNrOfJoints()); A_osqp_.setZero();  // [J, I]^T
        g_osqp_ = Eigen::VectorXd::Zero(hand_guide_chain_.getNrOfJoints());  // trivial -> zeros
        lb_osqp_ = Eigen::VectorXd::Constant(hand_guide_chain_.getNrOfJoints() + J_.data.rows(), -0.01);//std::numeric_limits<int>::lowest());
        ub_osqp_ = Eigen::VectorXd::Constant(hand_guide_chain_.getNrOfJoints() + J_.data.rows(),  0.01);//std::numeric_limits<int>::max());

        // identity for bounds on dq_osqp_
        // sparse matrices: https://eigen.tuxfamily.org/dox/group__SparseQuickRefPage.html
        // manipulation: https://eigen.tuxfamily.org/dox/group__TutorialSparse.html
        for (int i = 0; i < J_.data.cols(); i ++) {
            A_osqp_.insert(J_.data.rows()+i, i) = 1.;  // sparse matrix mostly empty, hence inserion required
        }

        dq_osqp_ = Eigen::VectorXd::Zero(hand_guide_chain_.getNrOfJoints());

        qp_osqp_ = std::make_unique<OsqpEigen::Solver>();
        qp_osqp_->settings()->setVerbosity(false);
        qp_osqp_->settings()->setWarmStart(true);
        qp_osqp_->data()->setNumberOfVariables(hand_guide_chain_.getNrOfJoints());
        qp_osqp_->data()->setNumberOfConstraints(hand_guide_chain_.getNrOfJoints() + J_.data.rows());

        if (!qp_osqp_->data()->setHessianMatrix(H_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to set Hessian matrix."); return CallbackReturn::ERROR; };
        if (!qp_osqp_->data()->setLinearConstraintsMatrix(A_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to set linear constraints matrix."); return CallbackReturn::ERROR; };
        if (!qp_osqp_->data()->setGradient(g_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to set gradient vector."); return CallbackReturn::ERROR; };
        if (!qp_osqp_->data()->setLowerBound(lb_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to set lower bounds."); return CallbackReturn::ERROR; };
        if (!qp_osqp_->data()->setUpperBound(ub_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to set upper bounds."); return CallbackReturn::ERROR; };

        qp_osqp_->settings()->setMaxIteration(nwsr_);
        qp_osqp_->settings()->setTimeLimit(cputime_);

        dq_ = Eigen::VectorXd::Zero(hand_guide_chain_.getNrOfJoints());

        if (!qp_osqp_->initSolver()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to initialized solver.");
            return CallbackReturn::ERROR;
        };

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CollaborativeTwistPositionController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // get a sorted reference
        position_interfaces_.clear(); external_torque_interfaces_.clear();
        RCLCPP_INFO(node_->get_logger(), "on_activate");
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

        // reset rt middleware buffers
        rt_twist_command_ptr_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn CollaborativeTwistPositionController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        RCLCPP_INFO(node_->get_logger(), "on_deactivate");

        // reset rt middleware buffers
        rt_twist_command_ptr_.reset();

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type CollaborativeTwistPositionController::update() {

        auto twist = rt_twist_command_ptr_.readFromRT();

        // add joint limits (read joint limits from robot description, limits via dt*dq) as inequality constraints
        // read camera frame rotation wrt base
        // add desired twist to QP via secondary task

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

        // compute jacobian
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            q_.data[i] = position_interfaces_[i].get().get_value();
        }
        hand_guide_jac_solver_->JntToJac(q_, J_);

        auto J = J_.data;
        auto J_pseudo_inv = dampedLeastSquares(J, 2.e-1);
        Eigen::VectorXd tau_ext = Eigen::VectorXd::Zero(J.cols());

        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            tau_ext[i] = external_torque_interfaces_[i].get().get_value();
        }

        // threshold tau_ext?
        for (int i = 0; i < J.rows(); i++) {
            for (int j = 0; j < J.cols(); j++) {
                if (i < 3) {
                    A_osqp_.coeffRef(i, j) = 3000.*J(i, j);
                } else {
                    A_osqp_.coeffRef(i, j) = 300.*J(i, j);
                }
            }
        }

        Eigen::VectorXd ba = J_pseudo_inv.transpose()*tau_ext;  // ba ~ f_ext, 6 dim

        // publish externally applied force
        if (rt_wrench_state_pub_->trylock()) {
            rt_wrench_state_pub_->msg_.force.x = ba[0];
            rt_wrench_state_pub_->msg_.force.y = ba[1];
            rt_wrench_state_pub_->msg_.force.z = ba[2];
            rt_wrench_state_pub_->msg_.torque.x = ba[0];
            rt_wrench_state_pub_->msg_.torque.y = ba[1];
            rt_wrench_state_pub_->msg_.torque.z = ba[2];
            rt_wrench_state_pub_->unlockAndPublish();
        }

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

        lb_osqp_.head(3) = ba.head(3).array() - 1.;  // 1N, 1Nm
        lb_osqp_.segment(3, 3) = ba.tail(3).array() - 0.25;  // 1N, 1Nm
        ub_osqp_.head(3) = ba.head(3).array() + 1.;
        ub_osqp_.segment(3, 3) = ba.tail(3).array() + 0.25;

        // update QP
        if (!qp_osqp_->updateLinearConstraintsMatrix(A_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to update linear constraints matrix."); return controller_interface::return_type::ERROR; };
        if (!qp_osqp_->updateBounds(lb_osqp_, ub_osqp_)) { RCLCPP_ERROR(node_->get_logger(), "Failed to update bounds."); return controller_interface::return_type::ERROR; };

        // compute solution
        auto ret = qp_osqp_->solveProblem();
        if (ret != OsqpEigen::ErrorExitFlag::NoError) {
            RCLCPP_ERROR(node_->get_logger(), "Failed init solve QP.");
            return controller_interface::return_type::ERROR;
        };

        Eigen::VectorXd dq = Eigen::VectorXd::Zero(hand_guide_chain_.getNrOfJoints());

        if (qp_osqp_->getStatus() == OsqpEigen::Status::Solved) {
            // get solution
            dq = qp_osqp_->getSolution(); // very simple!
        } else {
            RCLCPP_WARN(node_->get_logger(), "No solution found.");  // keep dq 0 at singularities
        }

        // execute solution
        for (std::size_t i = 0; i < joint_names_.size(); i++) {
            dq_[i] = alpha_*dq_[i] + (1-alpha_)*dq[i];
            // RCLCPP_INFO(node_->get_logger(), "%f", dq[i]);
            command_interfaces_[i].set_value(position_interfaces_[i].get().get_value() + dq_[i]);
        }

        return controller_interface::return_type::OK;
    }

}  // end of namespace rvim_position_controllers


#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rvim_position_controllers::CollaborativeTwistPositionController,
    controller_interface::ControllerInterface
)
