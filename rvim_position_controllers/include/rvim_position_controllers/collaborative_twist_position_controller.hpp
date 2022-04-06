#pragma once

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <OsqpEigen/OsqpEigen.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include <damped_least_squares.hpp>

// follow http://control.ros.org/ros2_controllers/doc/writing_new_controller.html
// see forward command controller for an example: https://github.com/ros-controls/ros2_controllers/blob/foxy/forward_command_controller/include/forward_command_controller/forward_command_controller.hpp
// cartesian controller: http://library.isr.ist.utl.pt/docs/roswiki/pr2_mechanism(2f)Tutorials(2f)Coding(20)a(20)realtime(20)Cartesian(20)controller(20)with(20)KDL.html
// replicate previous hand guiding mode

namespace rvim_position_controllers {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using RowMajorMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    constexpr char HW_IF_EXTERNAL_TORQUE[] = "external_torque";

    class CollaborativeTwistPositionController : public controller_interface::ControllerInterface {
        public:
            CollaborativeTwistPositionController() = default;

            controller_interface::return_type init(const std::string& controller_name) override;
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            controller_interface::return_type update() override;

        protected:
            std::vector<std::string> joint_names_;
            std::string command_interface_name_;              // for parsing in yaml file (external_torque & position required)
            std::vector<std::string> state_interface_names_;  // for parsing in yaml file (external_torque & position required)

            // twist command subscription and wrench state publisher through realtime middleware
            realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist::SharedPtr> rt_twist_command_ptr_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_command_sub_;

            rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_state_pub_;
            std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Wrench>> rt_wrench_state_pub_;

            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> position_interfaces_;
            std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> external_torque_interfaces_;

            // KDL
            urdf::Model urdf_;

            KDL::Tree tree_;
            KDL::Chain hand_guide_chain_, camera_chain_;

            std::string base_link_, hand_guide_link_, camera_link_;

            KDL::JntArray q_;
            KDL::Jacobian J_hand_guide_, J_cam_;

            std::unique_ptr<KDL::ChainJntToJacSolver> hand_guide_jac_solver_, camera_jac_solver_;

            double th_f_=2.;
            double th_d_=1.;

            // frequency filtering, exponential smoothing
            std::vector<double> prev_update_ = std::vector<double>(7, 0.);
            double alpha_ = 0.98;

            int nwsr_;
            double cputime_;  // max cpu time in seconds

            // osqp-eigen
            std::unique_ptr<OsqpEigen::Solver> qp_osqp_;
            Eigen::SparseMatrix<double> H_osqp_, A_osqp_;
            Eigen::VectorXd g_osqp_, lb_osqp_, ub_osqp_;
            Eigen::VectorXd dq_osqp_;
            Eigen::VectorXd dq_;

            // upper and lower bounds on joint angles
            Eigen::VectorXd lb_q_, ub_q_;

            // additional parameters
            double force_constraint_relaxation_, torque_constraint_relaxation_;
            double force_threshold_, torque_threshold_;
            double dq_lim_;
            double mu_;
    };

}  // end of namespace rvim_position_controllers
