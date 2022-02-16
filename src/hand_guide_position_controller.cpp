#include <rvim_ros2_controllers_experimental/hand_guide_position_controller.hpp>

namespace rvim_ros2_controllers_experimental {

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
//

// how to access robot description?
// read from parameters, e.g. node->get_param https://github.com/ros-controls/ros2_controllers/blob/69958d7c5f998d95ef67f248cba3b8923b83b3b9/forward_command_controller/src/forward_command_controller.cpp#L66
// load string to kdl http://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
// how to load it into KDL?
// how to obtain Jacobian?
// how to link qpoases?

// input:
// tau_ext (state interface), dx (homography in vision node)

// control (joint impedance control mode with target position):
// qp with hot start
// q_i-1 + dt dq = q_i (target)

}
