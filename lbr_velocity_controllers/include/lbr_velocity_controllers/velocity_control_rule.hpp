#ifndef LBR_VELOCITY_CONTROLLERS__VELOCITY_CONTROL_RULE_HPP_
#define LBR_VELOCITY_CONTROLLERS__VELOCITY_CONTROL_RULE_HPP_

#include <cmath>
#include <limits>
#include <vector>

#include "control_toolbox/filters.hpp"

namespace lbr_velocity_controllers {
class VelocityControlRule {
public:
  struct VelocityControlRuleParam {
    VelocityControlRuleParam(const double &position_exp_smooth, const double &velocity_exp_smooth)
        : position_exp_smooth(position_exp_smooth), velocity_exp_smooth(velocity_exp_smooth){};
    VelocityControlRuleParam() : position_exp_smooth(0.01), velocity_exp_smooth(0.01){};

    double position_exp_smooth;
    double velocity_exp_smooth;
  };

  VelocityControlRule(const std::size_t &dof)
      : smooth_desired_velocity_(dof, std::numeric_limits<double>::quiet_NaN()),
        smooth_position_(dof, std::numeric_limits<double>::quiet_NaN()),
        smooth_velocity_(dof, std::numeric_limits<double>::quiet_NaN()){};

  // position, velocity
  bool compute(const std::vector<double> &current_position,
               const std::vector<double> &current_velocity,
               const std::vector<double> &desired_velocity, const double &period,
               std::vector<double> &position_command) {
    if (desired_velocity.size() != smooth_desired_velocity_.size()) {
      return false;
    }
    if (current_position.size() != smooth_position_.size()) {
      return false;
    }
    if (current_velocity.size() != smooth_velocity_.size()) {
      return false;
    }
    for (std::size_t i = 0; i < current_position.size(); ++i) {
      if (std::isnan(smooth_desired_velocity_[i])) {
        smooth_desired_velocity_[i] = desired_velocity[i];
      } else {
        smooth_desired_velocity_[i] =
            filters::exponentialSmoothing(desired_velocity[i], smooth_desired_velocity_[i],
                                          param_.velocity_exp_smooth // TODO: replace!
            );
      }
      if (std::isnan(smooth_position_[i])) {
        smooth_position_[i] = current_position[i];
      } else {
        smooth_position_[i] = filters::exponentialSmoothing(
            current_position[i], smooth_position_[i], param_.position_exp_smooth);
      }
      if (std::isnan(smooth_velocity_[i])) {
        smooth_velocity_[i] = current_velocity[i];
      } else {
        smooth_velocity_[i] = filters::exponentialSmoothing(
            current_velocity[i], smooth_velocity_[i], param_.velocity_exp_smooth);
      }
      position_command[i] = smooth_position_[i] + smooth_desired_velocity_[i] * period;
    }
    return true;
  };

  inline const VelocityControlRuleParam &param() const { return param_; };
  inline VelocityControlRuleParam &param() { return param_; };

protected:
  VelocityControlRuleParam param_;
  std::vector<double> smooth_desired_velocity_;
  std::vector<double> smooth_position_;
  std::vector<double> smooth_velocity_;
};
} // namespace lbr_velocity_controllers
#endif // LBR_VELOCITY_CONTROLLERS__VELOCITY_CONTROL_RULE_HPP_
