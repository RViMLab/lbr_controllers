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
    VelocityControlRuleParam(const double &period_exp_smooth, const double &position_exp_smooth,
                             const double &velocity_exp_smooth,
                             const double &desired_velocity_exp_smooth)
        : period_exp_smooth(period_exp_smooth), position_exp_smooth(position_exp_smooth),
          velocity_exp_smooth(velocity_exp_smooth),
          desired_velocity_exp_smooth(desired_velocity_exp_smooth){};
    VelocityControlRuleParam()
        : period_exp_smooth(0.03), position_exp_smooth(0.03), velocity_exp_smooth(0.03),
          desired_velocity_exp_smooth(0.03){};

    double period_exp_smooth;
    double position_exp_smooth;
    double velocity_exp_smooth;
    double desired_velocity_exp_smooth;
  };

  VelocityControlRule(const std::size_t &dof)
      : smooth_period_(std::numeric_limits<double>::quiet_NaN()),
        smooth_position_(dof, std::numeric_limits<double>::quiet_NaN()),
        smooth_velocity_(dof, std::numeric_limits<double>::quiet_NaN()),
        smooth_desired_velocity_(dof, std::numeric_limits<double>::quiet_NaN()){};

  // position, velocity
  bool compute(const std::vector<double> &current_position,
               const std::vector<double> &current_velocity,
               const std::vector<double> &desired_velocity, const double &period,
               std::vector<double> &position_command) {
    if (current_position.size() != smooth_position_.size()) {
      return false;
    }
    if (current_velocity.size() != smooth_velocity_.size()) {
      return false;
    }
    if (desired_velocity.size() != smooth_desired_velocity_.size()) {
      return false;
    }
    exponential_smoothing_(period, param_.period_exp_smooth, smooth_period_);
    for (std::size_t i = 0; i < current_position.size(); ++i) {
      exponential_smoothing_(current_position[i], param_.position_exp_smooth, smooth_position_[i]);
      exponential_smoothing_(current_velocity[i], param_.velocity_exp_smooth, smooth_velocity_[i]);
      exponential_smoothing_(desired_velocity[i], param_.desired_velocity_exp_smooth,
                             smooth_desired_velocity_[i]);
      position_command[i] = smooth_position_[i] + smooth_desired_velocity_[i] * smooth_period_;
    }
    return true;
  };

  inline const VelocityControlRuleParam &param() const { return param_; };
  inline VelocityControlRuleParam &param() { return param_; };

protected:
  void exponential_smoothing_(const double &raw, const double &alpha, double &smooth) {
    if (std::isnan(smooth)) {
      smooth = raw;
    } else {
      smooth = filters::exponentialSmoothing(raw, smooth, alpha);
    }
  }

  VelocityControlRuleParam param_;
  double smooth_period_;
  std::vector<double> smooth_position_;
  std::vector<double> smooth_velocity_;
  std::vector<double> smooth_desired_velocity_;
};
} // namespace lbr_velocity_controllers
#endif // LBR_VELOCITY_CONTROLLERS__VELOCITY_CONTROL_RULE_HPP_
