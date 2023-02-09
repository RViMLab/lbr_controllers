#ifndef LBR_POSITION_CONTROLLERS__RAMP_HPP_
#define LBR_POSITION_CONTROLLERS__RAMP_HPP_

#include <cmath>
#include <stdexcept>

namespace lbr_position_controllers {
class Ramp {
public:
  Ramp(double v_max, double a) {
    if (a <= 0.) {
      throw std::invalid_argument("Acceleration must be greater equal zero.");
    }
    if (v_max <= 0.) {
      throw std::invalid_argument("Maximum velocity must be greater equal zero.");
    }
    v_max_ = v_max;
    a_ = a;
  }

  double operator()(double t) {
    if (t < 0.) {
      throw std::invalid_argument("Time must be greater zero.");
    }
    if (t > tend_) {
      throw std::invalid_argument("Time must be smaller than end time.");
    }

    if (t < t0_) {
      return 0.5 * a_ * std::pow(t, 2);
    } else if (t < t1_) {
      return 0.5 * a_ * std::pow(t0_, 2) + v_max_ * (t - t0_);
    } else {
      return s_ - 0.5 * a_ * std::pow(tend_ - t, 2);
    }
  }

  void set_s(const double &s) {
    s_ = s;
    compute_times_();
  }
  void set_v_max(const double &v_max) {
    if (v_max <= 0.) {
      throw std::invalid_argument("Maximum velocity must be greater equal zero.");
    }
    v_max_ = v_max;
    compute_times_();
  }
  void set_a(const double &a) {
    if (a <= 0.) {
      throw std::invalid_argument("Acceleration must be greater equal zero.");
    }
    a_ = a;
    compute_times_();
  }

  inline const double &get_s() const { return s_; }
  inline const double &get_v_max() const { return v_max_; }
  inline const double &get_a() const { return a_; }

protected:
  bool maximum_velocity_reached_() { return std::pow(v_max_, 2) / a_ < std::abs(s_); }
  void compute_times_() {
    if (maximum_velocity_reached_()) {
      t0_ = std::sqrt(std::abs(s_) / a_);
      t1_ = t0_;
      tend_ = 2 * t1_;
    } else {
      t0_ = v_max_ / a_;
      tend_ = std::abs(s_) / v_max_ + v_max_ / a_;
      t1_ = tend_ - t0_;
    }
  }

  double s_, v_max_, a_;
  double t0_, t1_, tend_;
};
} // namespace lbr_position_controllers
#endif // LBR_POSITION_CONTROLLERS__RAMP_HPP_
