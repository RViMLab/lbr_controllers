#ifndef LBR_POSITION_CONTROLLERS__RAMP_HPP_
#define LBR_POSITION_CONTROLLERS__RAMP_HPP_

#include <cmath>
#include <limits>
#include <stdexcept>

namespace lbr_position_controllers {
class Ramp {
public:
  Ramp()
      : s_(std::numeric_limits<double>::quiet_NaN()),
        v_max_(std::numeric_limits<double>::quiet_NaN()),
        a_(std::numeric_limits<double>::quiet_NaN()), t0_(std::numeric_limits<double>::quiet_NaN()),
        t1_(std::numeric_limits<double>::quiet_NaN()),
        tend_(std::numeric_limits<double>::quiet_NaN()) {}

  double operator()(double t) {
    if (std::isnan(t0_) || std::isnan(t1_) || std::isnan(tend_)) {
      return 0.;
    }

    if (t < 0.) {
      throw std::invalid_argument("Time must be greater zero.");
    }

    if (t < t0_) {
      return 0.5 * a_ * std::pow(t, 2);
    } else if (t < t1_) {
      return 0.5 * a_ * std::pow(t0_, 2) + v_max_ * (t - t0_);
    } else if (t < tend_) {
      return s_ - 0.5 * a_ * std::pow(tend_ - t, 2);
    } else {
      return 0.;
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
    if (std::isnan(s_) || std::isnan(v_max_) || std::isnan(a_)) {
      return;
    }

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
