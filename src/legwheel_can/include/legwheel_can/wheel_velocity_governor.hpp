#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace legwheel_can
{
// This controller only removes driving torque. In particular, zero output means
// coasting, not regenerative braking: the rig's bench supply cannot absorb it.
class WheelVelocityGovernor
{
public:
  void configure(double speed_cap_rad_s, double proportional_gain, double integral_gain)
  {
    if (!std::isfinite(speed_cap_rad_s) || speed_cap_rad_s <= 0.0 ||
      !std::isfinite(proportional_gain) || proportional_gain <= 0.0 ||
      !std::isfinite(integral_gain) || integral_gain < 0.0)
    {
      throw std::invalid_argument("Invalid wheel velocity governor settings");
    }
    speed_cap_rad_s_ = speed_cap_rad_s;
    proportional_gain_ = proportional_gain;
    integral_gain_ = integral_gain;
    reset();
  }

  void reset()
  {
    integral_torque_nm_ = 0.0;
    previous_output_nm_ = 0.0;
    direction_ = 0;
  }

  double integral_torque_nm() const {return integral_torque_nm_;}

  double update(
    double requested_torque_nm, double velocity_rad_s, double dt_s,
    double torque_rise_rate_nm_s)
  {
    if (!std::isfinite(requested_torque_nm) || !std::isfinite(velocity_rad_s) ||
      !std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > 0.1 ||
      !std::isfinite(torque_rise_rate_nm_s) || torque_rise_rate_nm_s <= 0.0)
    {
      reset();
      return 0.0;
    }
    if (requested_torque_nm == 0.0) {
      reset();
      return 0.0;
    }

    const int requested_direction = requested_torque_nm > 0.0 ? 1 : -1;
    if (direction_ != 0 && direction_ != requested_direction) {
      // Give the caller one zero-torque cycle before changing drive direction.
      reset();
      direction_ = requested_direction;
      return 0.0;
    }
    direction_ = requested_direction;
    if (direction_ * velocity_rad_s < 0.0) {
      // Driving against an already reversing shaft is braking too. Wait for it
      // to stop rather than returning energy to the non-regenerative supply.
      integral_torque_nm_ = 0.0;
      previous_output_nm_ = 0.0;
      return 0.0;
    }
    const double available_torque_nm = std::abs(requested_torque_nm);
    const double speed_error_rad_s = speed_cap_rad_s_ - direction_ * velocity_rad_s;
    const double proportional_torque_nm = proportional_gain_ * speed_error_rad_s;

    // Include the output rise limit in anti-windup. Torque reductions are immediate;
    // a later slew limiter must never keep an excessive drive command alive.
    const double output_ceiling_nm = std::min(
      available_torque_nm, previous_output_nm_ + torque_rise_rate_nm_s * dt_s);
    const double unconstrained_torque_nm = proportional_torque_nm + integral_torque_nm_;
    const double integral_change_nm = integral_gain_ * speed_error_rad_s * dt_s;
    if (!std::isfinite(unconstrained_torque_nm) || !std::isfinite(integral_change_nm)) {
      reset();
      return 0.0;
    }
    const bool pushing_above_limit =
      unconstrained_torque_nm >= output_ceiling_nm && integral_change_nm > 0.0;
    const bool pushing_below_zero =
      unconstrained_torque_nm <= 0.0 && integral_change_nm < 0.0;
    if (!pushing_above_limit && !pushing_below_zero) {
      integral_torque_nm_ = std::clamp(
        integral_torque_nm_ + integral_change_nm, 0.0, available_torque_nm);
    }
    // The envelope can shrink during ramp-down even when integration is frozen.
    integral_torque_nm_ = std::min(integral_torque_nm_, available_torque_nm);
    previous_output_nm_ = std::clamp(
      proportional_torque_nm + integral_torque_nm_, 0.0, output_ceiling_nm);
    return direction_ * previous_output_nm_;
  }

private:
  double speed_cap_rad_s_{1.0};
  double proportional_gain_{0.5};
  double integral_gain_{0.1};
  double integral_torque_nm_{0.0};
  double previous_output_nm_{0.0};
  int direction_{0};
};
}  // namespace legwheel_can
