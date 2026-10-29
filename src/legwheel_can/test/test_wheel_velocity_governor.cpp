// A standalone test executable: the controller can be checked without ROS or CAN.
#include "legwheel_can/wheel_velocity_governor.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void test_torque_bounds_and_direction()
{
  for (double direction : {-1.0, 1.0}) {
    legwheel_can::WheelVelocityGovernor governor;
    governor.configure(3.0, 0.5, 0.1);
    for (int step = 0; step < 1000; ++step) {
      const double output = governor.update(direction * 2.0, direction, 0.01, 20.0);
      require(output * direction >= 0.0, "Governor must never request reverse torque");
      require(std::abs(output) <= 2.0, "Governor exceeded requested torque");
    }
    const double overspeed_output = governor.update(direction * 2.0, direction * 50.0, 0.01, 20.0);
    require(overspeed_output == 0.0, "Large overspeed must remove driving torque");
    require(governor.update(0.0, direction, 0.01, 20.0) == 0.0, "Stop must be immediate");
    require(governor.integral_torque_nm() == 0.0, "Stop must clear integral history");
    require(governor.update(-direction * 2.0, direction, 0.01, 20.0) == 0.0,
      "Opposing shaft motion would regenerate into the bench supply");
  }
}

void test_saturation_and_ramps()
{
  legwheel_can::WheelVelocityGovernor governor;
  governor.configure(4.5, 0.5, 0.1);
  for (int step = 0; step < 10000; ++step) {
    governor.update(0.2, 0.0, 0.01, 20.0);
  }
  require(governor.integral_torque_nm() == 0.0, "Blocked wheel wound up the integrator");
  double previous = 0.0;
  governor.reset();
  for (int step = 0; step < 100; ++step) {
    const double available = 2.0 * step / 100.0;
    const double output = governor.update(available, 0.0, 0.01, 1.0);
    require(output <= available + 1e-12, "Governor violated the torque envelope");
    require(output <= previous + 0.01 + 1e-12, "Torque restoration exceeded rise limit");
    previous = output;
  }
  require(governor.update(0.001, 0.0, 0.01, 1.0) <= 0.001,
    "Ramp-down reduction must not wait for a slow slew limiter");
  require(governor.update(2.0, 0.0, 0.2, 20.0) == 0.0, "Invalid timing must suppress drive");
  require(governor.update(2.0, std::numeric_limits<double>::quiet_NaN(), 0.01, 20.0) == 0.0,
    "Invalid velocity must suppress drive");
}

void test_load_changes_and_integral_recovery()
{
  // A deliberately simple inertial plant with viscous drag. This checks the
  // feedback behavior, not suitability of the proposed gains for the real rig.
  legwheel_can::WheelVelocityGovernor governor;
  governor.configure(3.0, 0.5, 0.1);
  double speed = 0.0;
  double largest_speed = 0.0;
  for (int step = 0; step < 30000; ++step) {
    const double drag = step < 15000 ? 0.25 : 0.08;
    const double torque = governor.update(2.0, speed, 0.01, 20.0);
    speed += (torque - drag * speed) / 0.2 * 0.01;
    largest_speed = std::max(largest_speed, speed);
    if (step == 14999) {
      require(std::abs(speed - 3.0) < 0.02, "PI did not remove steady load error");
    }
  }
  require(std::abs(speed - 3.0) < 0.02, "PI did not recover after drag decreased");
  require(largest_speed < 4.5, "Simulated load release caused excessive windup");
}

int main()
{
  try {
    test_torque_bounds_and_direction();
    test_saturation_and_ramps();
    test_load_changes_and_integral_recovery();
    std::cout << "Wheel velocity governor tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
