#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// CANdle-SDK headers
#include "candle.hpp"
#include "MD.hpp"

using namespace std::chrono_literals;

class Md80ImpedanceNode : public rclcpp::Node
{
public:
  Md80ImpedanceNode()
  : Node("md80_impedance_node")
  {
    declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>{});
    declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{"hip_joint", "knee_joint"});

    declare_parameter<std::vector<double>>("position_min_rad", std::vector<double>{-0.5, -0.5});
    declare_parameter<std::vector<double>>("position_max_rad", std::vector<double>{ 0.5,  0.5});
    declare_parameter<std::vector<double>>("velocity_max_rad_s", std::vector<double>{1.0, 1.0});

    declare_parameter<double>("kp", 1.0);
    declare_parameter<double>("kd", 0.05);
    declare_parameter<double>("torque_ff", 0.0);

    declare_parameter<bool>("zero_on_startup", false);
    declare_parameter<bool>("enable_control", false);
    declare_parameter<double>("publish_rate_hz", 100.0);

    motor_ids_ = get_parameter("motor_ids").as_integer_array();
    joint_names_ = get_parameter("joint_names").as_string_array();
    position_min_ = get_parameter("position_min_rad").as_double_array();
    position_max_ = get_parameter("position_max_rad").as_double_array();
    velocity_max_ = get_parameter("velocity_max_rad_s").as_double_array();

    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    torque_ff_ = get_parameter("torque_ff").as_double();
    zero_on_startup_ = get_parameter("zero_on_startup").as_bool();
    enable_control_ = get_parameter("enable_control").as_bool();

    validate_parameters();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/legwheel/joint_states", 10);

    command_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/legwheel/target_positions",
      10,
      std::bind(&Md80ImpedanceNode::target_callback, this, std::placeholders::_1));

    connect_to_motors();

    const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Md80ImpedanceNode::update, this));
  }

  ~Md80ImpedanceNode() override
  {
    RCLCPP_WARN(get_logger(), "Shutting down MD80 node: disabling motors.");

    for (auto & md : mds_) {
      try {
        md.disable();
      } catch (...) {
        // Never throw from destructor.
      }
    }

    if (candle_ != nullptr) {
      try {
        mab::detachCandle(candle_);
      } catch (...) {
      }
    }
  }

private:
  void validate_parameters()
  {
    if (joint_names_.size() != 2) {
      throw std::runtime_error("joint_names must contain exactly 2 names.");
    }

    if (position_min_.size() != 2 || position_max_.size() != 2 || velocity_max_.size() != 2) {
      throw std::runtime_error("position_min_rad, position_max_rad, and velocity_max_rad_s must each contain 2 values.");
    }

    for (size_t i = 0; i < 2; ++i) {
      if (position_min_[i] >= position_max_[i]) {
        throw std::runtime_error("Each position_min_rad must be smaller than position_max_rad.");
      }
      if (velocity_max_[i] <= 0.0) {
        throw std::runtime_error("Each velocity_max_rad_s must be positive.");
      }
    }
  }

  void connect_to_motors()
  {
    RCLCPP_INFO(get_logger(), "Connecting to CANdle...");

    candle_ = mab::attachCandle(
      mab::CANdleDatarate_E::CAN_DATARATE_1M,
      mab::candleTypes::busTypes_t::USB);

    if (candle_ == nullptr) {
      throw std::runtime_error("Failed to attach CANdle.");
    }

    RCLCPP_INFO(get_logger(), "Discovering MD80 controllers...");

    auto discovered_ids = mab::MD::discoverMDs(candle_);

    if (discovered_ids.empty()) {
      throw std::runtime_error("No MD controllers discovered.");
    }

    RCLCPP_INFO(get_logger(), "Discovered %zu MD controller(s).", discovered_ids.size());

    std::vector<int> ids_to_use;

    if (motor_ids_.empty()) {
      if (discovered_ids.size() != 2) {
        throw std::runtime_error(
          "motor_ids parameter is empty, so expected exactly 2 discovered MDs. "
          "Set motor_ids explicitly if more/less are visible.");
      }

      for (auto id : discovered_ids) {
        ids_to_use.push_back(static_cast<int>(id));
      }
    } else {
      if (motor_ids_.size() != 2) {
        throw std::runtime_error("motor_ids must contain exactly 2 IDs.");
      }

      for (auto id : motor_ids_) {
        ids_to_use.push_back(static_cast<int>(id));
      }
    }

    for (const auto id : ids_to_use) {
      RCLCPP_INFO(get_logger(), "Initializing MD controller ID %d", id);

      mab::MD md(id, candle_);

      if (md.init() != mab::MD::Error_t::OK) {
        throw std::runtime_error("Failed to initialize one of the MD controllers.");
      }

      if (zero_on_startup_) {
        RCLCPP_WARN(
          get_logger(),
          "Zeroing MD ID %d. Make sure the joint is physically at its chosen zero pose.",
          id);
        md.zero();
      }

      md.setMotionMode(mab::MdMode_E::IMPEDANCE);

      if (enable_control_) {
        RCLCPP_WARN(get_logger(), "Enabling MD ID %d in impedance mode.", id);
        md.enable();
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Control disabled for MD ID %d. Publishing encoder data only.",
          id);
      }

      mds_.push_back(md);
    }

    target_positions_.resize(2, 0.0);
    last_positions_.resize(2, 0.0);
    velocities_.resize(2, 0.0);
    first_read_ = true;
  }

  void target_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(get_logger(), "Expected exactly 2 target positions.");
      return;
    }

    for (size_t i = 0; i < 2; ++i) {
      const double raw_target = msg->data[i];
      const double clamped_target = std::clamp(raw_target, position_min_[i], position_max_[i]);

      if (std::abs(raw_target - clamped_target) > 1e-9) {
        RCLCPP_WARN(
          get_logger(),
          "Target for %s was outside limits. Requested %.3f rad, clamped to %.3f rad.",
          joint_names_[i].c_str(),
          raw_target,
          clamped_target);
      }

      target_positions_[i] = clamped_target;
    }
  }

  void update()
  {
    const auto now = this->now();

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now;
    msg.name = joint_names_;
    msg.position.resize(2);
    msg.velocity.resize(2);
    msg.effort.resize(2);

    for (size_t i = 0; i < 2; ++i) {
      const double position = static_cast<double>(mds_[i].getPosition().first);

      double velocity = 0.0;
      if (!first_read_) {
        const double dt = (now - last_time_).seconds();
        if (dt > 1e-6) {
          velocity = (position - last_positions_[i]) / dt;
        }
      }

      velocity = std::clamp(velocity, -velocity_max_[i], velocity_max_[i]);

      msg.position[i] = position;
      msg.velocity[i] = velocity;
      msg.effort[i] = 0.0;  // Add measured/estimated torque later if SDK call is available.

      last_positions_[i] = position;
      velocities_[i] = velocity;

      if (enable_control_) {
        const double safe_target = std::clamp(
          target_positions_[i],
          position_min_[i],
          position_max_[i]);

        /*
         * Minimal impedance command:
         * - target position comes from /legwheel/target_positions
         * - velocity target is zero for now
         *
         * Depending on your CANdle-SDK version, you may have explicit methods like:
         *   setTargetPosition(...)
         *   setTargetVelocity(...)
         *   setTargetTorque(...)
         *   setImpedanceKp(...)
         *   setImpedanceKd(...)
         *
         * MAB's example shows setTargetPosition(...). Add the gain/torque calls
         * after confirming their exact names in your installed SDK headers/examples.
         */
        mds_[i].setTargetPosition(static_cast<float>(safe_target));
      }
    }

    first_read_ = false;
    last_time_ = now;

    joint_state_pub_->publish(msg);
  }

  mab::Candle * candle_{nullptr};
  std::vector<mab::MD> mds_;

  std::vector<int64_t> motor_ids_;
  std::vector<std::string> joint_names_;

  std::vector<double> position_min_;
  std::vector<double> position_max_;
  std::vector<double> velocity_max_;

  std::vector<double> target_positions_;
  std::vector<double> last_positions_;
  std::vector<double> velocities_;

  double kp_{1.0};
  double kd_{0.05};
  double torque_ff_{0.0};

  bool zero_on_startup_{false};
  bool enable_control_{false};
  bool first_read_{true};

  rclcpp::Time last_time_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Md80ImpedanceNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal error in md80_impedance_node: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}