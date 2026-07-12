#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "MD.hpp"
#include "candle.hpp"

namespace
{
constexpr std::size_t kHipIndex = 0;
constexpr std::size_t kKneeIndex = 1;

std::string md_error_to_string(const mab::MD::Error_t error)
{
  switch (error) {
    case mab::MD::Error_t::OK:
      return "OK";
    case mab::MD::Error_t::REQUEST_INVALID:
      return "REQUEST_INVALID";
    case mab::MD::Error_t::TRANSFER_FAILED:
      return "TRANSFER_FAILED";
    case mab::MD::Error_t::NOT_CONNECTED:
      return "NOT_CONNECTED";
    case mab::MD::Error_t::LEGACY_FW:
      return "LEGACY_FW";
    case mab::MD::Error_t::UNKNOWN_ERROR:
    default:
      return "UNKNOWN_ERROR";
  }
}
}  // namespace

class Md80ZeroNode : public rclcpp::Node
{
public:
  Md80ZeroNode()
  : Node("md80_zero_node")
  {
    declare_parameter<int>("hip_motor_id", 461);
    declare_parameter<int>("knee_motor_id", 923);
    declare_parameter<double>("publish_rate_hz", 50.0);

    joints_[kHipIndex].label = "hip";
    joints_[kHipIndex].joint_name = "hip_joint";
    joints_[kHipIndex].id = get_parameter("hip_motor_id").as_int();

    joints_[kKneeIndex].label = "knee";
    joints_[kKneeIndex].joint_name = "knee_joint";
    joints_[kKneeIndex].id = get_parameter("knee_motor_id").as_int();

    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz_ <= 0.0 || !std::isfinite(publish_rate_hz_)) {
      throw std::runtime_error("publish_rate_hz must be finite and positive.");
    }

    RCLCPP_WARN(
      get_logger(),
      "Starting MD80 zeroing node. This node never calls md.enable() and never commands motion.");
    RCLCPP_INFO(
      get_logger(),
      "Expected MD80 IDs: hip=%d, knee=%d",
      joints_[kHipIndex].id,
      joints_[kKneeIndex].id);

    connect_to_candle();
    discover_motors();
    connect_expected_motors();
    log_joint_availability();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/legwheel/joint_states", rclcpp::SensorDataQoS());

    hip_zero_srv_ = create_service<std_srvs::srv::Trigger>(
      "/legwheel/zero_hip_motor",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        zero_joint(kHipIndex, *response);
      });

    knee_zero_srv_ = create_service<std_srvs::srv::Trigger>(
      "/legwheel/zero_knee_motor",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        zero_joint(kKneeIndex, *response);
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Md80ZeroNode::update, this));
  }

  ~Md80ZeroNode() override
  {
    RCLCPP_WARN(get_logger(), "Shutting down zeroing node: disabling connected motors.");
    disable_all_motors();
    detach_candle();
  }

private:
  struct Joint
  {
    std::string label;
    std::string joint_name;
    int id{0};
    std::unique_ptr<mab::MD> md;
    bool connected{false};
    double last_position_rad{0.0};
    double last_velocity_rad_s{0.0};
    double last_effort_nm{0.0};
  };

  void connect_to_candle()
  {
    RCLCPP_INFO(get_logger(), "Connecting to CANdle USB adapter...");
    candle_ = mab::attachCandle(
      mab::CANdleDatarate_E::CAN_DATARATE_1M,
      mab::candleTypes::busTypes_t::USB);

    if (candle_ == nullptr) {
      throw std::runtime_error("Failed to attach CANdle.");
    }
  }

  void discover_motors()
  {
    RCLCPP_INFO(get_logger(), "Discovering MD80 controllers...");
    for (const auto id : mab::MD::discoverMDs(candle_)) {
      discovered_ids_.insert(static_cast<int>(id));
    }

    if (discovered_ids_.empty()) {
      RCLCPP_WARN(get_logger(), "No MD80 controllers were discovered.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Discovered MD80 IDs: %s", discovered_ids_string().c_str());
  }

  void connect_expected_motors()
  {
    for (auto & joint : joints_) {
      connect_motor_if_available(joint);
    }
  }

  void connect_motor_if_available(Joint & joint)
  {
    if (discovered_ids_.find(joint.id) == discovered_ids_.end()) {
      RCLCPP_WARN(
        get_logger(),
        "%s motor ID %d was expected but not discovered.",
        joint.label.c_str(),
        joint.id);
      return;
    }

    auto md = std::make_unique<mab::MD>(joint.id, candle_);
    const auto init_result = md->init();
    if (init_result != mab::MD::Error_t::OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize %s motor ID %d: %s",
        joint.label.c_str(),
        joint.id,
        md_error_to_string(init_result).c_str());
      return;
    }

    // Calibration must be passive. Disabling is safe and ensures the drive is unpowered.
    const auto disable_result = md->disable();
    if (disable_result != mab::MD::Error_t::OK) {
      RCLCPP_WARN(
        get_logger(),
        "Initial disable for %s motor ID %d returned %s.",
        joint.label.c_str(),
        joint.id,
        md_error_to_string(disable_result).c_str());
    }

    joint.md = std::move(md);
    joint.connected = true;

    RCLCPP_INFO(
      get_logger(),
      "Connected %s_joint to MD80 ID %d for passive zeroing.",
      joint.label.c_str(),
      joint.id);
  }

  void log_joint_availability()
  {
    for (const auto & joint : joints_) {
      RCLCPP_INFO(
        get_logger(),
        "%s motor availability: %s",
        joint.label.c_str(),
        joint.connected ? "connected" : "not connected");
    }
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(md_mutex_);
    for (auto & joint : joints_) {
      if (!joint.connected) {
        continue;
      }

      const auto position = joint.md->getPosition();
      if (position.second == mab::MD::Error_t::OK) {
        joint.last_position_rad = position.first;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Failed to read %s position: %s",
          joint.label.c_str(),
          md_error_to_string(position.second).c_str());
      }

      const auto velocity = joint.md->getVelocity();
      if (velocity.second == mab::MD::Error_t::OK) {
        joint.last_velocity_rad_s = velocity.first;
      }

      const auto torque = joint.md->getTorque();
      if (torque.second == mab::MD::Error_t::OK) {
        joint.last_effort_nm = torque.first;
      }
    }

    publish_joint_states();
  }

  void publish_joint_states()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    for (const auto & joint : joints_) {
      if (!joint.connected) {
        continue;
      }
      msg.name.push_back(joint.joint_name);
      msg.position.push_back(joint.last_position_rad);
      msg.velocity.push_back(joint.last_velocity_rad_s);
      msg.effort.push_back(joint.last_effort_nm);
    }
    joint_state_pub_->publish(msg);
  }

  void zero_joint(
    const std::size_t joint_index,
    std_srvs::srv::Trigger::Response & response)
  {
    std::lock_guard<std::mutex> lock(md_mutex_);
    auto & joint = joints_[joint_index];

    RCLCPP_WARN(
      get_logger(),
      "Zeroing requested for %s motor ID %d.",
      joint.label.c_str(),
      joint.id);

    if (!joint.connected) {
      response.success = false;
      response.message = joint.label + " motor is not connected; zeroing was not performed.";
      RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
      return;
    }

    // Never enable or command motion around zeroing. Disable first as a conservative guard.
    const auto disable_result = joint.md->disable();
    if (disable_result != mab::MD::Error_t::OK) {
      RCLCPP_WARN(
        get_logger(),
        "Disable before zeroing %s motor returned %s.",
        joint.label.c_str(),
        md_error_to_string(disable_result).c_str());
    }

    const auto zero_result = joint.md->zero();
    if (zero_result != mab::MD::Error_t::OK) {
      response.success = false;
      response.message =
        "Zeroing " + joint.label + " motor failed: " + md_error_to_string(zero_result);
      RCLCPP_ERROR(get_logger(), "%s", response.message.c_str());
      return;
    }

    response.success = true;
    response.message = "Zeroed " + joint.label + " motor ID " + std::to_string(joint.id) + ".";
    RCLCPP_INFO(get_logger(), "%s", response.message.c_str());
  }

  void disable_all_motors()
  {
    std::lock_guard<std::mutex> lock(md_mutex_);
    for (auto & joint : joints_) {
      if (!joint.connected) {
        continue;
      }
      const auto result = joint.md->disable();
      if (result != mab::MD::Error_t::OK) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to disable %s motor ID %d: %s",
          joint.label.c_str(),
          joint.id,
          md_error_to_string(result).c_str());
      }
    }
  }

  void detach_candle()
  {
    if (candle_ == nullptr) {
      return;
    }
    try {
      mab::detachCandle(candle_);
    } catch (...) {
      // Destructors must not throw during shutdown.
    }
    candle_ = nullptr;
  }

  std::string discovered_ids_string() const
  {
    std::ostringstream stream;
    bool first = true;
    for (const auto id : discovered_ids_) {
      if (!first) {
        stream << ", ";
      }
      stream << id;
      first = false;
    }
    return stream.str();
  }

  mab::Candle * candle_{nullptr};
  std::array<Joint, 2> joints_;
  std::unordered_set<int> discovered_ids_;
  double publish_rate_hz_{50.0};
  std::mutex md_mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hip_zero_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr knee_zero_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Md80ZeroNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal error in md80_zero_node: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
