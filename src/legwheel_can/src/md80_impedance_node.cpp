#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

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

bool is_finite(const double value)
{
  return std::isfinite(value);
}

enum class OperatingMode
{
  Impedance,
  OneDof,
};

std::string operating_mode_to_string(const OperatingMode mode)
{
  switch (mode) {
    case OperatingMode::Impedance:
      return "impedance";
    case OperatingMode::OneDof:
      return "1dof";
  }
  return "unknown";
}

std::optional<OperatingMode> parse_operating_mode(std::string mode)
{
  std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });

  if (mode == "impedance") {
    return OperatingMode::Impedance;
  }
  if (mode == "1dof" || mode == "one_dof" || mode == "onedof") {
    return OperatingMode::OneDof;
  }
  return std::nullopt;
}
}  // namespace

// Creating a ROS2 node class for MD80 impedance control

class Md80ImpedanceNode : public rclcpp::Node
{
public:
  Md80ImpedanceNode()
  : Node("md80_impedance_node") // Node name
  {
    declare_parameters();
    load_parameters();
    validate_limits();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/legwheel/joint_states", rclcpp::SensorDataQoS());

    status_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    command_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions status_options;
    status_options.callback_group = status_callback_group_;
    motor_status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/legwheel/motor_status",
      rclcpp::QoS(10).reliable(),
      std::bind(&Md80ImpedanceNode::handle_motor_status, this, std::placeholders::_1),
      status_options);

    rclcpp::SubscriptionOptions command_options;
    command_options.callback_group = command_callback_group_;
    spring_zero_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/legwheel/spring_zero_position",
      10,
      std::bind(&Md80ImpedanceNode::handle_spring_zero_position, this, std::placeholders::_1),
      command_options);
    spring_constant_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/legwheel/spring_constant",
      10,
      std::bind(&Md80ImpedanceNode::handle_spring_constant, this, std::placeholders::_1),
      command_options);
    damping_constant_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/legwheel/damping_constant",
      10,
      std::bind(&Md80ImpedanceNode::handle_damping_constant, this, std::placeholders::_1),
      command_options);
    operating_mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/legwheel/operating_mode",
      10,
      std::bind(&Md80ImpedanceNode::handle_operating_mode, this, std::placeholders::_1),
      command_options);

    connect_to_candle();
    discover_motors();
    connect_expected_motors();

    if (!any_motor_connected()) {
      RCLCPP_ERROR(
        get_logger(),
        "No expected MD80 controllers are connected. Node will remain alive but cannot publish data.");
    }

    if (enable_control_requested_ && !limits_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "enable_control was requested, but software limits are missing or invalid. "
        "Control will stay disabled.");
    } else if (!enable_control_requested_) {
      RCLCPP_WARN(get_logger(), "enable_control is false. Running in read-only encoder mode.");
    }

    if (enable_control_requested_ && limits_valid_) {
      RCLCPP_WARN(
        get_logger(),
        "Control is armed by parameter, but motors will remain disabled until "
        "/legwheel/motor_status publishes true.");
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Md80ImpedanceNode::update, this));
  }

  ~Md80ImpedanceNode() override
  {
    RCLCPP_WARN(get_logger(), "Shutting down impedance node: disabling all connected motors.");
    disable_all_motors();
    detach_candle();
  }

private:
  struct JointLimits
  {
    double position_min_rad{0.0};
    double position_max_rad{0.0};
    double velocity_max_rad_s{0.0};
    double torque_max_nm{0.0};
  };

  struct Joint
  {
    std::string label;
    std::string joint_name;
    int id{0};
    JointLimits limits;
    std::unique_ptr<mab::MD> md;
    bool connected{false};
    bool enabled{false};
    bool impedance_configured{false};
    double spring_zero_rad{0.0};
    double kp{1.0};
    double kd{0.05};
    double last_position_rad{0.0};
    double last_velocity_rad_s{0.0};
    double last_effort_nm{0.0};
  };

  void declare_parameters()
  {
    declare_parameter<int>("hip_motor_id", 461);
    declare_parameter<int>("knee_motor_id", 923);
    declare_parameter<bool>("limits_provided", false);

    declare_parameter<double>("hip_position_min_rad", -0.5);
    declare_parameter<double>("hip_position_max_rad", 0.5);
    declare_parameter<double>("hip_velocity_max_rad_s", 1.0);
    declare_parameter<double>("hip_torque_max_nm", 2.0);
    declare_parameter<double>("knee_position_min_rad", -0.5);
    declare_parameter<double>("knee_position_max_rad", 0.5);
    declare_parameter<double>("knee_velocity_max_rad_s", 1.0);
    declare_parameter<double>("knee_torque_max_nm", 2.0);

    declare_parameter<double>("initial_spring_constant", 4.0);
    declare_parameter<double>("initial_damping_constant", 0.05);
    declare_parameter<double>("max_spring_constant", 50.0);
    declare_parameter<double>("max_damping_constant", 5.0);
    declare_parameter<double>("hip_mirror_multiplier", -0.5);
    declare_parameter<std::string>("command_limit_policy", "reject");

    declare_parameter<bool>("enable_control", false);
    declare_parameter<double>("publish_rate_hz", 100.0);
  }

  void load_parameters()
  {
    joints_[kHipIndex].label = "hip";
    joints_[kHipIndex].joint_name = "hip_joint";
    joints_[kHipIndex].id = get_parameter("hip_motor_id").as_int();
    joints_[kHipIndex].limits.position_min_rad =
      get_parameter("hip_position_min_rad").as_double();
    joints_[kHipIndex].limits.position_max_rad =
      get_parameter("hip_position_max_rad").as_double();
    joints_[kHipIndex].limits.velocity_max_rad_s =
      get_parameter("hip_velocity_max_rad_s").as_double();
    joints_[kHipIndex].limits.torque_max_nm = get_parameter("hip_torque_max_nm").as_double();

    joints_[kKneeIndex].label = "knee";
    joints_[kKneeIndex].joint_name = "knee_joint";
    joints_[kKneeIndex].id = get_parameter("knee_motor_id").as_int();
    joints_[kKneeIndex].limits.position_min_rad =
      get_parameter("knee_position_min_rad").as_double();
    joints_[kKneeIndex].limits.position_max_rad =
      get_parameter("knee_position_max_rad").as_double();
    joints_[kKneeIndex].limits.velocity_max_rad_s =
      get_parameter("knee_velocity_max_rad_s").as_double();
    joints_[kKneeIndex].limits.torque_max_nm = get_parameter("knee_torque_max_nm").as_double();

    limits_provided_ = get_parameter("limits_provided").as_bool();
    enable_control_requested_ = get_parameter("enable_control").as_bool();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    max_spring_constant_ = get_parameter("max_spring_constant").as_double();
    max_damping_constant_ = get_parameter("max_damping_constant").as_double();
    hip_mirror_multiplier_ = get_parameter("hip_mirror_multiplier").as_double();
    command_limit_policy_ = get_parameter("command_limit_policy").as_string();

    const double initial_kp = get_parameter("initial_spring_constant").as_double();
    const double initial_kd = get_parameter("initial_damping_constant").as_double();
    for (auto & joint : joints_) { // This sets the initial spring and damping constants for each joint
      joint.kp = initial_kp;
      joint.kd = initial_kd;
    }

    if (publish_rate_hz_ <= 0.0 || !is_finite(publish_rate_hz_)) {
      throw std::runtime_error("publish_rate_hz must be finite and positive.");
    }
    if (command_limit_policy_ != "reject" && command_limit_policy_ != "clamp") { // This checks if the command limit policy is valid. Command limit policy is used to determine how to handle commands that exceed the software limits. If the policy is "reject", commands outside the limits will be rejected. If the policy is "clamp", commands outside the limits will be clamped to the nearest limit.
      throw std::runtime_error("command_limit_policy must be either 'reject' or 'clamp'.");
    }
    if (!is_finite(hip_mirror_multiplier_)) {
      throw std::runtime_error("hip_mirror_multiplier must be finite.");
    }

    RCLCPP_INFO( // This prints out data into the INFO-level logs
      get_logger(),
      "Expected MD80 IDs: hip=%d, knee=%d",
      joints_[kHipIndex].id,
      joints_[kKneeIndex].id);
  }

  void validate_limits()
  {
    limits_valid_ = limits_provided_;

    if (!limits_provided_) {
      RCLCPP_WARN(
        get_logger(),
        "limits_provided is false. Encoder publishing is allowed, but control cannot be enabled.");
      return;
    }

    if (max_spring_constant_ <= 0.0 || !is_finite(max_spring_constant_)) {
      RCLCPP_ERROR(get_logger(), "max_spring_constant must be finite and positive.");
      limits_valid_ = false;
    }
    if (max_damping_constant_ <= 0.0 || !is_finite(max_damping_constant_)) {
      RCLCPP_ERROR(get_logger(), "max_damping_constant must be finite and positive.");
      limits_valid_ = false;
    }

    for (const auto & joint : joints_) {
      const auto & limits = joint.limits;
      if (!is_finite(limits.position_min_rad) || !is_finite(limits.position_max_rad) ||
        limits.position_min_rad >= limits.position_max_rad)
      {
        RCLCPP_ERROR(
          get_logger(),
          "%s position limits are invalid: min=%.3f max=%.3f",
          joint.label.c_str(),
          limits.position_min_rad,
          limits.position_max_rad);
        limits_valid_ = false;
      }
      if (!is_finite(limits.velocity_max_rad_s) || limits.velocity_max_rad_s <= 0.0) {
        RCLCPP_ERROR(
          get_logger(),
          "%s velocity limit must be finite and positive.",
          joint.label.c_str());
        limits_valid_ = false;
      }
      if (!is_finite(limits.torque_max_nm) || limits.torque_max_nm <= 0.0) {
        RCLCPP_ERROR(
          get_logger(),
          "%s torque limit must be finite and positive.",
          joint.label.c_str());
        limits_valid_ = false;
      }
      if (joint.kp < 0.0 || joint.kp > max_spring_constant_ || !is_finite(joint.kp)) {
        RCLCPP_ERROR(get_logger(), "initial_spring_constant is invalid.");
        limits_valid_ = false;
      }
      if (joint.kd < 0.0 || joint.kd > max_damping_constant_ || !is_finite(joint.kd)) {
        RCLCPP_ERROR(get_logger(), "initial_damping_constant is invalid.");
        limits_valid_ = false;
      }
    }

    if (limits_valid_) {
      RCLCPP_INFO(get_logger(), "Software motor limits are present and valid.");
    }
  }

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
    RCLCPP_INFO(get_logger(), "Discovering MD80 controllers... This might take 10-30 seconds...");
    discovered_ids_.clear();
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
        "%s motor ID %d was expected but not discovered. Continuing without this joint.",
        joint.label.c_str(),
        joint.id);
      return;
    }

    auto md = std::make_unique<mab::MD>(joint.id, candle_); // This creates a new instance of the MD class for the joint with the specified ID and candle pointer.
    const auto init_result = md->init(); // This initializes the MD instance, which involves setting up communication with the motor controller and preparing it for operation.
    if (init_result != mab::MD::Error_t::OK) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to initialize %s motor ID %d: %s",
        joint.label.c_str(),
        joint.id,
        md_error_to_string(init_result).c_str());
      return;
    }

    // Keep the drive unpowered on startup. Enabling is gated by parameter, valid limits,
    // and an explicit true message on /legwheel/motor_status.
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
    joint.enabled = false;
    joint.impedance_configured = false;

    RCLCPP_INFO(
      get_logger(),
      "Connected %s_joint to MD80 ID %d.",
      joint.label.c_str(),
      joint.id);
  }

  bool any_motor_connected() const
  {
    return std::any_of(joints_.begin(), joints_.end(), [](const auto & joint) {
      return joint.connected;
    });
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(md_mutex_);
    read_connected_motors();
    publish_joint_states();

    if (!control_may_run()) {
      return;
    }

    for (auto & joint : joints_) {
      if (!joint.connected) {
        continue;
      }
      if (operating_mode_ == OperatingMode::Impedance) {
        command_impedance(joint, joint.spring_zero_rad);
      }
    }

    if (operating_mode_ == OperatingMode::OneDof) {
      command_one_dof();
    }
  }

  void read_connected_motors()
  {
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
        joint.last_velocity_rad_s = std::clamp(
          static_cast<double>(velocity.first),
          -joint.limits.velocity_max_rad_s,
          joint.limits.velocity_max_rad_s);
      }

      const auto torque = joint.md->getTorque();
      if (torque.second == mab::MD::Error_t::OK) {
        joint.last_effort_nm = torque.first;
      }
    }
  }

  void publish_joint_states()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.reserve(joints_.size());
    msg.position.reserve(joints_.size());
    msg.velocity.reserve(joints_.size());
    msg.effort.reserve(joints_.size());

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

  bool control_may_run() const
  {
    return enable_control_requested_ && limits_valid_ && motor_status_allows_enable_.load() &&
           any_motor_connected();
  }

  void command_one_dof()
  {
    auto & knee = joints_[kKneeIndex];
    auto & hip = joints_[kHipIndex];

    if (knee.connected) {
      command_impedance(knee, knee.spring_zero_rad);
    }

    if (!hip.connected) {
      return;
    }
    if (!knee.connected) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "1DOF mode requires the knee motor. Hip mirror command is skipped.");
      return;
    }

    const double hip_target = hip_mirror_multiplier_ * knee.last_position_rad;
    hip.md->setMotionMode(mab::MdMode_E::POSITION_PID);
    RCLCPP_INFO(
      get_logger(),
      "Set the Hip motor to POSITION_PID motion mode");
    hip.md->setTargetPosition(hip_target); // This sets the target position of the hip motor to be a scaled version of the knee's last position, effectively mirroring the knee's movement in a 1DOF configuration.
  }

  void command_impedance(Joint & joint, const double target_position_rad)
  {
    if (!motor_status_allows_enable_.load()) {
      return;
    }

    const auto safe_target = checked_position_command(
      joint,
      target_position_rad,
      "target position",
      true);
    if (!safe_target.has_value()) {
      return;
    }

    if (!joint.impedance_configured) {
      if (!check_md(joint, joint.md->setMotionMode(mab::MdMode_E::IMPEDANCE), "set impedance mode")) {
        return;
      }
      if (!motor_status_allows_enable_.load()) {
        return;
      }
      if (!check_md(joint, joint.md->setImpedanceParams(joint.kp, joint.kd), "set impedance gains")) {
        return;
      }
      if (!motor_status_allows_enable_.load()) {
        return;
      }
      if (!check_md(joint, joint.md->setMaxTorque(joint.limits.torque_max_nm), "set max torque")) {
        return;
      }
      joint.impedance_configured = true;
    }

    if (!joint.enabled) {
      if (!motor_status_allows_enable_.load()) {
        return;
      }
      if (!check_md(joint, joint.md->enable(), "enable")) {
        return;
      }
      joint.enabled = true;
      RCLCPP_WARN(
        get_logger(),
        "%s motor ID %d enabled in impedance mode.",
        joint.label.c_str(),
        joint.id);
    }

    if (!motor_status_allows_enable_.load()) {
      return;
    }
    if (!check_md(joint, joint.md->setTargetVelocity(0.0f), "set target velocity")) {
      return;
    }
    if (!motor_status_allows_enable_.load()) {
      return;
    }
    if (!check_md(joint, joint.md->setTargetTorque(0.0f), "set target torque")) {
      return;
    }
    if (!motor_status_allows_enable_.load()) {
      return;
    }
    check_md(
      joint,
      joint.md->setTargetPosition(static_cast<float>(safe_target.value())),
      "set target position");
  }

  bool check_md(const Joint & joint, const mab::MD::Error_t result, const std::string & action)
  {
    if (result == mab::MD::Error_t::OK) {
      return true;
    }
    RCLCPP_ERROR(
      get_logger(),
      "Failed to %s for %s motor ID %d: %s",
      action.c_str(),
      joint.label.c_str(),
      joint.id,
      md_error_to_string(result).c_str());
    return false;
  }

  void handle_motor_status(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      RCLCPP_ERROR(get_logger(), "Received motor_status=false. Disabling motors immediately.");
      motor_status_allows_enable_.store(false);
      disable_all_motors();
      return;
    }

    motor_status_allows_enable_.store(true);
    if (!enable_control_requested_) {
      RCLCPP_WARN(
        get_logger(),
        "Received motor_status=true, but enable_control is false. Motors remain disabled.");
    } else if (!limits_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "Received motor_status=true, but software limits are invalid. Motors remain disabled.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Received motor_status=true. Connected motors may be enabled by the update loop.");
    }
  }

  void handle_spring_zero_position(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!validate_array_size(*msg, "spring_zero_position")) {
      return;
    }

    std::lock_guard<std::mutex> lock(md_mutex_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto & joint = joints_[i];
      if (!joint.connected) {
        continue;
      }
      if (operating_mode_ == OperatingMode::OneDof && i == kHipIndex) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring hip spring zero command in 1DOF mode. Hip mirrors knee position.");
        continue;
      }

      const auto safe_value = checked_position_command(joint, msg->data[i], "spring zero", false);
      if (!safe_value.has_value()) {
        continue;
      }
      joint.spring_zero_rad = safe_value.value();
      RCLCPP_INFO(
        get_logger(),
        "%s spring zero set to %.4f rad.",
        joint.label.c_str(),
        joint.spring_zero_rad);
    }
  }

  void handle_spring_constant(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!validate_array_size(*msg, "spring_constant")) {
      return;
    }
    update_gain_array(*msg, "spring constant", max_spring_constant_, true);
  }

  void handle_damping_constant(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!validate_array_size(*msg, "damping_constant")) {
      return;
    }
    update_gain_array(*msg, "damping constant", max_damping_constant_, false);
  }

  void handle_operating_mode(const std_msgs::msg::String::SharedPtr msg)
  {
    const auto requested_mode = parse_operating_mode(msg->data);
    if (!requested_mode.has_value()) {
      RCLCPP_ERROR(
        get_logger(),
        "Unknown operating mode '%s'. Expected 'impedance' or '1dof'.",
        msg->data.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(md_mutex_);
    if (requested_mode.value() == operating_mode_) {
      RCLCPP_INFO(
        get_logger(),
        "Already in %s mode.",
        operating_mode_to_string(operating_mode_).c_str());
      return;
    }

    for (auto & joint : joints_) {
      joint.spring_zero_rad = 0.0;
      joint.impedance_configured = false;
    }

    operating_mode_ = requested_mode.value();
    if (operating_mode_ == OperatingMode::OneDof && !joints_[kKneeIndex].connected) {
      RCLCPP_ERROR(
        get_logger(),
        "1DOF mode selected, but knee motor is not connected. Hip mirror commands will be skipped.");
    }
    RCLCPP_WARN(
      get_logger(),
      "Operating mode changed to %s. Spring zero targets were reset to 0.0 rad.",
      operating_mode_to_string(operating_mode_).c_str());
  }

  bool validate_array_size(
    const std_msgs::msg::Float64MultiArray & msg,
    const std::string & topic_name) const
  {
    if (msg.data.size() != joints_.size()) {
      RCLCPP_ERROR(
        get_logger(),
        "Expected %zu values on /legwheel/%s, got %zu.",
        joints_.size(),
        topic_name.c_str(),
        msg.data.size());
      return false;
    }
    return true;
  }

  std::optional<double> checked_position_command(
    const Joint & joint,
    const double requested,
    const std::string & command_name,
    const bool throttle_errors)
  {
    if (!is_finite(requested)) {
      if (throttle_errors) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Rejecting non-finite %s %s command.",
          joint.label.c_str(),
          command_name.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Rejecting non-finite %s %s command.",
          joint.label.c_str(),
          command_name.c_str());
      }
      return std::nullopt;
    }

    const auto & limits = joint.limits;
    if (requested >= limits.position_min_rad && requested <= limits.position_max_rad) {
      return requested;
    }

    if (command_limit_policy_ == "clamp") {
      const double clamped = std::clamp(
        requested,
        limits.position_min_rad,
        limits.position_max_rad);
      if (throttle_errors) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Clamping %s %s from %.4f rad to %.4f rad.",
          joint.label.c_str(),
          command_name.c_str(),
          requested,
          clamped);
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Clamping %s %s from %.4f rad to %.4f rad.",
          joint.label.c_str(),
          command_name.c_str(),
          requested,
          clamped);
      }
      return clamped;
    }

    if (throttle_errors) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Rejecting %s %s %.4f rad outside software limits [%.4f, %.4f].",
        joint.label.c_str(),
        command_name.c_str(),
        requested,
        limits.position_min_rad,
        limits.position_max_rad);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting %s %s %.4f rad outside software limits [%.4f, %.4f].",
        joint.label.c_str(),
        command_name.c_str(),
        requested,
        limits.position_min_rad,
        limits.position_max_rad);
    }
    return std::nullopt;
  }

  void update_gain_array(
    const std_msgs::msg::Float64MultiArray & msg,
    const std::string & name,
    const double max_value,
    const bool update_kp)
  {
    std::lock_guard<std::mutex> lock(md_mutex_);
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      auto & joint = joints_[i];
      if (!joint.connected) {
        continue;
      }
      if (operating_mode_ == OperatingMode::OneDof && i == kHipIndex) {
        RCLCPP_WARN(
          get_logger(),
          "Ignoring hip %s command in 1DOF mode. Hip mirrors knee position.",
          name.c_str());
        continue;
      }

      const double value = msg.data[i];
      if (!is_finite(value) || value < 0.0 || value > max_value) {
        RCLCPP_ERROR(
          get_logger(),
          "Rejecting %s %s %.4f. Expected finite value in [0, %.4f].",
          joint.label.c_str(),
          name.c_str(),
          value,
          max_value);
        continue;
      }

      if (update_kp) {
        joint.kp = value;
      } else {
        joint.kd = value;
      }
      joint.impedance_configured = false;

      RCLCPP_INFO(
        get_logger(),
        "%s %s set to %.4f.",
        joint.label.c_str(),
        name.c_str(),
        value);
    }
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
      joint.enabled = false;
      joint.impedance_configured = false;
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

  bool limits_provided_{false};
  bool limits_valid_{false};
  bool enable_control_requested_{false};
  std::atomic_bool motor_status_allows_enable_{false};
  double publish_rate_hz_{100.0};
  double max_spring_constant_{50.0};
  double max_damping_constant_{5.0};
  double hip_mirror_multiplier_{-0.5};
  std::string command_limit_policy_{"reject"};
  OperatingMode operating_mode_{OperatingMode::Impedance};

  mutable std::mutex md_mutex_;

  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spring_zero_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spring_constant_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr damping_constant_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operating_mode_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Md80ImpedanceNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & e) {
    std::cerr << "Fatal error in md80_impedance_node: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
