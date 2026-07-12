#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "MD.hpp"
#include "candle.hpp"

namespace
{
constexpr std::size_t kHipIndex = 0;
constexpr std::size_t kKneeIndex = 1;
constexpr std::size_t kWheelIndex = 2;
constexpr std::size_t kLegJointCount = 2;
constexpr std::size_t kTotalJointCount = 3;
constexpr double kWheelStopDeadbandRadS = 1.0e-4;

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
    wheel_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "/wheel/wheel_state", rclcpp::SensorDataQoS());

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
    wheel_requested_speed_sub_ = create_subscription<std_msgs::msg::Float64>(
      "/wheel/requested_speed",
      10,
      std::bind(&Md80ImpedanceNode::handle_wheel_requested_speed, this, std::placeholders::_1),
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
    bool has_position_limits{true};
    bool control_mode_configured{false};
    bool enabled{false};
    bool knee_impedance_params_configured{false};
    double spring_zero_rad{std::numeric_limits<double>::quiet_NaN()};
    double kp{std::numeric_limits<double>::quiet_NaN()};
    double kd{std::numeric_limits<double>::quiet_NaN()};
    double last_position_rad{0.0};
    double last_velocity_rad_s{0.0};
    double last_effort_nm{0.0};
  };

  void declare_parameters()
  {
    declare_parameter<int>("hip_motor_id", 461);
    declare_parameter<int>("knee_motor_id", 923);
    declare_parameter<int>("wheel_motor_id", 0);
    declare_parameter<bool>("limits_provided", false);

    declare_parameter<double>("hip_position_min_rad", -0.5);
    declare_parameter<double>("hip_position_max_rad", 0.5);
    declare_parameter<double>("hip_velocity_max_rad_s", 1.0);
    declare_parameter<double>("hip_torque_max_nm", 2.0);
    declare_parameter<double>("knee_position_min_rad", -0.5);
    declare_parameter<double>("knee_position_max_rad", 0.5);
    declare_parameter<double>("knee_velocity_max_rad_s", 1.0);
    declare_parameter<double>("knee_torque_max_nm", 2.0);
    declare_parameter<double>("wheel_velocity_max_rad_s", 2.0);
    declare_parameter<double>("wheel_torque_max_nm", 2.0);

    declare_parameter<double>("max_spring_constant", 50.0);
    declare_parameter<double>("max_damping_constant", 5.0);
    declare_parameter<double>("hip_mirror_multiplier", -0.5);
    declare_parameter<std::string>("command_limit_policy", "reject");
    declare_parameter<std::string>("motor_config_json_path", "");

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

    joints_[kWheelIndex].label = "wheel";
    joints_[kWheelIndex].joint_name = "wheel_joint";
    joints_[kWheelIndex].id = get_parameter("wheel_motor_id").as_int();
    joints_[kWheelIndex].has_position_limits = false;
    joints_[kWheelIndex].limits.velocity_max_rad_s =
      get_parameter("wheel_velocity_max_rad_s").as_double();
    joints_[kWheelIndex].limits.torque_max_nm = get_parameter("wheel_torque_max_nm").as_double();

    limits_provided_ = get_parameter("limits_provided").as_bool();
    enable_control_requested_ = get_parameter("enable_control").as_bool();
    publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
    max_spring_constant_ = get_parameter("max_spring_constant").as_double();
    max_damping_constant_ = get_parameter("max_damping_constant").as_double();
    hip_mirror_multiplier_ = get_parameter("hip_mirror_multiplier").as_double();
    command_limit_policy_ = get_parameter("command_limit_policy").as_string();
    motor_config_json_path_ = get_parameter("motor_config_json_path").as_string();

    if (publish_rate_hz_ <= 0.0 || !is_finite(publish_rate_hz_)) {
      throw std::runtime_error("publish_rate_hz must be finite and positive.");
    }
    if (command_limit_policy_ != "reject" && command_limit_policy_ != "clamp") { // This checks if the command limit policy is valid. Command limit policy is used to determine how to handle commands that exceed the software limits. If the policy is "reject", commands outside the limits will be rejected. If the policy is "clamp", commands outside the limits will be clamped to the nearest limit.
      throw std::runtime_error("command_limit_policy must be either 'reject' or 'clamp'.");
    }
    if (!is_finite(hip_mirror_multiplier_)) {
      throw std::runtime_error("hip_mirror_multiplier must be finite.");
    }

    load_motor_config_json();
    validate_motor_config();

    RCLCPP_INFO( // This prints out data into the INFO-level logs
      get_logger(),
      "Expected MD80 IDs: hip=%d, knee=%d",
      joints_[kHipIndex].id,
      joints_[kKneeIndex].id);
    RCLCPP_INFO(
      get_logger(),
      "Expected wheel MD80 ID: wheel=%d",
      joints_[kWheelIndex].id);
  }

  void load_motor_config_json()
  {
    if (motor_config_json_path_.empty()) {
      throw std::runtime_error(
        "motor_config_json_path is empty. md80_impedance_node requires config/motor_config.json.");
    }

    std::ifstream config_file(motor_config_json_path_);
    if (!config_file.is_open()) {
      throw std::runtime_error(
        "Could not open motor config JSON: " + motor_config_json_path_);
    }

    std::stringstream buffer;
    buffer << config_file.rdbuf();
    const std::string contents = buffer.str();

    shutdown_to_startup_deviation_tolerance_ = require_json_number(
      contents,
      "shutdown_to_startup_deviation_tolerance");
    wheel_speed_rampup_time_s_ = require_json_number(
      contents,
      "wheel_speed_rampup_time");
    wheel_default_max_speed_rad_s_ = require_json_number(
      contents,
      "default_max_speed");

    joints_[kHipIndex].spring_zero_rad = require_json_number(
      contents,
      "initial_hip_zero_position_rad");
    joints_[kKneeIndex].spring_zero_rad = require_json_number(
      contents,
      "initial_knee_zero_position_rad");
    joints_[kHipIndex].kp = require_json_number(
      contents,
      "initial_hip_spring_constant");
    joints_[kKneeIndex].kp = require_json_number(
      contents,
      "initial_knee_spring_constant");
    joints_[kHipIndex].kd = require_json_number(
      contents,
      "initial_hip_damping_constant");
    joints_[kKneeIndex].kd = require_json_number(
      contents,
      "initial_knee_damping_constant");

    RCLCPP_INFO(
      get_logger(),
      "Loaded motor config from %s: tolerance=%.4f rad, wheel_ramp=%.4f s, "
      "default_wheel_max=%.4f rad/s, hip_zero=%.4f rad, knee_zero=%.4f rad, "
      "hip_kp=%.4f, knee_kp=%.4f, hip_kd=%.4f, knee_kd=%.4f.",
      motor_config_json_path_.c_str(),
      shutdown_to_startup_deviation_tolerance_,
      wheel_speed_rampup_time_s_,
      wheel_default_max_speed_rad_s_,
      joints_[kHipIndex].spring_zero_rad,
      joints_[kKneeIndex].spring_zero_rad,
      joints_[kHipIndex].kp,
      joints_[kKneeIndex].kp,
      joints_[kHipIndex].kd,
      joints_[kKneeIndex].kd);
  }

  double require_json_number(const std::string & contents, const std::string & key) const
  {
    const std::string number_regex =
      R"regex(([-+]?([0-9]+(\.[0-9]*)?|\.[0-9]+)([eE][-+]?[0-9]+)?))regex";
    const std::regex pattern("\"" + key + R"regex("\s*:\s*)regex" + number_regex);
    std::smatch match;
    if (!std::regex_search(contents, match, pattern)) {
      throw std::runtime_error("motor_config.json is missing numeric key: " + key);
    }

    const double value = std::stod(match[1].str());
    if (!is_finite(value)) {
      throw std::runtime_error("motor_config.json key is not finite: " + key);
    }
    return value;
  }

  void validate_motor_config() const
  {
    if (!is_finite(shutdown_to_startup_deviation_tolerance_) ||
      shutdown_to_startup_deviation_tolerance_ <= 0.0)
    {
      throw std::runtime_error(
        "shutdown_to_startup_deviation_tolerance must be finite and positive.");
    }
    if (!is_finite(wheel_speed_rampup_time_s_) || wheel_speed_rampup_time_s_ <= 0.0) {
      throw std::runtime_error("wheel_speed_rampup_time must be finite and positive.");
    }
    if (!is_finite(wheel_default_max_speed_rad_s_) || wheel_default_max_speed_rad_s_ <= 0.0) {
      throw std::runtime_error("default_max_speed must be finite and positive.");
    }
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
      if (!joint.has_position_limits) {
        continue;
      }
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
      if (!is_finite(joint.spring_zero_rad) ||
        joint.spring_zero_rad < limits.position_min_rad ||
        joint.spring_zero_rad > limits.position_max_rad)
      {
        RCLCPP_ERROR(
          get_logger(),
          "%s initial zero position %.4f rad is outside software limits [%.4f, %.4f].",
          joint.label.c_str(),
          joint.spring_zero_rad,
          limits.position_min_rad,
          limits.position_max_rad);
        limits_valid_ = false;
      }
      if (joint.kp < 0.0 || joint.kp > max_spring_constant_ || !is_finite(joint.kp)) {
        RCLCPP_ERROR(
          get_logger(),
          "%s initial spring constant %.4f is invalid. Expected finite value in [0, %.4f].",
          joint.label.c_str(),
          joint.kp,
          max_spring_constant_);
        limits_valid_ = false;
      }
      if (joint.kd < 0.0 || joint.kd > max_damping_constant_ || !is_finite(joint.kd)) {
        RCLCPP_ERROR(
          get_logger(),
          "%s initial damping constant %.4f is invalid. Expected finite value in [0, %.4f].",
          joint.label.c_str(),
          joint.kd,
          max_damping_constant_);
        limits_valid_ = false;
      }
    }

    if (wheel_default_max_speed_rad_s_ > joints_[kWheelIndex].limits.velocity_max_rad_s) {
      RCLCPP_ERROR(
        get_logger(),
        "default_max_speed %.4f rad/s exceeds wheel_velocity_max_rad_s %.4f rad/s.",
        wheel_default_max_speed_rad_s_,
        joints_[kWheelIndex].limits.velocity_max_rad_s);
      limits_valid_ = false;
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
    if (joint.id <= 0) {
      RCLCPP_WARN(
        get_logger(),
        "%s motor ID is %d. Configure a positive MD80 ID to use this motor.",
        joint.label.c_str(),
        joint.id);
      return;
    }

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
    joint.control_mode_configured = false;
    joint.enabled = false;
    joint.knee_impedance_params_configured = false;

    configure_motor_for_runtime_control(joint);

    RCLCPP_INFO(
      get_logger(),
      "Connected %s_joint to MD80 ID %d.",
      joint.label.c_str(),
      joint.id);
  }

  bool configure_all_motors_for_runtime_control()
  {
    bool all_ok = true;

    for (auto & joint : joints_) {
      if (!joint.connected) {
        continue;
      }

      configure_motor_for_runtime_control(joint);

      if (!joint.control_mode_configured) {
        all_ok = false;
      }
    }

    return all_ok;
  }

  void configure_motor_for_runtime_control(Joint & joint)
  {
    if (!joint.connected) {
      return;
    }

    // Motion modes are configured while motors are disabled. The command loop does
    // not switch modes while sending targets: knee is impedance, hip is position
    // PID, and wheel is velocity PID.
    if (joint.label == "hip") {
      if (check_md(
          joint,
          joint.md->setMotionMode(mab::MdMode_E::POSITION_PID),
          "set position PID mode"))
      {
        joint.control_mode_configured = true;
        RCLCPP_INFO(
          get_logger(),
          "Configured hip motor ID %d for POSITION_PID mode.",
          joint.id);
      }
      return;
    }

    if (joint.label == "knee") {
      if (check_md(
          joint,
          joint.md->setMotionMode(mab::MdMode_E::IMPEDANCE),
          "set impedance mode"))
      {
        joint.control_mode_configured = true;
        RCLCPP_INFO(
          get_logger(),
          "Configured knee motor ID %d for IMPEDANCE mode.",
          joint.id);
      }
      joint.knee_impedance_params_configured = false;
      return;
    }

    if (joint.label == "wheel") {
      if (!check_md(
          joint,
          joint.md->setMotionMode(mab::MdMode_E::VELOCITY_PID),
          "set velocity PID mode"))
      {
        return;
      }
      if (!check_md(
          joint,
          joint.md->setMaxTorque(joint.limits.torque_max_nm),
          "set max torque"))
      {
        return;
      }
      joint.control_mode_configured = true;
      RCLCPP_INFO(
        get_logger(),
        "Configured wheel motor ID %d for VELOCITY_PID mode.",
        joint.id);
    }
  }

  bool any_motor_connected() const
  {
    return std::any_of(joints_.begin(), joints_.end(), [](const auto & joint) {
      return joint.connected;
    });
  }

  void update()
  {
    const double dt = update_dt_seconds();

    std::lock_guard<std::mutex> lock(md_mutex_);
    read_connected_motors();
    publish_joint_states();
    publish_wheel_state();

    if (!control_may_run()) {
      return;
    }

    command_fixed_one_dof();
    command_wheel_velocity(dt);
  }

  double update_dt_seconds()
  {
    const auto current_time = now();
    double dt = 1.0 / publish_rate_hz_;
    if (last_update_time_.has_value()) {
      dt = (current_time - last_update_time_.value()).seconds();
    }
    last_update_time_ = current_time;

    if (!is_finite(dt) || dt <= 0.0) {
      return 1.0 / publish_rate_hz_;
    }
    return std::clamp(dt, 0.0, 0.1);
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
    msg.name.reserve(kLegJointCount);
    msg.position.reserve(kLegJointCount);
    msg.velocity.reserve(kLegJointCount);
    msg.effort.reserve(kLegJointCount);

    for (std::size_t i = 0; i < kLegJointCount; ++i) {
      const auto & joint = joints_[i];
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

  void publish_wheel_state()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();

    const auto & wheel = joints_[kWheelIndex];
    if (wheel.connected) {
      msg.name.push_back(wheel.joint_name);
      msg.position.push_back(wheel.last_position_rad);
      msg.velocity.push_back(wheel.last_velocity_rad_s);
      msg.effort.push_back(wheel.last_effort_nm);
    }

    wheel_state_pub_->publish(msg);
  }

  bool control_may_run() const
  {
    return enable_control_requested_ && limits_valid_ && motor_status_allows_enable_.load() &&
           any_motor_connected();
  }

  void command_fixed_one_dof()
  {
    auto & knee = joints_[kKneeIndex];
    auto & hip = joints_[kHipIndex];

    bool knee_command_sent = false;
    if (knee.connected) {
      knee_command_sent = command_knee_impedance(knee);
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
    if (!knee_command_sent) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Knee impedance command was not accepted. Hip mirror command is skipped.");
      return;
    }

    const double hip_target = hip_mirror_multiplier_ * knee.last_position_rad;
    command_hip_position_pid(hip, hip_target);
  }

  bool command_knee_impedance(Joint & joint)
  {
    if (!motor_status_allows_enable_.load()) {
      return false;
    }
    if (!joint.control_mode_configured) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Knee motor ID %d was not configured for IMPEDANCE mode. Command skipped.",
        joint.id);
      return false;
    }

    const auto safe_target = checked_position_command(
      joint,
      joint.spring_zero_rad,
      "target position",
      true);
    if (!safe_target.has_value()) {
      return false;
    }

    // The knee is already in IMPEDANCE mode from initialization. Gains may still
    // be updated live, so only impedance parameters are refreshed here.
    if (!joint.knee_impedance_params_configured) {
      if (!check_md(
          joint,
          joint.md->setImpedanceParams(joint.kp, joint.kd),
          "set impedance gains"))
      {
        return false;
      }
      if (!motor_status_allows_enable_.load()) {
        return false;
      }
      if (!check_md(
          joint,
          joint.md->setMaxTorque(joint.limits.torque_max_nm),
          "set max torque"))
      {
        return false;
      }
      joint.knee_impedance_params_configured = true;
    }

    if (!joint.enabled) {
      if (!motor_status_allows_enable_.load()) {
        return false;
      }
      if (!check_md(joint, joint.md->enable(), "enable")) {
        return false;
      }
      joint.enabled = true;
      RCLCPP_WARN(
        get_logger(),
        "%s motor ID %d enabled in knee impedance control.",
        joint.label.c_str(),
        joint.id);
    }

    if (!motor_status_allows_enable_.load()) {
      return false;
    }
    if (!check_md(joint, joint.md->setTargetVelocity(0.0f), "set target velocity")) {
      return false;
    }
    if (!motor_status_allows_enable_.load()) {
      return false;
    }
    if (!check_md(joint, joint.md->setTargetTorque(0.0f), "set target torque")) {
      return false;
    }
    if (!motor_status_allows_enable_.load()) {
      return false;
    }
    return check_md(
      joint,
      joint.md->setTargetPosition(static_cast<float>(safe_target.value())),
      "set target position");
  }

  void command_hip_position_pid(Joint & joint, const double target_position_rad)
  {
    if (!motor_status_allows_enable_.load()) {
      return;
    }
    if (!joint.control_mode_configured) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Hip motor ID %d was not configured for POSITION_PID mode. Command skipped.",
        joint.id);
      return;
    }

    // This sets the target position of the hip motor to a scaled version of the
    // knee encoder position, effectively mirroring the knee in the fixed 1DOF
    // configuration.
    const auto safe_target = checked_position_command(
      joint,
      target_position_rad,
      "mirror target position",
      true);
    if (!safe_target.has_value()) {
      return;
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
        "%s motor ID %d enabled in hip POSITION_PID mirror control.",
        joint.label.c_str(),
        joint.id);
    }

    if (!motor_status_allows_enable_.load()) {
      return;
    }
    check_md(
      joint,
      joint.md->setTargetPosition(static_cast<float>(safe_target.value())),
      "set position PID target");
  }

  void command_wheel_velocity(const double dt)
  {
    auto & wheel = joints_[kWheelIndex];
    if (!wheel.connected) {
      return;
    }
    if (!wheel.control_mode_configured) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Wheel motor ID %d was not configured for VELOCITY_PID mode. Command skipped.",
        wheel.id);
      return;
    }

    const double target_speed = std::clamp(
      wheel_requested_speed_rad_s_,
      -wheel.limits.velocity_max_rad_s,
      wheel.limits.velocity_max_rad_s);

    // Rate limit the command inside the hardware node as a final guard against
    // abrupt keyboard/controller messages causing velocity or torque jumps.
    const double max_delta =
      (wheel.limits.velocity_max_rad_s / wheel_speed_rampup_time_s_) * dt;
    const double delta = std::clamp(
      target_speed - wheel_commanded_speed_rad_s_,
      -max_delta,
      max_delta);
    wheel_commanded_speed_rad_s_ += delta;

    if (std::abs(target_speed) < kWheelStopDeadbandRadS &&
      std::abs(wheel_commanded_speed_rad_s_) < kWheelStopDeadbandRadS)
    {
      wheel_commanded_speed_rad_s_ = 0.0;
    }

    if (!wheel.enabled) {
      if (std::abs(wheel_commanded_speed_rad_s_) < kWheelStopDeadbandRadS) {
        return;
      }
      if (!check_md(wheel, wheel.md->setTargetVelocity(0.0f), "set initial wheel velocity")) {
        return;
      }
      if (!motor_status_allows_enable_.load()) {
        return;
      }
      if (!check_md(wheel, wheel.md->enable(), "enable")) {
        return;
      }
      wheel.enabled = true;
      RCLCPP_WARN(
        get_logger(),
        "Wheel motor ID %d enabled in VELOCITY_PID control.",
        wheel.id);
    }

    if (!motor_status_allows_enable_.load()) {
      return;
    }
    check_md(
      wheel,
      wheel.md->setTargetVelocity(static_cast<float>(wheel_commanded_speed_rad_s_)),
      "set wheel target velocity");
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
      reset_wheel_speed_command();
      if (startup_suppressed_by_shutdown_deviation_) {
        report_shutdown_position_save_blocked();
        return;
      }
      save_motor_positions_on_shutdown();
      return;
    }

    if (!enable_control_requested_) {
      RCLCPP_WARN(
        get_logger(),
        "Received motor_status=true, but enable_control is false. Motors remain disabled.");
      motor_status_allows_enable_.store(false);
      return;
    }

    if (!limits_valid_) {
      RCLCPP_ERROR(
        get_logger(),
        "Received motor_status=true, but software limits are invalid. Motors remain disabled.");
      motor_status_allows_enable_.store(false);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(md_mutex_);

      read_connected_motors();
      if (!startup_positions_within_shutdown_tolerance()) {
        startup_suppressed_by_shutdown_deviation_ = true;
        RCLCPP_WARN(
          get_logger(),
          "Received motor_status=true, but encoder positions moved too far from the saved "
          "shutdown pose. Motors remain disabled.");
        motor_status_allows_enable_.store(false);
        return;
      }
      startup_suppressed_by_shutdown_deviation_ = false;

      const bool configured_ok = configure_all_motors_for_runtime_control();
      if (!configured_ok) {
        RCLCPP_ERROR(
          get_logger(),
          "Received motor_status=true, but failed to configure one or more motor modes. "
          "Motors remain disabled.");
        motor_status_allows_enable_.store(false);
        return;
      }
    }

    motor_status_allows_enable_.store(true);

    RCLCPP_WARN(
      get_logger(),
      "Received motor_status=true. Motor modes reconfigured; connected motors may now be enabled by the update loop.");
  }

  void reset_wheel_speed_command()
  {
    std::lock_guard<std::mutex> lock(md_mutex_);
    wheel_requested_speed_rad_s_ = 0.0;
    wheel_commanded_speed_rad_s_ = 0.0;
  }

  void report_shutdown_position_save_blocked()
  {
    std::lock_guard<std::mutex> lock(md_mutex_);

    read_connected_motors();
    const bool now_within_tolerance = startup_positions_within_shutdown_tolerance();

    RCLCPP_WARN(
      get_logger(),
      "motor_status=false was received after startup was suppressed by the shutdown-to-startup "
      "deviation check. A new motor_position_on_shutdown will not be saved.");

    if (now_within_tolerance) {
      RCLCPP_WARN(
        get_logger(),
        "Current encoder positions are now within tolerance, but the saved shutdown pose is still "
        "protected. Publish motor_status=true to clear this interlock.");
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Current encoder positions are still outside the saved shutdown tolerance. Move the motors "
        "back near the logged expected positions before enabling again.");
    }
  }

  void save_motor_positions_on_shutdown()
  {
    std::lock_guard<std::mutex> lock(md_mutex_);

    // The motors are already disabled before this is called. Take a fresh encoder
    // read where possible, then save the values used as the next enable reference.
    read_connected_motors();
    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto & joint = joints_[i];
      if (!joint.connected) {
        continue;
      }
      motor_position_on_shutdown_[i] = joint.last_position_rad;
      RCLCPP_INFO(
        get_logger(),
        "Saved %s motor_position_on_shutdown=%.4f rad.",
        joint.label.c_str(),
        motor_position_on_shutdown_[i]);
    }
  }

  bool startup_positions_within_shutdown_tolerance() const
  {
    bool all_within_tolerance = true;

    for (std::size_t i = 0; i < joints_.size(); ++i) {
      const auto & joint = joints_[i];
      if (!joint.connected) {
        continue;
      }

      const double expected = motor_position_on_shutdown_[i];
      const double actual = joint.last_position_rad;
      const double deviation = actual - expected;
      const double abs_deviation = std::abs(deviation);
      const bool within_tolerance =
        abs_deviation <= shutdown_to_startup_deviation_tolerance_;

      if (within_tolerance) {
        RCLCPP_INFO(
          get_logger(),
          "%s startup position check: expected=%.4f rad, actual=%.4f rad, "
          "deviation=%.4f rad, tolerance=%.4f rad.",
          joint.label.c_str(),
          expected,
          actual,
          deviation,
          shutdown_to_startup_deviation_tolerance_);
      } else {
        all_within_tolerance = false;
        RCLCPP_WARN(
          get_logger(),
          "%s startup position check failed: expected=%.4f rad, actual=%.4f rad, "
          "deviation=%.4f rad, tolerance=%.4f rad.",
          joint.label.c_str(),
          expected,
          actual,
          deviation,
          shutdown_to_startup_deviation_tolerance_);
      }
    }

    return all_within_tolerance;
  }

  void handle_spring_zero_position(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!validate_array_size(*msg, "spring_zero_position")) {
      return;
    }

    std::lock_guard<std::mutex> lock(md_mutex_);
    for (std::size_t i = 0; i < kLegJointCount; ++i) {
      auto & joint = joints_[i];
      if (!joint.connected) {
        continue;
      }
      if (i == kHipIndex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
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

  void handle_wheel_requested_speed(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!is_finite(msg->data)) {
      RCLCPP_ERROR(get_logger(), "Rejecting non-finite /wheel/requested_speed command.");
      return;
    }

    std::lock_guard<std::mutex> lock(md_mutex_);
    const auto & wheel = joints_[kWheelIndex];
    if (!is_finite(wheel.limits.velocity_max_rad_s) || wheel.limits.velocity_max_rad_s <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Rejecting wheel speed command because wheel velocity limit is invalid.");
      return;
    }
    const double clamped_speed = std::clamp(
      msg->data,
      -wheel.limits.velocity_max_rad_s,
      wheel.limits.velocity_max_rad_s);

    if (clamped_speed != msg->data) {
      RCLCPP_WARN(
        get_logger(),
        "Clamping requested wheel speed from %.4f rad/s to %.4f rad/s. Limit is %.4f rad/s.",
        msg->data,
        clamped_speed,
        wheel.limits.velocity_max_rad_s);
    }

    wheel_requested_speed_rad_s_ = clamped_speed;
  }

  bool validate_array_size(
    const std_msgs::msg::Float64MultiArray & msg,
    const std::string & topic_name) const
  {
    if (msg.data.size() != kLegJointCount) {
      RCLCPP_ERROR(
        get_logger(),
        "Expected %zu values on /legwheel/%s, got %zu.",
        kLegJointCount,
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
    for (std::size_t i = 0; i < kLegJointCount; ++i) {
      auto & joint = joints_[i];
      if (!joint.connected) {
        continue;
      }
      if (i == kHipIndex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
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
      joint.knee_impedance_params_configured = false;

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
      joint.control_mode_configured = false; // so that I know I need to reconfigure the control mode if I re-enable the motor
      joint.knee_impedance_params_configured = false;
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
  std::array<Joint, kTotalJointCount> joints_;
  std::unordered_set<int> discovered_ids_;

  bool limits_provided_{false};
  bool limits_valid_{false};
  bool enable_control_requested_{false};
  bool startup_suppressed_by_shutdown_deviation_{false};
  std::atomic_bool motor_status_allows_enable_{false};
  double publish_rate_hz_{100.0};
  double max_spring_constant_{50.0};
  double max_damping_constant_{5.0};
  double hip_mirror_multiplier_{-0.5};
  std::string command_limit_policy_{"reject"};
  std::string motor_config_json_path_;
  double shutdown_to_startup_deviation_tolerance_{std::numeric_limits<double>::quiet_NaN()};
  double wheel_speed_rampup_time_s_{std::numeric_limits<double>::quiet_NaN()};
  double wheel_default_max_speed_rad_s_{std::numeric_limits<double>::quiet_NaN()};
  double wheel_requested_speed_rad_s_{0.0};
  double wheel_commanded_speed_rad_s_{0.0};
  std::array<double, kTotalJointCount> motor_position_on_shutdown_{0.0, 0.0, 0.0};
  std::optional<rclcpp::Time> last_update_time_;

  mutable std::mutex md_mutex_;

  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spring_zero_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr spring_constant_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr damping_constant_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_requested_speed_sub_;
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
