#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <sys/select.h>
#include <unistd.h>

namespace
{
constexpr std::size_t kHipIndex = 0;
constexpr std::size_t kKneeIndex = 1;

std::string lowercase(std::string value)
{
  for (auto & character : value) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }
  return value;
}

bool parse_joint(const std::string & token, std::size_t & index)
{
  const auto joint = lowercase(token);
  if (joint == "hip") {
    index = kHipIndex;
    return true;
  }
  if (joint == "knee") {
    index = kKneeIndex;
    return true;
  }
  return false;
}

std_msgs::msg::Float64MultiArray make_array_message(const std::array<double, 2> & values)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(values.begin(), values.end());
  return msg;
}
}  // namespace

class LegwheelControllerNode : public rclcpp::Node
{
public:
  LegwheelControllerNode()
  : Node("legwheel_controller_node")
  {
    declare_parameter<double>("initial_hip_zero_position_rad", 0.0);
    declare_parameter<double>("initial_knee_zero_position_rad", 0.0);
    declare_parameter<double>("initial_hip_spring_constant", 1.0);
    declare_parameter<double>("initial_knee_spring_constant", 1.0);
    declare_parameter<double>("initial_hip_damping_constant", 0.05);
    declare_parameter<double>("initial_knee_damping_constant", 0.05);
    declare_parameter<bool>("publish_initial_commands", true);
    declare_parameter<bool>("auto_enable", false);

    spring_zero_position_[kHipIndex] =
      get_parameter("initial_hip_zero_position_rad").as_double();
    spring_zero_position_[kKneeIndex] =
      get_parameter("initial_knee_zero_position_rad").as_double();
    spring_constant_[kHipIndex] = get_parameter("initial_hip_spring_constant").as_double();
    spring_constant_[kKneeIndex] = get_parameter("initial_knee_spring_constant").as_double();
    damping_constant_[kHipIndex] = get_parameter("initial_hip_damping_constant").as_double();
    damping_constant_[kKneeIndex] = get_parameter("initial_knee_damping_constant").as_double();
    publish_initial_commands_ = get_parameter("publish_initial_commands").as_bool();
    auto_enable_ = get_parameter("auto_enable").as_bool();

    validate_initial_values();

    motor_status_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/legwheel/motor_status", rclcpp::QoS(10).reliable());
    spring_zero_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/legwheel/spring_zero_position", 10);
    spring_constant_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/legwheel/spring_constant", 10);
    damping_constant_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/legwheel/damping_constant", 10);
    command_sub_ = create_subscription<std_msgs::msg::String>(
      "/legwheel/controller_command",
      10,
      std::bind(&LegwheelControllerNode::handle_command_message, this, std::placeholders::_1));

    startup_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LegwheelControllerNode::publish_startup_commands, this));

    RCLCPP_INFO(
      get_logger(),
      "Legwheel controller ready. Type commands followed by Enter, or publish std_msgs/String to /legwheel/controller_command.");
    input_thread_ = std::thread(&LegwheelControllerNode::input_loop, this);
  }

  ~LegwheelControllerNode() override
  {
    stop_requested_.store(true);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
  }

private:
  void validate_initial_values()
  {
    for (const auto value : spring_zero_position_) {
      if (!std::isfinite(value)) {
        throw std::runtime_error("Initial spring zero positions must be finite.");
      }
    }
    for (const auto value : spring_constant_) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::runtime_error("Initial spring constants must be finite and non-negative.");
      }
    }
    for (const auto value : damping_constant_) {
      if (!std::isfinite(value) || value < 0.0) {
        throw std::runtime_error("Initial damping constants must be finite and non-negative.");
      }
    }
  }

  void publish_startup_commands()
  {
    if (publish_initial_commands_) {
      publish_all_command_arrays();
      if (startup_publish_count_ == 0) {
        RCLCPP_INFO(
          get_logger(),
          "Publishing initial spring zero, spring constant, and damping constant commands.");
      }
    }

    if (auto_enable_ && !startup_enable_published_) {
      publish_motor_status(true);
      startup_enable_published_ = true;
      RCLCPP_WARN(
        get_logger(),
        "auto_enable is true: published /legwheel/motor_status=true.");
    } else if (!auto_enable_ && startup_publish_count_ == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Motors are not enabled by the controller on startup. Type 'enable' or 'e' to allow control.");
    }

    ++startup_publish_count_;
    if (startup_publish_count_ >= 5) {
      startup_timer_->cancel();
    }
  }

  void input_loop()
  {
    while (!stop_requested_.load()) {
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(STDIN_FILENO, &read_fds);

      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;

      const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
      if (ready < 0) {
        RCLCPP_ERROR(get_logger(), "Failed while polling stdin. Controller input is stopping.");
        return;
      }
      if (ready == 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
        continue;
      }

      std::string line;
      if (!std::getline(std::cin, line)) {
        RCLCPP_WARN(get_logger(), "Controller stdin closed. Command input is stopping.");
        return;
      }
      handle_command(line);
    }
  }

  void handle_command(const std::string & line)
  {
    std::istringstream stream(line);
    std::vector<std::string> tokens;
    std::string token;
    while (stream >> token) {
      tokens.push_back(token);
    }

    if (tokens.empty()) {
      return;
    }

    const auto command = lowercase(tokens[0]);
    if (command == "s" || command == "stop" || command == "disable") {
      publish_motor_status(false);
      RCLCPP_ERROR(get_logger(), "Published motor_status=false.");
      return;
    }

    if (command == "e" || command == "enable" || command == "start") {
      publish_motor_status(true);
      RCLCPP_WARN(get_logger(), "Published motor_status=true.");
      return;
    }

    if (command == "help" || command == "h" || command == "?") {
      print_help();
      return;
    }

    if (command == "status") {
      print_status();
      return;
    }

    if (tokens.size() != 3) {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid command. Expected '<hip|knee> <set_zeropos|set_spring|set_damp> <value>', 'e', or 's'.");
      return;
    }

    std::size_t joint_index = 0;
    if (!parse_joint(tokens[0], joint_index)) {
      RCLCPP_ERROR(get_logger(), "Unknown joint '%s'. Use 'hip' or 'knee'.", tokens[0].c_str());
      return;
    }

    double value = 0.0;
    try {
      std::size_t parsed_chars = 0;
      value = std::stod(tokens[2], &parsed_chars);
      if (parsed_chars != tokens[2].size()) {
        throw std::invalid_argument("trailing characters");
      }
    } catch (const std::exception &) {
      RCLCPP_ERROR(get_logger(), "Invalid numeric value '%s'.", tokens[2].c_str());
      return;
    }

    if (!std::isfinite(value)) {
      RCLCPP_ERROR(get_logger(), "Value must be finite.");
      return;
    }

    const auto action = lowercase(tokens[1]);
    if (action == "set_zeropos" || action == "set_zero" || action == "set_zero_position") {
      set_spring_zero(joint_index, value);
      return;
    }
    if (action == "set_spring") {
      if (value < 0.0) {
        RCLCPP_ERROR(get_logger(), "Spring constant must be non-negative.");
        return;
      }
      set_spring_constant(joint_index, value);
      return;
    }
    if (action == "set_damp" || action == "set_damping") {
      if (value < 0.0) {
        RCLCPP_ERROR(get_logger(), "Damping constant must be non-negative.");
        return;
      }
      set_damping_constant(joint_index, value);
      return;
    }

    RCLCPP_ERROR(
      get_logger(),
      "Unknown action '%s'. Use set_zeropos, set_spring, or set_damp.",
      tokens[1].c_str());
  }

  void handle_command_message(const std_msgs::msg::String::SharedPtr msg)
  {
    handle_command(msg->data);
  }

  void set_spring_zero(const std::size_t joint_index, const double value)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      spring_zero_position_[joint_index] = value;
      spring_zero_pub_->publish(make_array_message(spring_zero_position_));
    }
    RCLCPP_INFO(
      get_logger(),
      "Published %s spring zero position %.4f rad.",
      joint_name(joint_index).c_str(),
      value);
  }

  void set_spring_constant(const std::size_t joint_index, const double value)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      spring_constant_[joint_index] = value;
      spring_constant_pub_->publish(make_array_message(spring_constant_));
    }
    RCLCPP_INFO(
      get_logger(),
      "Published %s spring constant %.4f Nm/rad.",
      joint_name(joint_index).c_str(),
      value);
  }

  void set_damping_constant(const std::size_t joint_index, const double value)
  {
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      damping_constant_[joint_index] = value;
      damping_constant_pub_->publish(make_array_message(damping_constant_));
    }
    RCLCPP_INFO(
      get_logger(),
      "Published %s damping constant %.4f N/(rad/s).",
      joint_name(joint_index).c_str(),
      value);
  }

  void publish_all_command_arrays()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    spring_zero_pub_->publish(make_array_message(spring_zero_position_));
    spring_constant_pub_->publish(make_array_message(spring_constant_));
    damping_constant_pub_->publish(make_array_message(damping_constant_));
  }

  void publish_motor_status(const bool enabled)
  {
    std_msgs::msg::Bool msg;
    msg.data = enabled;
    motor_status_pub_->publish(msg);
  }

  void print_help() const
  {
    RCLCPP_INFO(
      get_logger(),
      "Commands: 'e' or 'enable' allow motors; 's' disables motors; "
      "'hip set_zeropos 0.0'; 'knee set_spring 4.0'; 'hip set_damp 5.0'; 'status'.");
  }

  void print_status()
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    RCLCPP_INFO(
      get_logger(),
      "Current commands: zero=[%.4f, %.4f] rad, spring=[%.4f, %.4f] Nm/rad, damping=[%.4f, %.4f] N/(rad/s).",
      spring_zero_position_[kHipIndex],
      spring_zero_position_[kKneeIndex],
      spring_constant_[kHipIndex],
      spring_constant_[kKneeIndex],
      damping_constant_[kHipIndex],
      damping_constant_[kKneeIndex]);
  }

  std::string joint_name(const std::size_t joint_index) const
  {
    return joint_index == kHipIndex ? "hip" : "knee";
  }

  std::array<double, 2> spring_zero_position_{0.0, 0.0};
  std::array<double, 2> spring_constant_{1.0, 1.0};
  std::array<double, 2> damping_constant_{0.05, 0.05};
  bool publish_initial_commands_{true};
  bool auto_enable_{false};
  int startup_publish_count_{0};
  bool startup_enable_published_{false};

  std::mutex command_mutex_;
  std::atomic_bool stop_requested_{false};
  std::thread input_thread_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr spring_zero_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr spring_constant_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr damping_constant_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<LegwheelControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "Fatal error in legwheel_controller_node: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
