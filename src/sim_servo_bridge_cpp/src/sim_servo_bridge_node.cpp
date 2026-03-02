#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "servo_msgs/msg/servo_command.hpp"
#include "servo_msgs/msg/servo_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

class SimServoBridgeNode : public rclcpp::Node {
 public:
  SimServoBridgeNode()
  : Node("sim_servo_bridge"),
    latest_joint_cmd_rad_{},
    latest_joint_state_rad_{} {
    const auto default_mapping = std::vector<int64_t>{
      1, 2, 3, 4, 5, 6, 7, 8, -1, 9, 10, 11, 12, 13, 14, -1
    };

    joint_cmd_topic_ = this->declare_parameter<std::string>(
      "joint_cmd_topic", "/sim/joint_cmd");
    joint_state_fb_topic_ = this->declare_parameter<std::string>(
      "joint_state_fb_topic", "/sim/joint_state_fb");
    servo_cmd_topic_ = this->declare_parameter<std::string>(
      "servo_cmd_topic", "/servo/command");
    servo_state_topic_ = this->declare_parameter<std::string>(
      "servo_state_topic", "/servo/state");

    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    speed_ = this->declare_parameter<int64_t>("speed", 100);
    servo_type_ = this->declare_parameter<std::string>("servo_type", "bus");
    min_pulse_us_ = this->declare_parameter<double>("min_pulse_us", 500.0);
    max_pulse_us_ = this->declare_parameter<double>("max_pulse_us", 2500.0);
    center_pulse_us_ = this->declare_parameter<double>("center_pulse_us", 1500.0);
    us_per_rad_ = this->declare_parameter<double>("us_per_rad", 636.6197723675814);
    joint_to_servo_id_ = this->declare_parameter<std::vector<int64_t>>(
      "joint_to_servo_id", default_mapping);

    const auto servo_ids_with_offset = this->declare_parameter<std::vector<int64_t>>(
      "servo_ids_with_offset", std::vector<int64_t>{});
    const auto servo_offsets_us = this->declare_parameter<std::vector<double>>(
      "servo_offsets_us", std::vector<double>{});
    debug_ = this->declare_parameter<bool>("debug", false);

    validate_and_fix_parameters(default_mapping);
    load_servo_offsets(servo_ids_with_offset, servo_offsets_us);
    build_reverse_mapping();

    latest_joint_cmd_rad_.fill(0.0);
    latest_joint_state_rad_.fill(0.0);

    servo_cmd_pub_ = this->create_publisher<servo_msgs::msg::ServoCommand>(
      servo_cmd_topic_, 100);
    joint_state_fb_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      joint_state_fb_topic_, 10);

    joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      joint_cmd_topic_,
      10,
      std::bind(&SimServoBridgeNode::on_joint_cmd, this, std::placeholders::_1));

    servo_state_sub_ = this->create_subscription<servo_msgs::msg::ServoState>(
      servo_state_topic_,
      100,
      std::bind(&SimServoBridgeNode::on_servo_state, this, std::placeholders::_1));

    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    publish_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&SimServoBridgeNode::on_publish_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "sim_servo_bridge_node started: %s -> %s, %s -> %s, rate=%.2fHz",
      joint_cmd_topic_.c_str(),
      servo_cmd_topic_.c_str(),
      servo_state_topic_.c_str(),
      joint_state_fb_topic_.c_str(),
      publish_rate_hz_);
  }

 private:
  static constexpr std::size_t kJointCount = 16;
  static constexpr int64_t kIgnoreServoId = -1;

  void validate_and_fix_parameters(const std::vector<int64_t> & default_mapping) {
    if (publish_rate_hz_ <= 0.0) {
      RCLCPP_WARN(
        this->get_logger(),
        "publish_rate_hz=%.3f invalid, reset to 50.0",
        publish_rate_hz_);
      publish_rate_hz_ = 50.0;
    }

    if (std::abs(us_per_rad_) < 1e-6) {
      RCLCPP_WARN(
        this->get_logger(),
        "us_per_rad=%.6f too small, reset to default",
        us_per_rad_);
      us_per_rad_ = 636.6197723675814;
    }

    if (min_pulse_us_ > max_pulse_us_) {
      RCLCPP_WARN(
        this->get_logger(),
        "min_pulse_us(%.2f) > max_pulse_us(%.2f), swapping",
        min_pulse_us_,
        max_pulse_us_);
      std::swap(min_pulse_us_, max_pulse_us_);
    }

    if (joint_to_servo_id_.size() != kJointCount) {
      RCLCPP_WARN(
        this->get_logger(),
        "joint_to_servo_id size=%zu invalid, expected=%zu, using default mapping",
        joint_to_servo_id_.size(),
        kJointCount);
      joint_to_servo_id_ = default_mapping;
    }
  }

  void load_servo_offsets(
    const std::vector<int64_t> & servo_ids_with_offset,
    const std::vector<double> & servo_offsets_us) {
    const auto pair_count = std::min(servo_ids_with_offset.size(), servo_offsets_us.size());
    for (std::size_t i = 0; i < pair_count; ++i) {
      servo_offset_us_[servo_ids_with_offset[i]] = servo_offsets_us[i];
    }

    if (servo_ids_with_offset.size() != servo_offsets_us.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "servo_ids_with_offset size=%zu != servo_offsets_us size=%zu, extra values ignored",
        servo_ids_with_offset.size(),
        servo_offsets_us.size());
    }
  }

  void build_reverse_mapping() {
    servo_to_joint_idx_.clear();
    for (std::size_t idx = 0; idx < joint_to_servo_id_.size(); ++idx) {
      const auto servo_id = joint_to_servo_id_[idx];
      if (servo_id == kIgnoreServoId) {
        continue;
      }

      if (servo_to_joint_idx_.find(servo_id) != servo_to_joint_idx_.end()) {
        RCLCPP_WARN(
          this->get_logger(),
          "duplicate servo_id=%ld in joint_to_servo_id, keep first mapping",
          servo_id);
        continue;
      }
      servo_to_joint_idx_.emplace(servo_id, idx);
    }
  }

  double servo_offset(int64_t servo_id) const {
    const auto it = servo_offset_us_.find(servo_id);
    if (it == servo_offset_us_.end()) {
      return 0.0;
    }
    return it->second;
  }

  void on_joint_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!msg || msg->data.size() < kJointCount) {
      RCLCPP_WARN(
        this->get_logger(),
        "ignore /sim/joint_cmd: length=%zu < %zu",
        msg ? msg->data.size() : 0U,
        kJointCount);
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (std::size_t i = 0; i < kJointCount; ++i) {
      latest_joint_cmd_rad_[i] = static_cast<double>(msg->data[i]);
    }
    has_joint_cmd_ = true;
  }

  void on_servo_state(const servo_msgs::msg::ServoState::SharedPtr msg) {
    if (!msg || msg->servo_type != "bus") {
      return;
    }

    const auto it = servo_to_joint_idx_.find(static_cast<int64_t>(msg->servo_id));
    if (it == servo_to_joint_idx_.end()) {
      return;
    }

    const int64_t servo_id = static_cast<int64_t>(msg->servo_id);
    const double clamped_us = std::clamp(
      static_cast<double>(msg->position),
      min_pulse_us_,
      max_pulse_us_);
    const double rad = (clamped_us - center_pulse_us_ - servo_offset(servo_id)) / us_per_rad_;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_joint_state_rad_[it->second] = rad;
    has_servo_state_ = true;
  }

  void on_publish_timer() {
    std::array<double, kJointCount> cmd_snapshot{};
    std::array<double, kJointCount> state_snapshot{};
    bool has_cmd = false;
    bool has_state = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      cmd_snapshot = latest_joint_cmd_rad_;
      state_snapshot = latest_joint_state_rad_;
      has_cmd = has_joint_cmd_;
      has_state = has_servo_state_;
    }

    if (has_cmd) {
      publish_servo_commands(cmd_snapshot);
    }
    if (has_state) {
      publish_joint_feedback(state_snapshot);
    }

    if (debug_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "bridge stats: cmd_pub=%llu, fb_pub=%llu, invalid_joint=%llu",
        static_cast<unsigned long long>(command_publish_count_),
        static_cast<unsigned long long>(state_publish_count_),
        static_cast<unsigned long long>(invalid_joint_value_count_));
    }
  }

  void publish_servo_commands(const std::array<double, kJointCount> & cmd_snapshot) {
    auto stamp = this->get_clock()->now();
    for (std::size_t idx = 0; idx < kJointCount; ++idx) {
      const auto servo_id = joint_to_servo_id_[idx];
      if (servo_id == kIgnoreServoId) {
        continue;
      }

      const double rad = cmd_snapshot[idx];
      if (!std::isfinite(rad)) {
        ++invalid_joint_value_count_;
        continue;
      }

      const double target_us = center_pulse_us_ + rad * us_per_rad_ + servo_offset(servo_id);
      const double clamped_us = std::clamp(target_us, min_pulse_us_, max_pulse_us_);

      servo_msgs::msg::ServoCommand cmd_msg;
      cmd_msg.servo_type = servo_type_;
      cmd_msg.servo_id = static_cast<uint16_t>(servo_id);
      cmd_msg.position = static_cast<uint16_t>(std::lround(clamped_us));
      cmd_msg.speed = static_cast<uint16_t>(
        std::clamp<int64_t>(speed_, 0, std::numeric_limits<uint16_t>::max()));
      cmd_msg.stamp = stamp;
      servo_cmd_pub_->publish(cmd_msg);
      ++command_publish_count_;
    }
  }

  void publish_joint_feedback(const std::array<double, kJointCount> & state_snapshot) {
    std_msgs::msg::Float32MultiArray fb_msg;
    fb_msg.data.resize(kJointCount, 0.0F);
    for (std::size_t idx = 0; idx < kJointCount; ++idx) {
      fb_msg.data[idx] = static_cast<float>(state_snapshot[idx]);
    }
    joint_state_fb_pub_->publish(fb_msg);
    ++state_publish_count_;
  }

  std::string joint_cmd_topic_;
  std::string joint_state_fb_topic_;
  std::string servo_cmd_topic_;
  std::string servo_state_topic_;
  std::string servo_type_;

  double publish_rate_hz_{50.0};
  double min_pulse_us_{500.0};
  double max_pulse_us_{2500.0};
  double center_pulse_us_{1500.0};
  double us_per_rad_{636.6197723675814};
  int64_t speed_{100};
  bool debug_{false};

  std::vector<int64_t> joint_to_servo_id_;
  std::unordered_map<int64_t, std::size_t> servo_to_joint_idx_;
  std::unordered_map<int64_t, double> servo_offset_us_;

  std::array<double, kJointCount> latest_joint_cmd_rad_;
  std::array<double, kJointCount> latest_joint_state_rad_;
  bool has_joint_cmd_{false};
  bool has_servo_state_{false};

  uint64_t command_publish_count_{0};
  uint64_t state_publish_count_{0};
  uint64_t invalid_joint_value_count_{0};

  std::mutex data_mutex_;

  rclcpp::Publisher<servo_msgs::msg::ServoCommand>::SharedPtr servo_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr joint_state_fb_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<servo_msgs::msg::ServoState>::SharedPtr servo_state_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimServoBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
