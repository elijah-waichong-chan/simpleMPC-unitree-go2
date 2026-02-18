#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/low_state.hpp"

using namespace std::chrono_literals;

namespace {
constexpr uint8_t kLowLevel = 0xFF;
constexpr uint8_t kHead0 = 0xFE;
constexpr uint8_t kHead1 = 0xEF;
constexpr double kPosStopF = 2.146E+9;
constexpr double kVelStopF = 16000.0;

struct MotorCmdRaw {
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  std::array<uint32_t, 3> reserve;
};

struct BmsCmdRaw {
  uint8_t off;
  std::array<uint8_t, 3> reserve;
};

struct LowCmdRaw {
  std::array<uint8_t, 2> head;
  uint8_t level_flag;
  uint8_t frame_reserve;
  std::array<uint32_t, 2> sn;
  std::array<uint32_t, 2> version;
  uint16_t bandwidth;
  std::array<MotorCmdRaw, 20> motor_cmd;
  BmsCmdRaw bms;
  std::array<uint8_t, 40> wireless_remote;
  std::array<uint8_t, 12> led;
  std::array<uint8_t, 2> fan;
  uint8_t gpio;
  uint32_t reserve;
  uint32_t crc;
};

uint32_t crc32_core(uint32_t * ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  const uint32_t poly = 0x04c11db7;
  for (uint32_t i = 0; i < len; ++i) {
    xbit = 1u << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (crc32 & 0x80000000) {
        crc32 = (crc32 << 1) ^ poly;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) crc32 ^= poly;
      xbit >>= 1;
    }
  }
  return crc32;
}

void get_crc(unitree_go::msg::LowCmd & msg) {
  LowCmdRaw raw{};
  std::memcpy(&raw.head[0], &msg.head[0], 2);
  raw.level_flag = msg.level_flag;
  raw.frame_reserve = msg.frame_reserve;
  std::memcpy(&raw.sn[0], &msg.sn[0], 8);
  std::memcpy(&raw.version[0], &msg.version[0], 8);
  raw.bandwidth = msg.bandwidth;

  for (int i = 0; i < 20; ++i) {
    raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
    raw.motor_cmd[i].q = msg.motor_cmd[i].q;
    raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
    raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
    raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
    raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
    std::memcpy(&raw.motor_cmd[i].reserve[0], &msg.motor_cmd[i].reserve[0], 12);
  }

  raw.bms.off = msg.bms_cmd.off;
  std::memcpy(&raw.bms.reserve[0], &msg.bms_cmd.reserve[0], 3);
  std::memcpy(&raw.wireless_remote[0], &msg.wireless_remote[0], 40);
  std::memcpy(&raw.led[0], &msg.led[0], 12);
  std::memcpy(&raw.fan[0], &msg.fan[0], 2);
  raw.gpio = msg.gpio;
  raw.reserve = msg.reserve;

  raw.crc = crc32_core(reinterpret_cast<uint32_t*>(&raw), (sizeof(LowCmdRaw) >> 2) - 1);
  msg.crc = raw.crc;
}
}  // namespace

class StandUpInitNode : public rclcpp::Node {
 public:
  StandUpInitNode() : Node("stand_up_init") {
    declare_parameter<double>("kp", 60.0);
    declare_parameter<double>("kd", 5.0);
    declare_parameter<double>("crouch_time_s", 1.0);
    declare_parameter<double>("crouch_hold_s", 0.2);
    declare_parameter<double>("ramp_time_s", 2.0);
    declare_parameter<double>("start_delay_s", 0.5);
    declare_parameter<double>("command_hz", 500.0);
    // Unitree order: FR, FL, RR, RL (3 joints each)
    declare_parameter<std::vector<double>>(
      "crouch_pos",
      {0.0, 1.36, -2.65,
       0.0, 1.36, -2.65,
       0.0, 1.36, -2.65,
       0.0, 1.36, -2.65});
    // Unitree order: FR, FL, RR, RL (3 joints each)
    declare_parameter<std::vector<double>>(
      "target_pos",
      {0.0, 0.9, -1.8,
       0.0, 0.9, -1.8,
       0.0, 0.9, -1.8,
       0.0, 0.9, -1.8});

    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    crouch_time_s_ = std::max(0.01, get_parameter("crouch_time_s").as_double());
    crouch_hold_s_ = std::max(0.0, get_parameter("crouch_hold_s").as_double());
    ramp_time_s_ = std::max(0.01, get_parameter("ramp_time_s").as_double());
    start_delay_s_ = std::max(0.0, get_parameter("start_delay_s").as_double());
    command_hz_ = std::max(1.0, get_parameter("command_hz").as_double());

    crouch_pos_ = get_parameter("crouch_pos").as_double_array();
    if (crouch_pos_.size() != 12) {
      RCLCPP_WARN(get_logger(),
        "crouch_pos must have 12 elements (FR,FL,RR,RL). Using defaults.");
      crouch_pos_ = {0.0, 1.36, -2.65,
                     0.0, 1.36, -2.65,
                     0.0, 1.36, -2.65,
                     0.0, 1.36, -2.65};
    }

    target_pos_ = get_parameter("target_pos").as_double_array();
    if (target_pos_.size() != 12) {
      RCLCPP_WARN(get_logger(),
        "target_pos must have 12 elements (FR,FL,RR,RL). Using defaults.");
      target_pos_ = {0.0, 0.67, -1.3,
                     0.0, 0.67, -1.3,
                     0.0, 0.67, -1.3,
                     0.0, 0.67, -1.3};
    }

    auto qos = rclcpp::QoS(10);
    lowcmd_pub_ = create_publisher<unitree_go::msg::LowCmd>("/lowcmd", qos);
    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
      "/lowstate", qos,
      std::bind(&StandUpInitNode::on_lowstate, this, std::placeholders::_1));
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    status_pub_ = create_publisher<std_msgs::msg::Bool>("/status/standing_init", status_qos);
    {
      std_msgs::msg::Bool msg;
      msg.data = false;
      status_pub_->publish(msg);
    }
    ctrl_status_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/status/loco_ctrl/is_running", status_qos,
      std::bind(&StandUpInitNode::on_ctrl_status, this, std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0 / command_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&StandUpInitNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "stand_up_init running");
  }

 private:
  void on_lowstate(const unitree_go::msg::LowState::SharedPtr msg) {
    last_state_ = *msg;
    have_state_ = true;
  }

  void on_timer() {
    if (!have_state_) return;

    if (!started_) {
      for (int i = 0; i < 12; ++i) {
        start_pos_[i] = last_state_.motor_state[i].q;
      }
      start_time_ = now();
      started_ = true;
    }

    const double t = (now() - start_time_).seconds() - start_delay_s_;
    if (t < 0.0) {
      return;  // wait for delay, do not publish yet
    }
    const double t_crouch_end = crouch_time_s_;
    const double t_hold_end = t_crouch_end + crouch_hold_s_;
    const double t_stand_end = t_hold_end + ramp_time_s_;

    if (t >= t_stand_end && !status_sent_) {
      std_msgs::msg::Bool msg;
      msg.data = true;
      status_pub_->publish(msg);
      status_sent_ = true;
    }

    unitree_go::msg::LowCmd cmd;
    cmd.head[0] = kHead0;
    cmd.head[1] = kHead1;
    cmd.level_flag = kLowLevel;
    cmd.gpio = 0;

    for (int i = 0; i < 20; ++i) {
      cmd.motor_cmd[i].mode = 0x01;
      cmd.motor_cmd[i].q = static_cast<float>(kPosStopF);
      cmd.motor_cmd[i].dq = static_cast<float>(kVelStopF);
      cmd.motor_cmd[i].kp = 0.0f;
      cmd.motor_cmd[i].kd = 0.0f;
      cmd.motor_cmd[i].tau = 0.0f;
    }

    for (int i = 0; i < 12; ++i) {
      double q_des = start_pos_[i];
      if (t < t_crouch_end) {
        const double a = std::clamp(t / crouch_time_s_, 0.0, 1.0);
        q_des = (1.0 - a) * start_pos_[i] + a * crouch_pos_[i];
      } else if (t < t_hold_end) {
        q_des = crouch_pos_[i];
      } else if (t < t_stand_end) {
        const double a = std::clamp((t - t_hold_end) / ramp_time_s_, 0.0, 1.0);
        q_des = (1.0 - a) * crouch_pos_[i] + a * target_pos_[i];
      } else {
        q_des = target_pos_[i];
      }

      cmd.motor_cmd[i].q = static_cast<float>(q_des);
      cmd.motor_cmd[i].dq = 0.0f;
      cmd.motor_cmd[i].kp = static_cast<float>(kp_);
      cmd.motor_cmd[i].kd = static_cast<float>(kd_);
      cmd.motor_cmd[i].tau = 0.0f;
      cmd.motor_cmd[i].mode = 0x01;  // servo mode
    }

    get_crc(cmd);
    lowcmd_pub_->publish(cmd);
  }

  void on_ctrl_status(const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg->data || ctrl_running_) {
      return;
    }
    ctrl_running_ = true;
    RCLCPP_INFO(get_logger(), "locomotion_controller is running; stopping stand_up_init.");
    if (timer_) {
      timer_->cancel();
    }
    rclcpp::shutdown();
  }

  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr lowcmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_status_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  unitree_go::msg::LowState last_state_{};
  bool have_state_{false};
  bool started_{false};
  rclcpp::Time start_time_{};

  double kp_{60.0};
  double kd_{5.0};
  double crouch_time_s_{1.0};
  double crouch_hold_s_{0.2};
  double ramp_time_s_{2.0};
  double start_delay_s_{0.5};
  double command_hz_{500.0};

  std::vector<double> crouch_pos_;
  std::vector<double> target_pos_;
  double start_pos_[12]{};
  bool status_sent_{false};
  bool ctrl_running_{false};
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StandUpInitNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
