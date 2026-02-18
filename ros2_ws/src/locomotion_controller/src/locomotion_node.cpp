#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "go2_msgs/msg/mpc_forces.hpp"
#include "go2_msgs/msg/q_dq.hpp"
#include "std_msgs/msg/bool.hpp"
#include "unitree_go/msg/low_cmd.hpp"

#include "locomotion_controller/gait.hpp"
#include "locomotion_controller/leg_controller.hpp"
#include "locomotion_controller/pinocchio_model.hpp"

namespace locomotion_mpc {

namespace {
constexpr uint8_t kHead0 = 0xFE;
constexpr uint8_t kHead1 = 0xEF;
constexpr uint8_t kLowLevel = 0xFF;
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

class LocomotionNode : public rclcpp::Node {
public:
  LocomotionNode()
  : rclcpp::Node("locomotion_controller")
  {
    const std::string go2_share = ament_index_cpp::get_package_share_directory("go2_description");
    std::string default_urdf = go2_share + "/model/go2.urdf";
    std::string default_pkg_dir = go2_share;

    std::string urdf_path = declare_parameter<std::string>("urdf_path", default_urdf);
    std::string pkg_dir = declare_parameter<std::string>("urdf_package_dir", default_pkg_dir);

    gait_hz_ = declare_parameter<double>("gait_hz", 3.0);
    gait_duty_ = declare_parameter<double>("gait_duty", 0.6);
    ctrl_hz_ = declare_parameter<double>("ctrl_hz", 250.0);

    x_vel_des_body_ = declare_parameter<double>("x_vel_des_body", 0.0);
    y_vel_des_body_ = declare_parameter<double>("y_vel_des_body", 0.0);
    z_pos_des_body_ = declare_parameter<double>("z_pos_des_body", 0.27);
    yaw_rate_des_body_ = declare_parameter<double>("yaw_rate_des_body", 0.0);
    ground_offset_ = declare_parameter<double>("ground_offset", ground_offset_);
    swing_height_ = declare_parameter<double>("swing_height", swing_height_);
    swing_kp_ = declare_parameter<double>("swing_kp", swing_kp_);
    swing_kd_ = declare_parameter<double>("swing_kd", swing_kd_);
    stance_force_min_ = declare_parameter<double>("stance_force_min", stance_force_min_);
    stance_fallback_force_z_ = declare_parameter<double>(
      "stance_fallback_force_z", stance_fallback_force_z_);
    stance_fallback_force_ = Eigen::Vector3d(0.0, 0.0, stance_fallback_force_z_);

    go2_ = std::make_unique<PinocchioModel>(urdf_path, std::vector<std::string>{pkg_dir});
    gait_ = std::make_unique<Gait>(gait_hz_, gait_duty_, ground_offset_, swing_height_);
    leg_controller_.setSwingGains(swing_kp_, swing_kd_);

    tau_raw_ = Eigen::VectorXd::Zero(12);
    tau_hold_ = Eigen::VectorXd::Zero(12);
    mpc_force_world_ = Eigen::VectorXd::Zero(12);

    const double hip_lim = 23.7;
    const double abd_lim = 23.7;
    const double knee_lim = 45.43;
    const double safety = 0.9;

    tau_lim_ << safety * hip_lim, safety * abd_lim, safety * knee_lim,
      safety * hip_lim, safety * abd_lim, safety * knee_lim,
      safety * hip_lim, safety * abd_lim, safety * knee_lim,
      safety * hip_lim, safety * abd_lim, safety * knee_lim;

    auto qos = rclcpp::SensorDataQoS();
    declare_parameter<std::string>("qdq_topic", "/qdq");
    const auto qdq_topic = get_parameter("qdq_topic").as_string();
    sub_state_ = this->create_subscription<go2_msgs::msg::QDq>(
      qdq_topic, qos, std::bind(&LocomotionNode::onState, this, std::placeholders::_1));
    sub_mpc_ = this->create_subscription<go2_msgs::msg::MpcForces>(
      "/mpc_forces", qos, std::bind(&LocomotionNode::onMpc, this, std::placeholders::_1));
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    sub_mpc_status_ = this->create_subscription<std_msgs::msg::Bool>(
      "/status/mpc/is_running", status_qos,
      std::bind(&LocomotionNode::onMpcStatus, this, std::placeholders::_1));
    sub_inekf_status_ = this->create_subscription<std_msgs::msg::Bool>(
      "/status/inekf/is_running", status_qos,
      std::bind(&LocomotionNode::onInekfStatus, this, std::placeholders::_1));
    sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", qos, std::bind(&LocomotionNode::onCmdVel, this, std::placeholders::_1));

    pub_lowcmd_ = this->create_publisher<unitree_go::msg::LowCmd>("/lowcmd", qos);
    auto status_pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pub_status_ = this->create_publisher<std_msgs::msg::Bool>(
      "/status/loco_ctrl/is_running", status_pub_qos);
    {
      std_msgs::msg::Bool msg;
      msg.data = false;
      pub_status_->publish(msg);
    }

    double ctrl_dt = 1.0 / std::max(1.0, ctrl_hz_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(ctrl_dt)),
      std::bind(&LocomotionNode::onControl, this));

    last_mpc_time_ = this->now();
    status_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        std_msgs::msg::Bool msg;
        msg.data = ctrl_running_;
        pub_status_->publish(msg);
      });

    param_cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params)
      -> rcl_interfaces::msg::SetParametersResult {
        for (const auto & param : params) {
          const auto & name = param.get_name();
          if (name == "swing_kp") {
            swing_kp_ = param.as_double();
          } else if (name == "swing_kd") {
            swing_kd_ = param.as_double();
          } else if (name == "x_vel_des_body") {
            x_vel_des_body_ = param.as_double();
          } else if (name == "y_vel_des_body") {
            y_vel_des_body_ = param.as_double();
          } else if (name == "z_pos_des_body") {
            z_pos_des_body_ = param.as_double();
          } else if (name == "yaw_rate_des_body") {
            yaw_rate_des_body_ = param.as_double();
          } else if (name == "ground_offset") {
            ground_offset_ = param.as_double();
          } else if (name == "swing_height") {
            swing_height_ = param.as_double();
          } else if (name == "stance_force_min") {
            stance_force_min_ = param.as_double();
          } else if (name == "stance_fallback_force_z") {
            stance_fallback_force_z_ = param.as_double();
          }
        }
        leg_controller_.setSwingGains(swing_kp_, swing_kd_);
        if (gait_) {
          gait_->setGroundOffset(ground_offset_);
          gait_->setSwingHeight(swing_height_);
        }
        stance_fallback_force_ = Eigen::Vector3d(0.0, 0.0, stance_fallback_force_z_);
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  }

private:
  void onState(const go2_msgs::msg::QDq::SharedPtr msg)
  {
    if (!sim_time_init_) {
      sim_time_t0_ = msg->sim_time;
      sim_time_init_ = true;
    }
    sim_time_now_ = msg->sim_time;

    Eigen::VectorXd q_pin(19);
    Eigen::VectorXd dq_pin(18);

    q_pin(0) = msg->q[0];
    q_pin(1) = msg->q[1];
    q_pin(2) = msg->q[2];

    double qw = msg->q[3];
    double qx = msg->q[4];
    double qy = msg->q[5];
    double qz = msg->q[6];

    q_pin.segment<4>(3) << qx, qy, qz, qw;
    for (int i = 0; i < 12; ++i) {
      q_pin(7 + i) = msg->q[7 + i];
    }

    Eigen::Vector3d v_world(msg->dq[0], msg->dq[1], msg->dq[2]);
    Eigen::Vector3d w_body(msg->dq[3], msg->dq[4], msg->dq[5]);

    Eigen::Quaterniond q_world(qw, qx, qy, qz);
    Eigen::Matrix3d R = q_world.toRotationMatrix();
    Eigen::Vector3d v_body = R.transpose() * v_world;

    dq_pin.segment<3>(0) = v_body;
    dq_pin.segment<3>(3) = w_body;
    for (int i = 0; i < 12; ++i) {
      dq_pin(6 + i) = msg->dq[6 + i];
    }

    {
      std::lock_guard<std::mutex> lock(go2_mutex_);
      go2_->updateModel(q_pin, dq_pin);
    }

    have_state_ = true;
    last_state_time_ = this->now();
  }

  void onMpc(const go2_msgs::msg::MpcForces::SharedPtr msg)
  {
    Eigen::VectorXd forces = Eigen::VectorXd::Zero(12);
    int count = std::min(static_cast<int>(msg->forces.size()), 12);
    for (int i = 0; i < count; ++i) {
      forces(i) = msg->forces[i];
    }
    {
      std::lock_guard<std::mutex> lock(force_mutex_);
      mpc_force_world_ = forces;
    }
    have_mpc_ = true;
    last_mpc_time_ = this->now();
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    x_vel_des_body_ = msg->linear.x;
    yaw_rate_des_body_ = msg->angular.z;
  }

  void onMpcStatus(const std_msgs::msg::Bool::SharedPtr msg)
  {
    mpc_running_ = msg->data;
  }

  void onInekfStatus(const std_msgs::msg::Bool::SharedPtr msg)
  {
    inekf_running_ = msg->data;
  }

  void onControl()
  {
    if (!have_state_ || !sim_time_init_) {
      tau_hold_.setZero();
      ctrl_running_ = false;
      return;
    }

    const double time_now_s = sim_time_now_ - sim_time_t0_;

    Eigen::VectorXd forces;
    {
      std::lock_guard<std::mutex> lock(force_mutex_);
      forces = mpc_force_world_;
    }
    bool mpc_alive = have_mpc_ && (this->now() - last_mpc_time_).seconds() <= mpc_timeout_s_;
    bool mpc_ready = mpc_running_ && mpc_alive;
    bool est_ready = inekf_running_;
    if (!mpc_ready) {
      if (!mpc_wait_logged_) {
        RCLCPP_INFO(
          get_logger(),
          "Waiting for MPC: running=%s alive=%s",
          mpc_running_ ? "true" : "false",
          mpc_alive ? "true" : "false"
        );
        mpc_wait_logged_ = true;
      }
    } else {
      mpc_wait_logged_ = false;
    }
    if (!est_ready) {
      if (!est_wait_logged_) {
        RCLCPP_INFO(
          get_logger(),
          "Waiting for state estimator (/status/inekf/is_running)"
        );
        est_wait_logged_ = true;
      }
    } else {
      est_wait_logged_ = false;
    }
    if (mpc_ready && gait_) {
      const Eigen::Vector4i mask = gait_->computeCurrentMask(time_now_s);
      for (int i = 0; i < 4; ++i) {
        if (mask(i) == 1) {
          const Eigen::Vector3d f_leg = forces.segment<3>(i * 3);
          if (f_leg.norm() < stance_force_min_) {
            forces.segment<3>(i * 3) = stance_fallback_force_;
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(go2_mutex_);
      Eigen::Vector3d vel_des_body(x_vel_des_body_, y_vel_des_body_, 0.0);
      Eigen::Vector3d vel_des_world = go2_->Rz() * vel_des_body;
      go2_->setXVelDesWorld(vel_des_world.x());
      go2_->setYVelDesWorld(vel_des_world.y());
      go2_->setYawRateDesWorld(yaw_rate_des_body_);

      if (!pos_des_init_) {
        pos_des_world_ = go2_->posComWorld();
        pos_des_init_ = true;
      }

      const double max_pos_error = 0.1;
      pos_des_world_.x() = std::min(
        std::max(pos_des_world_.x(), go2_->posComWorld().x() - max_pos_error),
        go2_->posComWorld().x() + max_pos_error);
      pos_des_world_.y() = std::min(
        std::max(pos_des_world_.y(), go2_->posComWorld().y() - max_pos_error),
        go2_->posComWorld().y() + max_pos_error);
      pos_des_world_.z() = z_pos_des_body_;

      go2_->setXPosDesWorld(pos_des_world_.x());
      go2_->setYPosDesWorld(pos_des_world_.y());

      tau_raw_ = leg_controller_.computeAllLegTorques(
        *go2_, *gait_, forces, time_now_s);
    }

    tau_hold_ = tau_raw_.cwiseMin(tau_lim_).cwiseMax(-tau_lim_);

    bool forces_nonzero = forces.cwiseAbs().maxCoeff() > force_eps_;

    if ((mpc_ready && est_ready) || forces_nonzero) {
      publishLowCmd();
      ctrl_running_ = true;
    } else {
      ctrl_running_ = false;
    }
  }

  void publishLowCmd()
  {
    unitree_go::msg::LowCmd msg;
    msg.head[0] = kHead0;
    msg.head[1] = kHead1;
    msg.level_flag = kLowLevel;
    msg.gpio = 0;

    for (int i = 0; i < 20; ++i) {
      msg.motor_cmd[i].mode = 0x01;
      msg.motor_cmd[i].q = static_cast<float>(kPosStopF);
      msg.motor_cmd[i].dq = static_cast<float>(kVelStopF);
      msg.motor_cmd[i].kp = 0.0f;
      msg.motor_cmd[i].kd = 0.0f;
      msg.motor_cmd[i].tau = 0.0f;
    }
    for (int i = 0; i < 12; ++i) {
      int src = mujoco_to_unitree_[i];
      msg.motor_cmd[i].tau = static_cast<float>(tau_hold_(src));
    }
    get_crc(msg);
    pub_lowcmd_->publish(msg);
  }

  double gait_hz_{3.0};
  double gait_duty_{0.6};
  double ctrl_hz_{250.0};

  double x_vel_des_body_{0.0};
  double y_vel_des_body_{0.0};
  double z_pos_des_body_{0.27};
  double yaw_rate_des_body_{0.0};
  double swing_kp_{400.0};
  double swing_kd_{75.0};
  double ground_offset_{0.02};
  double swing_height_{0.1};
  double stance_force_min_{1.0};
  double stance_fallback_force_z_{0.0};
  Eigen::Vector3d stance_fallback_force_{Eigen::Vector3d::Zero()};

  std::unique_ptr<PinocchioModel> go2_;
  std::unique_ptr<Gait> gait_;
  LegController leg_controller_;

  Eigen::VectorXd tau_raw_{Eigen::VectorXd::Zero(12)};
  Eigen::VectorXd tau_hold_{Eigen::VectorXd::Zero(12)};
  Eigen::VectorXd tau_lim_{Eigen::VectorXd::Zero(12)};
  Eigen::VectorXd mpc_force_world_{Eigen::VectorXd::Zero(12)};

  std::mutex go2_mutex_;
  std::mutex force_mutex_;

  bool have_state_{false};
  bool have_mpc_{false};
  bool mpc_running_{false};
  bool inekf_running_{false};
  bool ctrl_running_{false};
  bool mpc_wait_logged_{false};
  bool est_wait_logged_{false};
  rclcpp::Time last_state_time_{};
  rclcpp::Time last_mpc_time_{};

  double sim_time_now_{0.0};
  double sim_time_t0_{0.0};
  bool sim_time_init_{false};

  Eigen::Vector3d pos_des_world_{Eigen::Vector3d::Zero()};
  bool pos_des_init_{false};

  const double mpc_timeout_s_{0.5};
  const double force_eps_{1e-6};

  const std::array<int, 12> mujoco_to_unitree_{
    {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8}};

  rclcpp::Subscription<go2_msgs::msg::QDq>::SharedPtr sub_state_;
  rclcpp::Subscription<go2_msgs::msg::MpcForces>::SharedPtr sub_mpc_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mpc_status_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_inekf_status_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr pub_lowcmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace locomotion_mpc

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<locomotion_mpc::LocomotionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
