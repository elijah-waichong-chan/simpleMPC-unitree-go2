#include <rclcpp/rclcpp.hpp>

#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/low_cmd.hpp>
#include <go2_msgs/msg/q_dq.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <algorithm>

using namespace std::chrono_literals;

namespace {
inline void pub_f64(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & pub, double x)
{
  if (!pub) return;
  std_msgs::msg::Float64 m;
  m.data = x;
  pub->publish(m);
}

constexpr std::array<const char *, 4> kLegs = {"FR", "FL", "RR", "RL"};
constexpr int kMujocoToUnitree[12] = {3,4,5, 0,1,2, 9,10,11, 6,7,8};
}  // namespace

class MuJoCoRobot : public rclcpp::Node
{
public:
  MuJoCoRobot()
  : Node("mujoco_robot_node")
  {
    // Rates
    sim_hz_ = 500.0;
    pub_hz_ = 250.0;

    // Parameters
    declare_parameter<std::string>("xml_path", "/home/elijah/go2-convex-mpc/models/MJCF/go2/scene.xml");
    declare_parameter<bool>("enable_viewer", true);
    declare_parameter<bool>("freeze_base", false);
    declare_parameter<bool>("debug_print", true);
    declare_parameter<bool>("debug_publish", true);
    declare_parameter<double>("foot_force_lpf_alpha", 0.1);
    declare_parameter<double>("imu_gyro_noise_std", 0.0);
    declare_parameter<double>("imu_acc_noise_std", 0.0);
    declare_parameter<int>("imu_noise_seed", 0);
    declare_parameter<double>("render_hz", 30.0);
    declare_parameter<double>("sim_hz", sim_hz_);
    declare_parameter<double>("pub_hz", pub_hz_);
    xml_path_ = get_parameter("xml_path").as_string();
    enable_viewer_ = get_parameter("enable_viewer").as_bool();
    freeze_base_ = get_parameter("freeze_base").as_bool();
    debug_print_ = get_parameter("debug_print").as_bool();
    debug_publish_ = get_parameter("debug_publish").as_bool();
    foot_force_lpf_alpha_ = get_parameter("foot_force_lpf_alpha").as_double();
    imu_gyro_noise_std_ = get_parameter("imu_gyro_noise_std").as_double();
    imu_acc_noise_std_ = get_parameter("imu_acc_noise_std").as_double();
    imu_noise_seed_ = get_parameter("imu_noise_seed").as_int();
    render_hz_ = get_parameter("render_hz").as_double();
    sim_hz_ = get_parameter("sim_hz").as_double();
    pub_hz_ = get_parameter("pub_hz").as_double();
    if (sim_hz_ <= 0.0 || pub_hz_ <= 0.0) {
      throw std::runtime_error("sim_hz and pub_hz must be > 0.");
    }
    if (std::fmod(sim_hz_, pub_hz_) != 0.0) {
      throw std::runtime_error("sim_hz must be divisible by pub_hz.");
    }
    pub_decim_ = static_cast<int>(sim_hz_ / pub_hz_);
    sim_dt_ = 1.0 / sim_hz_;
    if (render_hz_ < 0.0) {
      render_hz_ = 0.0;
    }
    if (imu_noise_seed_ == 0) {
      std::random_device rd;
      rng_.seed(rd());
    } else {
      rng_.seed(static_cast<uint32_t>(imu_noise_seed_));
    }

    // Load MuJoCo
    char err[1024];
    std::memset(err, 0, sizeof(err));

    model_ = mj_loadXML(xml_path_.c_str(), nullptr, err, sizeof(err));
    if (!model_) {
      throw std::runtime_error(std::string("Failed to load MJCF: ") + err);
    }

    data_ = mj_makeData(model_);
    if (!data_) {
      mj_deleteModel(model_);
      model_ = nullptr;
      throw std::runtime_error("Failed to allocate mjData.");
    }
    
    render_data_ = mj_makeData(model_);
    if (!render_data_) {
      mj_deleteData(data_);
      mj_deleteModel(model_);
      data_ = nullptr;
      model_ = nullptr;
      throw std::runtime_error("Failed to allocate render mjData.");
    }

    model_->opt.timestep = sim_dt_;

    // Cache sensors
    imu_quat_ = cache_sensor_slice_("imu_quat");
    imu_gyro_ = cache_sensor_slice_("imu_gyro");
    imu_acc_  = cache_sensor_slice_("imu_acc");

    // Optional touch sensors
    fl_touch_ = cache_sensor_slice_("FL_touch");
    fr_touch_ = cache_sensor_slice_("FR_touch");
    rl_touch_ = cache_sensor_slice_("RL_touch");
    rr_touch_ = cache_sensor_slice_("RR_touch");

    for (const char * leg : kLegs) {
      foot_geoms_[leg] = mj_name2id(model_, mjOBJ_GEOM, leg);
    }
    for (const auto & kv : foot_geoms_) {
      if (kv.second < 0) {
        throw std::runtime_error("Foot geom '" + kv.first + "' not found in MJCF.");
      }
    }

    for (const char * leg : kLegs) {
      cache_actuator_ids_(leg);
    }

    const double base_pos[3] = {0.0, 0.0, 0.108};
    const double base_quat_wxyz[4] = {1.0, 0.0, 0.0, 0.0};
    const double leg[3] = {0.0, 1.13, -2.73};


    // const double base_pos[3] = {0.0, 0.0, 0.27};
    // const double base_quat_wxyz[4] = {1.0, 0.0, 0.0, 0.0};
    // const double leg[3] = {0.0, 0.9, -1.8};

    {
      std::lock_guard<std::mutex> lk(mj_mtx_);

      for (int i = 0; i < 3; ++i) data_->qpos[i] = base_pos[i];
      for (int i = 0; i < 4; ++i) data_->qpos[3 + i] = base_quat_wxyz[i];

      int idx = 7;
      // Order: FL, FR, RL, RR
      for (int leg_i = 0; leg_i < 4; ++leg_i) {
        for (int j = 0; j < 3; ++j) data_->qpos[idx++] = leg[j];
      }

      mj_forward(model_, data_);
    }

    // ROS pub/sub
    pub_lowstate_ = create_publisher<unitree_go::msg::LowState>("/lowstate", 10);
    pub_qdq_      = create_publisher<go2_msgs::msg::QDq>("/qdq", rclcpp::SensorDataQoS());
    // Volatile QoS avoids stale latched "true" when the node shuts down.
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    pub_status_ = create_publisher<std_msgs::msg::Bool>("/status/mujoco/is_running", status_qos);
    status_timer_ = create_wall_timer(
      100ms,
      [this]() {
        std_msgs::msg::Bool msg;
        msg.data = true;
        pub_status_->publish(msg);
      }
    );

    rclcpp::on_shutdown([weak_pub = std::weak_ptr<rclcpp::Publisher<std_msgs::msg::Bool>>(pub_status_)]() {
      if (auto pub = weak_pub.lock()) {
        std_msgs::msg::Bool msg;
        msg.data = false;
        pub->publish(msg);
      }
    });

    if (debug_publish_) {
      auto tqos = rclcpp::SensorDataQoS();
      pub_outer_loop_ms_      = create_publisher<std_msgs::msg::Float64>("/timing/outer_loop_ms", tqos);
      pub_physics_step_hz_    = create_publisher<std_msgs::msg::Float64>("/timing/physics_step_hz", tqos);
      pub_physics_step_ms_    = create_publisher<std_msgs::msg::Float64>("/timing/physics_step_ms", tqos);
      pub_steps_per_loop_     = create_publisher<std_msgs::msg::Float64>("/timing/steps_per_outer_loop", tqos);
      pub_rtf_                = create_publisher<std_msgs::msg::Float64>("/timing/rtf", tqos);
      pub_storing_cmd_ms_     = create_publisher<std_msgs::msg::Float64>("/timing/storing_cmd_ms", tqos);
      pub_heartbeat_          = create_publisher<std_msgs::msg::Float64>("/timing/heartbeat", tqos);
      heartbeat_timer_ = create_wall_timer(
        500ms,
        [this]() { pub_f64(pub_heartbeat_, this->now().seconds()); }
      );
    }

    sub_cmd_ = create_subscription<unitree_go::msg::LowCmd>(
      "/lowcmd", rclcpp::SensorDataQoS(),
      std::bind(&MuJoCoRobot::store_cmd_, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Loaded MuJoCo XML: %s", xml_path_.c_str());
    {
      std::lock_guard<std::mutex> lk(mj_mtx_);
      for (int i = 0; i < 7; ++i) base_qpos_[i] = data_->qpos[i];
    }

    if (debug_print_) {
      RCLCPP_INFO(get_logger(),
                  "sim_hz=%.1f pub_hz=%.1f pub_decim=%d sim_dt=%.6f viewer=%s render_hz=%.1f freeze_base=%s",
                  sim_hz_, pub_hz_, pub_decim_, sim_dt_, enable_viewer_ ? "true" : "false",
                  render_hz_, freeze_base_ ? "true" : "false");
    }
  }

  ~MuJoCoRobot() override
  {
    stop_viewer();

    if (data_)  mj_deleteData(data_);
    if (render_data_) mj_deleteData(render_data_);
    if (model_) mj_deleteModel(model_);
  }

  // Getters
  double sim_hz() const { return sim_hz_; }
  double sim_dt() const { return sim_dt_; }
  int pub_decim() const { return pub_decim_; }

  double sim_time() const
  {
    std::lock_guard<std::mutex> lk(mj_mtx_);
    return static_cast<double>(data_->time);
  }

  bool debug_print() const { return debug_print_; }
  bool debug_publish() const { return debug_publish_; }

  uint64_t lowcmd_count() const
  {
    return lowcmd_count_.load(std::memory_order_relaxed);
  }

  // Viewer control
  void start_viewer()
  {
    if (!enable_viewer_) return;
    if (viewer_thread_.joinable()) return;

    viewer_should_exit_.store(false);
    viewer_thread_ = std::thread([this]() { this->viewer_loop_(); });
  }

  void stop_viewer()
  {
    viewer_should_exit_.store(true);
    if (viewer_thread_.joinable()) viewer_thread_.join();
  }

  // Sim step
  void sim_step()
  {
    std::lock_guard<std::mutex> lk(mj_mtx_);

    if (freeze_base_) {
      // Force base pose/velocity before stepping.
      for (int i = 0; i < 7; ++i) data_->qpos[i] = base_qpos_[i];
      for (int i = 0; i < 6; ++i) data_->qvel[i] = 0.0;
    }

    mj_step1(model_, data_);

    std::array<double, 12> q_unitree{};
    std::array<double, 12> dq_unitree{};
    for (int i = 0; i < 12; ++i) {
      const int j = kMujocoToUnitree[i];
      q_unitree[i] = static_cast<double>(data_->qpos[7 + j]);
      dq_unitree[i] = static_cast<double>(data_->qvel[6 + j]);
    }

    // Snapshot latest command
    std::array<double, 12> cmd_tau{};
    std::array<double, 12> cmd_q{};
    std::array<double, 12> cmd_dq{};
    std::array<double, 12> cmd_kp{};
    std::array<double, 12> cmd_kd{};
    std::array<bool, 12> cmd_pd_enable{};
    {
      std::lock_guard<std::mutex> clk(cmd_mtx_);
      cmd_tau = cmd_tau_;
      cmd_q = cmd_q_;
      cmd_dq = cmd_dq_;
      cmd_kp = cmd_kp_;
      cmd_kd = cmd_kd_;
      cmd_pd_enable = cmd_pd_enable_;
    }

    // Build torque command: tau has priority; otherwise PD if enabled
    std::array<double, 12> tau{};
    for (int i = 0; i < 12; ++i) {
      const double tau_in = cmd_tau[i];
      if (std::fabs(tau_in) > 1e-9) {
        tau[i] = tau_in;
      } else if (cmd_pd_enable[i]) {
        const double q_err = cmd_q[i] - q_unitree[i];
        const double dq_err = cmd_dq[i] - dq_unitree[i];
        tau[i] = cmd_kp[i] * q_err + cmd_kd[i] * dq_err;
      } else {
        tau[i] = 0.0;
      }
    }

    last_tau_ = tau;
    set_joint_torque_(tau);

    mj_step2(model_, data_);

    if (freeze_base_) {
      // Re-freeze base after stepping to keep reported state fixed.
      for (int i = 0; i < 7; ++i) data_->qpos[i] = base_qpos_[i];
      for (int i = 0; i < 6; ++i) data_->qvel[i] = 0.0;
    }
  }

  // Publish state
  void publish_state()
  {
    std::lock_guard<std::mutex> lk(mj_mtx_);

    if (freeze_base_) {
      for (int i = 0; i < 7; ++i) data_->qpos[i] = base_qpos_[i];
      for (int i = 0; i < 6; ++i) data_->qvel[i] = 0.0;
      mj_forward(model_, data_);
    }

    unitree_go::msg::LowState msg;

    const mjtNum * q = data_->qpos;
    const mjtNum * v = data_->qvel;

    for (int i = 0; i < 12; ++i) {
      const int j = kMujocoToUnitree[i];
      msg.motor_state[i].q  = static_cast<double>(q[7 + j]);
      msg.motor_state[i].dq = static_cast<double>(v[6 + j]);
      msg.motor_state[i].tau_est = static_cast<double>(last_tau_[i]);
    }

    const auto quat_wxyz = sensor_ptr_(imu_quat_);  // [w,x,y,z]
    const auto gyro_xyz  = sensor_ptr_(imu_gyro_);  // [wx,wy,wz]
    const auto acc_xyz   = sensor_ptr_(imu_acc_);   // [ax,ay,az]

    msg.imu_state.quaternion[0] = static_cast<double>(quat_wxyz[0]);
    msg.imu_state.quaternion[1] = static_cast<double>(quat_wxyz[1]);
    msg.imu_state.quaternion[2] = static_cast<double>(quat_wxyz[2]);
    msg.imu_state.quaternion[3] = static_cast<double>(quat_wxyz[3]);

    msg.imu_state.gyroscope[0] = static_cast<double>(gyro_xyz[0]);
    msg.imu_state.gyroscope[1] = static_cast<double>(gyro_xyz[1]);
    msg.imu_state.gyroscope[2] = static_cast<double>(gyro_xyz[2]);
    if (imu_gyro_noise_std_ > 0.0) {
      msg.imu_state.gyroscope[0] += sample_noise_(imu_gyro_noise_std_);
      msg.imu_state.gyroscope[1] += sample_noise_(imu_gyro_noise_std_);
      msg.imu_state.gyroscope[2] += sample_noise_(imu_gyro_noise_std_);
    }

    msg.imu_state.accelerometer[0] = static_cast<double>(acc_xyz[0]);
    msg.imu_state.accelerometer[1] = static_cast<double>(acc_xyz[1]);
    msg.imu_state.accelerometer[2] = static_cast<double>(acc_xyz[2]);
    if (imu_acc_noise_std_ > 0.0) {
      msg.imu_state.accelerometer[0] += sample_noise_(imu_acc_noise_std_);
      msg.imu_state.accelerometer[1] += sample_noise_(imu_acc_noise_std_);
      msg.imu_state.accelerometer[2] += sample_noise_(imu_acc_noise_std_);
    }

    bool contact_fr = false, contact_fl = false, contact_rr = false, contact_rl = false;
    const int gid_fr = foot_geoms_.at("FR");
    const int gid_fl = foot_geoms_.at("FL");
    const int gid_rr = foot_geoms_.at("RR");
    const int gid_rl = foot_geoms_.at("RL");
    const int gid_floor = mj_name2id(model_, mjOBJ_GEOM, "floor");
    if (gid_floor < 0) {
      static bool warned_missing_floor = false;
      if (!warned_missing_floor) {
        RCLCPP_WARN(get_logger(),
                    "Geom 'floor' not found; counting any foot contact.");
        warned_missing_floor = true;
      }
    }

    for (int i = 0; i < data_->ncon; ++i) {
      const mjContact & c = data_->contact[i];
      auto is_contact = [&](int foot_gid) {
        if (gid_floor < 0) {
          return (c.geom1 == foot_gid || c.geom2 == foot_gid);
        }
        return (c.geom1 == foot_gid && c.geom2 == gid_floor) ||
               (c.geom2 == foot_gid && c.geom1 == gid_floor);
      };
      if (is_contact(gid_fr)) contact_fr = true;
      if (is_contact(gid_fl)) contact_fl = true;
      if (is_contact(gid_rr)) contact_rr = true;
      if (is_contact(gid_rl)) contact_rl = true;
    }

    const float raw_forces[4] = {
      contact_fr ? 30.0f : 0.0f,
      contact_fl ? 30.0f : 0.0f,
      contact_rr ? 30.0f : 0.0f,
      contact_rl ? 30.0f : 0.0f
    };

    double alpha = std::clamp(foot_force_lpf_alpha_, 0.0, 1.0);
    if (!foot_force_lpf_init_ || alpha <= 0.0) {
      for (int i = 0; i < 4; ++i) foot_force_lpf_[i] = raw_forces[i];
      foot_force_lpf_init_ = true;
    } else {
      for (int i = 0; i < 4; ++i) {
        foot_force_lpf_[i] =
          static_cast<float>((1.0 - alpha) * foot_force_lpf_[i] + alpha * raw_forces[i]);
      }
    }

    msg.foot_force[0] = foot_force_lpf_[0];
    msg.foot_force[1] = foot_force_lpf_[1];
    msg.foot_force[2] = foot_force_lpf_[2];
    msg.foot_force[3] = foot_force_lpf_[3];
    msg.foot_force_est = msg.foot_force;

    pub_lowstate_->publish(msg);

    go2_msgs::msg::QDq qdq;
    qdq.sim_time = static_cast<double>(data_->time);

    const int nq_msg = static_cast<int>(qdq.q.size());   // 19
    const int nv_msg = static_cast<int>(qdq.dq.size());  // 18
    const int nq = std::min(nq_msg, model_->nq);
    const int nv = std::min(nv_msg, model_->nv);

    for (int i = 0; i < nq; ++i) qdq.q[i]  = static_cast<double>(data_->qpos[i]);
    for (int i = 0; i < nv; ++i) qdq.dq[i] = static_cast<double>(data_->qvel[i]);

    if (model_->nq != nq_msg || model_->nv != nv_msg) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "QDq expects q[%d], dq[%d], but MuJoCo has nq=%d nv=%d (copying min).",
        nq_msg, nv_msg, model_->nq, model_->nv);
    }

    pub_qdq_->publish(qdq);
  }

  // Telemetry
  void publish_timing_window(double outer_loop_ms_avg,
                             double physics_step_hz,
                             double physics_step_ms,
                             double steps_per_outer_loop,
                             double rtf)
  {
    if (!debug_publish_) {
      return;
    }
    pub_f64(pub_outer_loop_ms_, outer_loop_ms_avg);
    pub_f64(pub_physics_step_hz_, physics_step_hz);
    pub_f64(pub_physics_step_ms_, physics_step_ms);
    pub_f64(pub_steps_per_loop_, steps_per_outer_loop);
    pub_f64(pub_rtf_, rtf);
  }

private:
  // Sensor helpers
  struct SensorSlice { int adr; int dim; };

  SensorSlice cache_sensor_slice_(const std::string & name)
  {
    const int sid = mj_name2id(model_, mjOBJ_SENSOR, name.c_str());
    if (sid < 0) {
      throw std::runtime_error("Sensor '" + name + "' not found in MJCF.");
    }
    SensorSlice s;
    s.adr = static_cast<int>(model_->sensor_adr[sid]);
    s.dim = static_cast<int>(model_->sensor_dim[sid]);

    return s;
  }

  const mjtNum * sensor_ptr_(const SensorSlice & s) const
  {
    return data_->sensordata + s.adr;
  }

  // Commands
  void store_cmd_(const unitree_go::msg::LowCmd::SharedPtr msg)
  {
    std::lock_guard<std::mutex> clk(cmd_mtx_);
    for (int i = 0; i < 12; ++i) {
      const double tau = static_cast<double>(msg->motor_cmd[i].tau);
      const double q = static_cast<double>(msg->motor_cmd[i].q);
      const double dq = static_cast<double>(msg->motor_cmd[i].dq);
      const double kp = static_cast<double>(msg->motor_cmd[i].kp);
      const double kd = static_cast<double>(msg->motor_cmd[i].kd);

      cmd_tau_[i] = tau;
      cmd_q_[i] = q;
      cmd_dq_[i] = dq;
      cmd_kp_[i] = kp;
      cmd_kd_[i] = kd;
      cmd_pd_enable_[i] = (kp != 0.0 || kd != 0.0);
    }
    lowcmd_count_.fetch_add(1, std::memory_order_relaxed);

    // pulse: publish 1.0 each time a cmd is received
    if (debug_publish_ && pub_storing_cmd_ms_) {
      pub_f64(pub_storing_cmd_ms_, 1.0);
    }
  }

  // Actuators / torques
  void cache_actuator_ids_(const std::string & leg)
  {
    const std::string hip   = leg + "_hip";
    const std::string thigh = leg + "_thigh";
    const std::string calf  = leg + "_calf";

    const int hip_id   = mj_name2id(model_, mjOBJ_ACTUATOR, hip.c_str());
    const int thigh_id = mj_name2id(model_, mjOBJ_ACTUATOR, thigh.c_str());
    const int calf_id  = mj_name2id(model_, mjOBJ_ACTUATOR, calf.c_str());

    if (hip_id < 0 || thigh_id < 0 || calf_id < 0) {
      throw std::runtime_error("Actuator(s) not found for leg '" + leg + "'");
    }
    actuator_ids_[leg] = {hip_id, thigh_id, calf_id};
  }

  void set_joint_torque_(const std::array<double, 12> & tau)
  {
    // FR: 0..2, FL: 3..5, RR: 6..8, RL: 9..11
    set_leg_torque_("FR", tau[0], tau[1], tau[2]);
    set_leg_torque_("FL", tau[3], tau[4], tau[5]);
    set_leg_torque_("RR", tau[6], tau[7], tau[8]);
    set_leg_torque_("RL", tau[9], tau[10], tau[11]);
  }

  void set_leg_torque_(const std::string & leg, double hip, double thigh, double calf)
  {
    const auto & ids = actuator_ids_.at(leg);
    data_->ctrl[ids[0]] = hip;
    data_->ctrl[ids[1]] = thigh;
    data_->ctrl[ids[2]] = calf;
  }

  // Viewer
  void viewer_loop_()
  {
    if (!glfwInit()) {
      RCLCPP_ERROR(get_logger(), "GLFW init failed; viewer disabled.");
      return;
    }

    GLFWwindow* window = glfwCreateWindow(1280, 720, "MuJoCo Viewer", nullptr, nullptr);
    if (!window) {
      glfwTerminate();
      RCLCPP_ERROR(get_logger(), "GLFW window creation failed; viewer disabled.");
      return;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);  // vsync

    mjvCamera cam;  mjv_defaultCamera(&cam);
    mjvOption opt;  mjv_defaultOption(&opt);
    mjvScene scn;   mjv_defaultScene(&scn);
    mjrContext con; mjr_defaultContext(&con);

    {
      std::lock_guard<std::mutex> lk(mj_mtx_);
      mjv_makeScene(model_, &scn, 2000);
      mjr_makeContext(model_, &con, mjFONTSCALE_150);
    }

    cam.type = mjCAMERA_FREE;
    cam.lookat[0] = 0.0; cam.lookat[1] = 0.0; cam.lookat[2] = 0.15;
    cam.distance = 1.6;
    cam.elevation = -20;
    cam.azimuth = 90;

    const double render_dt = (render_hz_ > 0.0) ? (1.0 / render_hz_) : 0.0;
    auto next_frame_tp = std::chrono::steady_clock::now();

    while (!viewer_should_exit_.load() && !glfwWindowShouldClose(window)) {
      glfwPollEvents();

      if (render_dt > 0.0) {
        const auto now_tp = std::chrono::steady_clock::now();
        if (now_tp < next_frame_tp) {
          std::this_thread::sleep_for(next_frame_tp - now_tp);
          continue;
        }
        next_frame_tp = now_tp + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(render_dt));
      }

      int width, height;
      glfwGetFramebufferSize(window, &width, &height);
      mjrRect viewport = {0, 0, width, height};

      {
        std::lock_guard<std::mutex> lk(mj_mtx_);
        mj_copyData(render_data_, model_, data_);
      }

      mjv_updateScene(model_, render_data_, &opt, nullptr, &cam, mjCAT_ALL, &scn);
      mjr_render(viewport, &scn, &con);

      glfwSwapBuffers(window);
    }

    {
      std::lock_guard<std::mutex> lk(mj_mtx_);
      mjr_freeContext(&con);
      mjv_freeScene(&scn);
    }

    glfwDestroyWindow(window);
    glfwTerminate();
  }

private:
  double sample_noise_(double stddev)
  {
    if (stddev <= 0.0) return 0.0;
    return stddev * normal_(rng_);
  }

  // MuJoCo
  mjModel * model_{nullptr};
  mjData  * data_{nullptr};
  mjData  * render_data_{nullptr};
  std::string xml_path_;

  // Thread safety for MuJoCo access
  mutable std::mutex mj_mtx_;

  // Rates
  double sim_hz_{1000.0};
  double pub_hz_{200.0};
  int pub_decim_{5};
  double sim_dt_{0.001};

  // Sensors
  SensorSlice imu_quat_{}, imu_gyro_{}, imu_acc_{};
  SensorSlice fl_touch_{}, fr_touch_{}, rl_touch_{}, rr_touch_{};

  // Geoms / actuators
  std::unordered_map<std::string, int> foot_geoms_;
  std::unordered_map<std::string, std::array<int,3>> actuator_ids_;

  // ROS
  rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr pub_lowstate_;

  // Last applied joint torques in Unitree motor_state order.
  std::array<double, 12> last_tau_{};
  rclcpp::Publisher<go2_msgs::msg::QDq>::SharedPtr pub_qdq_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Subscription<unitree_go::msg::LowCmd>::SharedPtr sub_cmd_;

  // Telemetry pubs
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_outer_loop_ms_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_physics_step_hz_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_physics_step_ms_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_steps_per_loop_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_rtf_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_storing_cmd_ms_;


  // Heartbeat
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heartbeat_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  // Command storage
  std::mutex cmd_mtx_;
  std::array<double, 12> cmd_tau_{};
  std::array<double, 12> cmd_q_{};
  std::array<double, 12> cmd_dq_{};
  std::array<double, 12> cmd_kp_{};
  std::array<double, 12> cmd_kd_{};
  std::array<bool, 12> cmd_pd_enable_{};

  // LowCmd rate tracking
  std::atomic<uint64_t> lowcmd_count_{0};

  // Viewer
  bool debug_print_{true};
  bool debug_publish_{true};
  double foot_force_lpf_alpha_{0.1};
  bool foot_force_lpf_init_{false};
  std::array<float, 4> foot_force_lpf_{};
  double imu_gyro_noise_std_{0.0};
  double imu_acc_noise_std_{0.0};
  int imu_noise_seed_{0};
  std::mt19937 rng_{};
  std::normal_distribution<double> normal_{0.0, 1.0};
  bool enable_viewer_{true};
  bool freeze_base_{false};
  double render_hz_{30.0};
  std::array<double, 7> base_qpos_{};
  std::atomic<bool> viewer_should_exit_{false};
  std::thread viewer_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MuJoCoRobot>();

  // Start viewer (optional)
  node->start_viewer();

  // Spin ROS in background
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  std::thread spin_thread([&exec]() { exec.spin(); });

  const double sim_dt = node->sim_dt();
  const int pub_decim = node->pub_decim();

  // Real-time sync: align sim time to wall time
  auto wall_t0 = std::chrono::steady_clock::now();
  double sim_t0 = node->sim_time();

  // Telemetry window (~10 Hz)
  auto tel_t0 = std::chrono::steady_clock::now();
  double tel_sim0 = node->sim_time();
  int steps_acc = 0;
  double simstep_wall_acc_s = 0.0;
  double outer_wall_acc_s = 0.0;
  int outer_loops_acc = 0;

  int sim_step_count = 0;
  bool did_resync = false;

  // Lowcmd 1-second window logger
  auto cmd_last_tp = std::chrono::steady_clock::now();
  uint64_t cmd_last_c = node->lowcmd_count();
  double loop_acc_s = 0.0;
  int loop_acc_n = 0;
  auto loop_last_tp = std::chrono::steady_clock::now();
  double period_acc_s = 0.0;
  int period_acc_n = 0;

  try {
    while (rclcpp::ok()) {
      const auto outer_t0 = std::chrono::steady_clock::now();
      {
        const double period_s = std::chrono::duration<double>(outer_t0 - loop_last_tp).count();
        loop_last_tp = outer_t0;
        period_acc_s += period_s;
        period_acc_n += 1;
      }

      // Single physics step per outer loop (no catch-up)
      const auto ts0 = std::chrono::steady_clock::now();
      node->sim_step();
      const auto ts1 = std::chrono::steady_clock::now();

      simstep_wall_acc_s += std::chrono::duration<double>(ts1 - ts0).count();

      ++sim_step_count;
      ++steps_acc;

      if (!did_resync) {
        wall_t0 = std::chrono::steady_clock::now();
        sim_t0 = node->sim_time();
        did_resync = true;
      }

      if ((sim_step_count % pub_decim) == 0) {
        node->publish_state();
      }

      const auto outer_t1 = std::chrono::steady_clock::now();
      const double outer_loop_s = std::chrono::duration<double>(outer_t1 - outer_t0).count();

      outer_wall_acc_s += outer_loop_s;
      outer_loops_acc += 1;
      loop_acc_s += outer_loop_s;
      loop_acc_n += 1;

      // 1 Hz-ish log: sim_time + lowcmd hz (proper 1-second window)
      if (node->debug_print()) {
        const auto now_tp = std::chrono::steady_clock::now();
        const double dt = std::chrono::duration<double>(now_tp - cmd_last_tp).count();
        if (dt >= 1.0) {
          const uint64_t c = node->lowcmd_count();
          const double cmd_hz = static_cast<double>(c - cmd_last_c) / dt;

          const double loop_ms = (loop_acc_n > 0) ? (loop_acc_s / loop_acc_n) * 1000.0 : 0.0;
          const double period_ms = (period_acc_n > 0) ? (period_acc_s / period_acc_n) * 1000.0 : 0.0;
          RCLCPP_INFO(
            node->get_logger(),
            "sim_time=%.3f    loop_ms=%.3f   period_ms=%.3f   lowcmd_hz=%.1f",
            node->sim_time(), loop_ms, period_ms, cmd_hz
          );

          cmd_last_tp = now_tp;
          cmd_last_c = c;
          loop_acc_s = 0.0;
          loop_acc_n = 0;
          period_acc_s = 0.0;
          period_acc_n = 0;
        }
      }

      // Sleep: aim for next sim tick (based on sim time vs wall time)
      const double wall_now_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_t0).count();
      const double wall_next_s = (node->sim_time() + sim_dt) - sim_t0;
      double sleep_s = wall_next_s - wall_now_s;

      if (sleep_s > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
      } else {
        const double lag_ms = (-sleep_s) * 1000.0;
        if (lag_ms > 1.0 && node->debug_print()) {
          RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
            "Sim is behind realtime by %.3f ms", lag_ms);
        }
        std::this_thread::sleep_for(200us);
      }

      // Publish timing window at ~10 Hz
      const auto tel_now = std::chrono::steady_clock::now();
      const double tel_wall_dt = std::chrono::duration<double>(tel_now - tel_t0).count();
      if (tel_wall_dt >= 0.1 && node->debug_publish()) {
        const double tel_sim1 = node->sim_time();
        const double tel_sim_dt = tel_sim1 - tel_sim0;

        const double rtf = (tel_wall_dt > 1e-9) ? (tel_sim_dt / tel_wall_dt) : 0.0;
        const double step_hz = (tel_wall_dt > 1e-9) ? (static_cast<double>(steps_acc) / tel_wall_dt) : 0.0;
        const double step_ms = (steps_acc > 0) ? (simstep_wall_acc_s / steps_acc) * 1000.0 : 0.0;
        const double outer_ms = (outer_loops_acc > 0) ? (outer_wall_acc_s / outer_loops_acc) * 1000.0 : 0.0;
      const double steps_per_loop = (outer_loops_acc > 0) ? (static_cast<double>(steps_acc) / outer_loops_acc) : 0.0;

        node->publish_timing_window(outer_ms, step_hz, step_ms, steps_per_loop, rtf);

        // reset window
        tel_t0 = tel_now;
        tel_sim0 = tel_sim1;
        steps_acc = 0;
        simstep_wall_acc_s = 0.0;
        outer_wall_acc_s = 0.0;
        outer_loops_acc = 0;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in sim loop: %s", e.what());
  }

  // shutdown
  node->stop_viewer();

  exec.cancel();
  if (spin_thread.joinable()) spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
