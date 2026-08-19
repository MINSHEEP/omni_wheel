/**
 * @file kiwi_base_node.cpp
 * @brief Kiwi 三轮全向底盘驱动节点
 *
 * 【数据流向】
 *   输入: /cmd_vel (Twist) → 速度限幅 → 逆运动学 → 电机指令 → USB CDC
 *   输出: 电机反馈 → 低通/死区 → 正运动学 → 原始里程计与关节状态
 *
 * 【发布话题】
 *   /chassis/odom/raw      - 电机实测速度推算的原始里程计（位姿 + 速度）
 *   /joint_states          - 三轮关节状态
 *   /chassis/imu/data_raw   - 异步 0x3B IMU 遥测
 *   /chassis/imu/calibration_ready - IMU 首个完整静止标定窗口完成状态
 *   /chassis/feedback/raw  - 异步 0x3B 原始字段调试消息
 *
 * 0x3A 命令没有 ACK，发送链永不等待 0x3B。异步 0x3B 只作为独立的电机/IMU
 * 测量流，里程计不使用 /cmd_vel 做开环推算。
 */

#include "kiwi_base/kiwi_kinematics.hpp"
#include "kiwi_base/measurement_processing.hpp"
#include "kiwi_base/robomaster_protocol.hpp"
#include "kiwi_base/stationary_covariance_estimator.hpp"
#include "kiwi_base/yaw_drift_estimator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <kiwi_base/msg/feedback_debug.hpp>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace kiwi_base
{
  namespace
  {
    using Clock = std::chrono::steady_clock;
    constexpr double kStationaryVelocityEpsilon = 1e-4;
    constexpr double kTwoPi = 6.28318530717958647692;
    constexpr double kStandardGravityMps2 = 9.80665;
    constexpr size_t kMaxSerialReadsPerUpdate = 8;

    //==============================================================================
    // 工具函数
    //==============================================================================

    /** 将字节数组转换为十六进制字符串，用于串口调试 */
    [[maybe_unused]]
    std::string to_hex(const std::vector<uint8_t> &data)
    {
      static constexpr char hex[] = "0123456789abcdef";
      std::string result;
      result.reserve(data.size() * 3);
      for (size_t i = 0; i < data.size(); ++i)
      {
        if (i > 0)
          result += ' ';
        result += hex[data[i] >> 4];
        result += hex[data[i] & 0x0f];
      }
      return result;
    }

    /** 欧拉角（roll/pitch/yaw）转四元数 */
    std::array<double, 4> euler_to_quat(double roll, double pitch, double yaw)
    {
      const double cr = std::cos(roll * 0.5);
      const double sr = std::sin(roll * 0.5);
      const double cp = std::cos(pitch * 0.5);
      const double sp = std::sin(pitch * 0.5);
      const double cy = std::cos(yaw * 0.5);
      const double sy = std::sin(yaw * 0.5);

      return {
          sr * cp * cy - cr * sp * sy, // x
          cr * sp * cy + sr * cp * sy, // y
          cr * cp * sy - sr * sp * cy, // z
          cr * cp * cy + sr * sp * sy  // w
      };
    }

    /** 声明必需参数，缺失时抛出异常 */
    template <typename T>
    T require_param(rclcpp::Node &node, const std::string &name)
    {
      return node.declare_parameter<T>(name);
    }

    /** 读取有限正数参数。 */
    template<typename T>
    T require_positive(rclcpp::Node & node, const std::string & name)
    {
      const auto value = require_param<T>(node, name);
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument(name + " must be finite and positive");
      return value;
    }

    /** 读取仅由 +1/-1 组成的三轴符号向量。 */
    Vector3 require_sign_vector3(rclcpp::Node & node, const std::string & name)
    {
      const auto values = require_param<std::vector<double>>(node, name);
      if (values.size() != 3 || std::any_of(
          values.begin(), values.end(),
          [](double value) {
            return !std::isfinite(value) || std::abs(value) != 1.0;
          }))
      {
        throw std::invalid_argument(name + " must be 3 finite values equal to +1 or -1");
      }
      return {values[0], values[1], values[2]};
    }

    /** 读取三个有限正数。 */
    Vector3 require_positive_vector3(rclcpp::Node & node, const std::string & name)
    {
      const auto values = require_param<std::vector<double>>(node, name);
      if (values.size() != 3 || std::any_of(
          values.begin(), values.end(),
          [](double value) {return !std::isfinite(value) || value <= 0.0;}))
      {
        throw std::invalid_argument(name + " must be 3 finite positive values");
      }
      return {values[0], values[1], values[2]};
    }

    /** 读取三个有限非负数，适用于允许关闭单轴过滤的死区参数。 */
    Vector3 require_non_negative_vector3(rclcpp::Node & node, const std::string & name)
    {
      const auto values = require_param<std::vector<double>>(node, name);
      if (values.size() != 3 || std::any_of(
          values.begin(), values.end(),
          [](double value) {return !std::isfinite(value) || value < 0.0;}))
      {
        throw std::invalid_argument(name + " must be 3 finite non-negative values");
      }
      return {values[0], values[1], values[2]};
    }

    /** 读取三轴方差参数（必须为正数）。 */
    std::array<double, 3> require_variance(rclcpp::Node &node, const std::string &name)
    {
      const auto values = require_param<std::vector<double>>(node, name);
      if (values.size() != 3 || std::any_of(values.begin(), values.end(),
                                            [](double v)
                                            { return !std::isfinite(v) || v <= 0.0; }))
        throw std::invalid_argument(name + " must be 3 finite positive values");
      return {values[0], values[1], values[2]};
    }

    /** 读取非负样本数；参数错误时拒绝启动，避免用代码值覆盖 YAML 配置。 */
    size_t require_non_negative_count(rclcpp::Node &node, const std::string &name)
    {
      const auto value = require_param<int64_t>(node, name);
      if (value < 0)
        throw std::invalid_argument(name + " must be non-negative");
      return static_cast<size_t>(value);
    }

    /** 读取至少为指定下限的样本数，避免样本方差发生除零。 */
    size_t require_minimum_count(
      rclcpp::Node & node, const std::string & name, size_t minimum)
    {
      const auto value = require_param<int64_t>(node, name);
      if (value < 0 || static_cast<size_t>(value) < minimum)
        throw std::invalid_argument(
                name + " must be at least " + std::to_string(minimum));
      return static_cast<size_t>(value);
    }

    /** 读取 (0, 1] 内的平滑系数，避免方差更新冻结或越过观测值。 */
    double require_unit_interval(rclcpp::Node & node, const std::string & name)
    {
      const double value = require_positive<double>(node, name);
      if (value > 1.0)
        throw std::invalid_argument(name + " must be no greater than 1.0");
      return value;
    }

    /** 用 EMA 平滑已提交方差；异常候选不得污染后续 IMU 消息。 */
    double smooth_variance(double current, double candidate, double alpha)
    {
      if (!std::isfinite(candidate) || candidate <= 0.0)
        return current;
      const double updated = current + alpha * (candidate - current);
      return std::isfinite(updated) && updated > 0.0 ? updated : current;
    }

  } // namespace

  //==============================================================================
  // KiwiBaseNode - 主节点类
  //==============================================================================

  /**
   * @brief Kiwi 底盘驱动节点
   *
   * 负责速度指令下发、电机反馈轮式里程计，以及异步 IMU 遥测发布。
   */
  class KiwiBaseNode final : public rclcpp::Node
  {
  public:
    KiwiBaseNode()
        : Node("kiwi_base_node"),
          parser_([this](uint8_t ver, const std::vector<uint8_t> &payload)
                  { on_feedback_frame(ver, payload); })
    {
      load_parameters();
      create_publishers();
      create_subscribers();
      open_serial();

      // 启动固定周期定时器
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / cfg_.publish_rate));
      // 极高频率配置向下取整后也至少保留 1 ms，避免创建零周期定时器。
      const auto period_ms = std::max(std::chrono::milliseconds(1), period);
      timer_ = create_wall_timer(period_ms, [this]()
                                 { update(); });

      RCLCPP_INFO(get_logger(), "Kiwi base node started (%.1f Hz)", cfg_.publish_rate);
    }

    ~KiwiBaseNode() override
    {
      // 退出时发送停止指令
      CommandFrame stop{};
      stop.enable = false;
      (void)send_command(stop);
      close_serial();
    }

  private:
    //============================================================================
    // 配置结构体 - 集中管理所有参数
    //============================================================================

    struct Config
    {
      // 串口
      std::string device;
      double publish_rate;
      bool log_frames;
      bool feedback_framed;
      double feedback_motor_scale;
      double motor_feedback_lowpass_cutoff;
      double cmd_vel_timeout_s;

      // 机械参数
      std::array<double, 3> motor_ratio;
      double wheel_radius;
      double wheel_distance;
      double max_wheel_speed;

      // 速度限幅
      double max_vx;
      double max_vy;
      double max_wz;
      double max_ax;
      double max_ay;
      double max_aw;

      // 死区阈值
      Vector3 motor_deadband;
      Vector3 wheel_twist_deadband;
      Vector3 imu_gyro_deadband;

      // IMU 预处理
      Vector3 imu_gyro_sign;
      Vector3 imu_gyro_scale;
      double imu_stationary_hold;
      double imu_max_sample_gap;
      double imu_stationary_yaw_rate_threshold;
      double imu_stationary_accel_tolerance;
      bool imu_publish_orientation;

      // yaw 零漂补偿
      bool enable_yaw_drift_compensation;
      double yaw_drift_tau;
      double yaw_drift_max;
      size_t yaw_drift_calib_samples;

      // 陀螺零漂滤波
      size_t gyro_calib_samples;
      double gyro_bias_tau;
      double gyro_max_bias;
      double gyro_lowpass_cutoff;

      // 测量方差（EKF 用）
      std::array<double, 3> wheel_pose_var;
      std::array<double, 3> wheel_twist_var;
      std::array<double, 3> imu_orient_var;
      std::array<double, 3> imu_gyro_var;
      std::array<double, 3> imu_accel_var;
      size_t covariance_calib_samples;
      double yaw_variance_floor;
      double gyro_z_variance_floor;
      double covariance_update_alpha;

      // 话题和坐标系
      std::string topic_cmd_vel;
      std::string topic_odom;
      std::string topic_imu;
      std::string topic_imu_ready;
      std::string topic_joints;
      std::string topic_debug;
      std::string frame_odom;
      std::string frame_base;
      std::string frame_imu;
      std::vector<std::string> wheel_names;
    };

    //============================================================================
    // 状态变量
    //============================================================================

    struct State
    {
      // 串口
      int fd{-1};
      std::vector<uint8_t> raw_buffer;
      std::optional<FeedbackFrame> pending_feedback;

      // 控制
      Twist2D cmd{0.0, 0.0, 0.0};

      // 里程计积分
      std::array<double, 3> wheel_pos{0.0, 0.0, 0.0};
      std::array<double, 3> wheel_vel{0.0, 0.0, 0.0};
      double odom_x{0.0};
      double odom_y{0.0};
      double odom_yaw{0.0};

      // 时间跟踪
      bool have_prev_time{false};
      Clock::time_point prev_time;

      // 电机反馈低通状态；断流恢复时由首帧重置，禁止旧速度拖尾。
      Vector3 motor_lowpass_state{};
      bool motor_lowpass_initialized{false};

      // 静止检测（用于陀螺零漂估计）
      bool stationary_candidate{false};
      Clock::time_point stationary_since;
      bool have_previous_feedback_yaw{false};
      double previous_feedback_yaw{0.0};

      // yaw 零漂标定日志状态；算法状态由 YawDriftEstimator 独立维护。
      bool yaw_drift_logged{false};
      bool covariance_logged{false};
      // 已提交方差与当前静止累加窗口分离；运动时保留，避免 EKF 权重突变。
      double committed_yaw_variance{0.0};
      double committed_gyro_z_variance{0.0};

      // cmd_vel 超时保护
      rclcpp::Time last_cmd_time;
      bool have_cmd{false};

      // 其他
      bool gyro_bias_logged{false};
      bool imu_ready{false};
    };

    //============================================================================
    // 初始化函数
    //============================================================================

    /** 从 YAML 加载所有参数 */
    void load_parameters()
    {
      cfg_.device = require_param<std::string>(*this, "device");
      cfg_.publish_rate = require_positive<double>(*this, "publish_rate");
      cfg_.log_frames = require_param<bool>(*this, "log_serial_frames");
      cfg_.feedback_framed = require_param<bool>(*this, "feedback_framed");
      cfg_.feedback_motor_scale = require_positive<double>(*this, "feedback_motor_scale_to_rad_s");
      cfg_.motor_feedback_lowpass_cutoff = require_positive<double>(
        *this, "motor_feedback_lowpass_cutoff_hz");
      cfg_.cmd_vel_timeout_s = require_positive<double>(*this, "cmd_vel_timeout_s");

      // 机械参数
      cfg_.motor_ratio = require_positive_vector3(*this, "motor_reduction_ratio");

      cfg_.wheel_radius = require_positive<double>(*this, "wheel_radius_m");
      cfg_.wheel_distance = require_positive<double>(*this, "wheel_center_distance_m");
      cfg_.max_wheel_speed = require_positive<double>(*this, "max_wheel_speed_rad_s");

      // 速度限幅
      cfg_.max_vx = require_positive<double>(*this, "max_linear_speed_m_s");
      cfg_.max_vy = require_positive<double>(*this, "max_lateral_speed_m_s");
      cfg_.max_wz = require_positive<double>(*this, "max_angular_speed_rad_s");
      cfg_.max_ax = require_positive<double>(*this, "max_linear_acceleration_m_s2");
      cfg_.max_ay = require_positive<double>(*this, "max_lateral_acceleration_m_s2");
      cfg_.max_aw = require_positive<double>(*this, "max_angular_acceleration_rad_s2");

      // 死区
      cfg_.motor_deadband = require_non_negative_vector3(
        *this, "motor_feedback_deadband_rad_s");
      cfg_.wheel_twist_deadband = require_non_negative_vector3(*this, "wheel_twist_deadband");
      cfg_.imu_gyro_deadband = require_non_negative_vector3(
        *this, "imu_angular_velocity_deadband_rad_s");

      // IMU 预处理
      cfg_.imu_gyro_sign = require_sign_vector3(*this, "imu_angular_velocity_sign");
      cfg_.imu_gyro_scale = require_positive_vector3(*this, "imu_angular_velocity_scale");
      cfg_.imu_stationary_hold = require_positive<double>(*this, "imu_stationary_hold_s");
      cfg_.imu_max_sample_gap = require_positive<double>(*this, "imu_max_sample_gap_s");
      cfg_.imu_stationary_yaw_rate_threshold = require_positive<double>(
        *this, "imu_stationary_yaw_rate_threshold_rad_s");
      cfg_.imu_stationary_accel_tolerance = require_positive<double>(
        *this, "imu_stationary_acceleration_tolerance_m_s2");
      cfg_.imu_publish_orientation = require_param<bool>(*this, "imu_publish_orientation");

      // yaw 零漂补偿
      cfg_.enable_yaw_drift_compensation = require_param<bool>(*this, "imu_yaw_drift_compensation");
      cfg_.yaw_drift_tau = require_positive<double>(*this, "imu_yaw_drift_time_constant_s");
      cfg_.yaw_drift_max = require_positive<double>(*this, "imu_yaw_drift_max_rate_rad_s");
      cfg_.yaw_drift_calib_samples = require_non_negative_count(
          *this, "imu_yaw_drift_calibration_samples");

      // 陀螺零漂滤波
      cfg_.gyro_calib_samples = require_non_negative_count(*this, "imu_gyro_calibration_samples");
      cfg_.gyro_bias_tau = require_positive<double>(*this, "imu_gyro_bias_time_constant_s");
      cfg_.gyro_max_bias = require_positive<double>(*this, "imu_gyro_max_bias_rad_s");
      cfg_.gyro_lowpass_cutoff = require_positive<double>(*this, "imu_gyro_lowpass_cutoff_hz");

      // 方差
      cfg_.wheel_pose_var = require_variance(*this, "wheel_pose_variance");
      cfg_.wheel_twist_var = require_variance(*this, "wheel_twist_variance");
      cfg_.imu_orient_var = require_variance(*this, "imu_orientation_variance");
      cfg_.imu_gyro_var = require_variance(*this, "imu_angular_velocity_variance");
      cfg_.imu_accel_var = require_variance(*this, "imu_linear_acceleration_variance");
      cfg_.covariance_calib_samples = require_minimum_count(
        *this, "imu_covariance_calibration_samples", 3);
      cfg_.yaw_variance_floor = require_positive<double>(*this, "imu_yaw_variance_floor");
      cfg_.gyro_z_variance_floor = require_positive<double>(
        *this, "imu_gyro_z_variance_floor");
      cfg_.covariance_update_alpha = require_unit_interval(
        *this, "imu_covariance_update_alpha");
      if (cfg_.yaw_variance_floor > cfg_.imu_orient_var[2] ||
        cfg_.gyro_z_variance_floor > cfg_.imu_gyro_var[2])
      {
        throw std::invalid_argument(
                "IMU variance floors must not exceed their startup variances");
      }
      state_.committed_yaw_variance = cfg_.imu_orient_var[2];
      state_.committed_gyro_z_variance = cfg_.imu_gyro_var[2];

      // 话题和坐标系
      cfg_.topic_cmd_vel = require_param<std::string>(*this, "cmd_vel_topic");
      cfg_.topic_odom = require_param<std::string>(*this, "wheel_odom_topic");
      cfg_.topic_imu = require_param<std::string>(*this, "imu_topic");
      cfg_.topic_imu_ready = require_param<std::string>(
        *this, "imu_calibration_ready_topic");
      cfg_.topic_joints = require_param<std::string>(*this, "joint_states_topic");
      cfg_.topic_debug = require_param<std::string>(*this, "feedback_debug_topic");
      cfg_.frame_odom = require_param<std::string>(*this, "odom_frame_id");
      cfg_.frame_base = require_param<std::string>(*this, "base_frame_id");
      cfg_.frame_imu = require_param<std::string>(*this, "imu_frame_id");
      cfg_.wheel_names = require_param<std::vector<std::string>>(*this, "wheel_names");

      if (cfg_.wheel_names.size() != 3)
        throw std::invalid_argument("wheel_names must have 3 entries");

      // 初始化运动学
      kinematics_.emplace(cfg_.wheel_radius, cfg_.wheel_distance, cfg_.max_wheel_speed);

      // 初始化 yaw 展开与漂移估计器。
      YawDriftEstimatorConfig yaw_cfg{};
      yaw_cfg.enabled = cfg_.enable_yaw_drift_compensation;
      yaw_cfg.time_constant_s = cfg_.yaw_drift_tau;
      yaw_cfg.max_rate_rad_s = cfg_.yaw_drift_max;
      yaw_cfg.calibration_samples = cfg_.yaw_drift_calib_samples;
      yaw_drift_estimator_.emplace(yaw_cfg);

      // 静止窗口只估计 EKF 实际融合的 yaw 与 gyro_z；X/Y 保留静态保守方差。
      StationaryCovarianceConfig covariance_cfg{};
      covariance_cfg.minimum_samples = cfg_.covariance_calib_samples;
      covariance_cfg.yaw_variance_floor = cfg_.yaw_variance_floor;
      covariance_cfg.gyro_z_variance_floor = cfg_.gyro_z_variance_floor;
      covariance_estimator_.emplace(covariance_cfg);

      // 初始化陀螺滤波器
      GyroFilterConfig gyro_cfg{};
      gyro_cfg.calibration_samples = cfg_.gyro_calib_samples;
      gyro_cfg.bias_time_constant_s = cfg_.gyro_bias_tau;
      gyro_cfg.max_bias_rad_s = cfg_.gyro_max_bias;
      gyro_cfg.lowpass_cutoff_hz = cfg_.gyro_lowpass_cutoff;
      gyro_filter_.emplace(gyro_cfg);
    }

    /** 创建发布者 */
    void create_publishers()
    {
      pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(cfg_.topic_odom, 10);
      pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(cfg_.topic_imu, 10);
      // 标定状态是低频状态量而不是数据流；可靠、瞬态本地 QoS 保证后启动的
      // 建图门控不会错过 ready=true，也不会把未标定状态误认为已就绪。
      pub_imu_ready_ = create_publisher<std_msgs::msg::Bool>(
        cfg_.topic_imu_ready, rclcpp::QoS(1).reliable().transient_local());
      pub_joints_ = create_publisher<sensor_msgs::msg::JointState>(cfg_.topic_joints, 10);
      pub_debug_ = create_publisher<kiwi_base::msg::FeedbackDebug>(cfg_.topic_debug, 10);

      std_msgs::msg::Bool initial_state;
      initial_state.data = false;
      pub_imu_ready_->publish(initial_state);
    }

    /** 创建订阅者 */
    void create_subscribers()
    {
      sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
          cfg_.topic_cmd_vel, 10,
          [this](const geometry_msgs::msg::Twist::SharedPtr msg)
          {
            on_cmd_vel(msg);
          });
    }

    //============================================================================
    // 串口操作
    //============================================================================

    /** 打开串口，失败时保持 fd=-1 */
    void open_serial()
    {
      if (state_.fd >= 0)
        return;

      state_.fd = open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (state_.fd < 0)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Failed to open %s: %s", cfg_.device.c_str(), std::strerror(errno));
        return;
      }

      // 同一物理 C 板不能被两个 ROS_DOMAIN_ID 中的底盘节点同时读取：两个 read()
      // 会随机分走字节流，最终表现为反馈断帧、EKF/TF 卡顿和导航恢复旋转。
      // TIOCEXCL 让后启动的进程在 open() 阶段得到 EBUSY；关闭 fd 后内核自动释放。
      if (ioctl(state_.fd, TIOCEXCL) != 0)
      {
        const int error = errno;
        close(state_.fd);
        state_.fd = -1;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Failed to claim %s exclusively: %s (is another base node running?)",
            cfg_.device.c_str(), std::strerror(error));
        return;
      }

      termios attr{};
      if (tcgetattr(state_.fd, &attr) != 0 ||
          (cfmakeraw(&attr), attr.c_cflag |= CLOCAL | CREAD,
           tcsetattr(state_.fd, TCSANOW, &attr) != 0))
      {
        close_serial();
        return;
      }

      RCLCPP_INFO(get_logger(), "Serial opened: %s", cfg_.device.c_str());
    }

    /** 关闭串口并清空缓冲 */
    void close_serial()
    {
      if (state_.fd >= 0)
      {
        close(state_.fd);
        state_.fd = -1;
      }
      parser_.reset();
      state_.raw_buffer.clear();
      state_.pending_feedback.reset();
      // 串口断开/重连会破坏样本连续性；只清尚未提交的观测状态，已提交方差保留。
      state_.have_prev_time = false;
      state_.have_previous_feedback_yaw = false;
      state_.stationary_candidate = false;
      if (yaw_drift_estimator_)
        yaw_drift_estimator_->notify_motion();
      if (covariance_estimator_)
        covariance_estimator_->notify_motion();
    }

    /** 读取串口数据 */
    void read_serial()
    {
      if (state_.fd < 0)
      {
        open_serial();
        return;
      }

      uint8_t buffer[512];
      for (size_t read_count = 0; read_count < kMaxSerialReadsPerUpdate; ++read_count)
      {
        const ssize_t n = read(state_.fd, buffer, sizeof(buffer));

        if (n > 0)
        {
          // 根据配置选择帧解析或裸数据模式。
          if (cfg_.feedback_framed)
            parser_.feed(buffer, static_cast<size_t>(n));
          else
            on_raw_data(buffer, static_cast<size_t>(n));
          continue;
        }

        if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)))
          return;

        if (n < 0 && errno == EINTR)
          continue;

        // 读取错误，关闭串口。
        close_serial();
        return;
      }
      // 即使遥测持续灌入，也给本周期 TX 留出确定的执行机会；剩余数据下周期读取。
    }

    /** 发送一帧完整指令；只有完整写入时返回 true。 */
    bool send_command(const CommandFrame &cmd)
    {
      if (state_.fd < 0)
        return false;

      const auto frame = encode_command(cmd);
      ssize_t written = -1;
      do
      {
        written = write(state_.fd, frame.data(), frame.size());
      } while (written < 0 && errno == EINTR);

      if (written == static_cast<ssize_t>(frame.size()))
        return true;

      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      {
        // 非阻塞发送缓冲暂满，下个控制周期会重新发送最新完整指令。
        return false;
      }

      if (written < 0)
      {
        // 只有不可恢复的真实 I/O 错误才重置设备。
        close_serial();
        return false;
      }

      // 短写不是设备故障；放弃残帧并等待下一周期从完整帧重发。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Short serial write: %zd/%zu bytes",
        written, frame.size());
      return false;
    }

    //============================================================================
    // 回调函数
    //============================================================================

    /** 速度指令回调 - 限幅后保存 */
    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      const Twist2D raw{msg->linear.x, msg->linear.y, msg->angular.z};
      const Twist2D limits{cfg_.max_vx, cfg_.max_vy, cfg_.max_wz};
      state_.cmd = limit_twist(raw, limits);
      const bool command_requests_motion =
        std::abs(state_.cmd.vx) >= kStationaryVelocityEpsilon ||
        std::abs(state_.cmd.vy) >= kStationaryVelocityEpsilon ||
        std::abs(state_.cmd.wz) >= kStationaryVelocityEpsilon;
      if (command_requests_motion)
      {
        // 命令事件可能发生在两帧反馈之间；立即失效旧静止区间，禁止跨运动估计漂移。
        state_.stationary_candidate = false;
        yaw_drift_estimator_->notify_motion();
        covariance_estimator_->notify_motion();
      }
      state_.last_cmd_time = this->now();
      state_.have_cmd = true;
    }

    /** 帧解析回调 - 缓存最新反馈 */
    void on_feedback_frame(uint8_t version, const std::vector<uint8_t> &payload)
    {
      if (version != kProtocolVersion)
        return;

      FeedbackFrame frame{};
      if (decode_feedback(payload, frame))
        state_.pending_feedback = frame;
    }

    /** 裸数据回调 - 按 48 字节拼接 */
    void on_raw_data(const uint8_t *data, size_t size)
    {
      state_.raw_buffer.insert(state_.raw_buffer.end(), data, data + size);

      while (state_.raw_buffer.size() >= kFeedbackPayloadLength)
      {
        const std::vector<uint8_t> payload(
            state_.raw_buffer.begin(),
            state_.raw_buffer.begin() + kFeedbackPayloadLength);

        FeedbackFrame frame{};
        if (decode_feedback(payload, frame))
          state_.pending_feedback = frame;
        else
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "Invalid raw feedback (not 12 finite floats), discarding 48 bytes");

        state_.raw_buffer.erase(
            state_.raw_buffer.begin(),
            state_.raw_buffer.begin() + kFeedbackPayloadLength);
      }
    }

    //============================================================================
    // 主循环
    //============================================================================

    /** 定时器回调 - 读取串口、处理反馈、发送指令 */
    void update()
    {
      // 发布者中断后在配置时限内强制清零，防止底盘继续执行陈旧速度。
      const auto now = this->now();
      if (state_.have_cmd &&
        (now - state_.last_cmd_time).seconds() > cfg_.cmd_vel_timeout_s)
      {
        state_.cmd = Twist2D{};
        state_.have_cmd = false;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "cmd_vel timed out after %.3f s; commanding stop", cfg_.cmd_vel_timeout_s);
      }

      // 0x3B 是可有可无的异步遥测；非阻塞读取绝不构成 0x3A 的 ACK 等待。
      read_serial();
      if (state_.pending_feedback)
      {
        process_feedback(*state_.pending_feedback);
        state_.pending_feedback.reset();
      }

      // 发送电机指令
      const auto inverse_result = kinematics_->inverse_with_twist(state_.cmd);
      const auto & wheel_speed = inverse_result.wheel_rad_s;

      CommandFrame cmd{};
      cmd.enable = true;
      for (size_t i = 0; i < 3; ++i)
        cmd.motor_rad_s[i] = static_cast<float>(wheel_speed[i] * cfg_.motor_ratio[i]);

      // 0x3A 无 ACK；写入结果只影响串口重连，不参与里程计状态估算。
      (void)send_command(cmd);

      // 调试日志
      if (cfg_.log_frames)
      {
        RCLCPP_INFO(get_logger(),
                    "CMD: vx=%.3f vy=%.3f wz=%.3f | WHEEL: %.2f %.2f %.2f | MOTOR: %.2f %.2f %.2f",
                    inverse_result.achievable_twist.vx,
                    inverse_result.achievable_twist.vy,
                    inverse_result.achievable_twist.wz,
                    wheel_speed[0], wheel_speed[1], wheel_speed[2],
                    cmd.motor_rad_s[0], cmd.motor_rad_s[1], cmd.motor_rad_s[2]);
      }
    }

    /**
     * 对电机转子速度做一阶低通，再应用静止死区。
     *
     * 首帧或反馈断流恢复时直接以当前测量重置滤波器，既不跨未知时长推进，
     * 也不把断流前的旧速度拖入恢复后的里程计。
     */
    Vector3 filter_motor_feedback(
      const Vector3 & motor_scaled, double dt, bool sample_contiguous)
    {
      if (!state_.motor_lowpass_initialized || !sample_contiguous)
      {
        state_.motor_lowpass_state = motor_scaled;
        state_.motor_lowpass_initialized = true;
      }
      else
      {
        const double rc = 1.0 / (kTwoPi * cfg_.motor_feedback_lowpass_cutoff);
        const double alpha = dt / (dt + rc);
        for (size_t i = 0; i < state_.motor_lowpass_state.size(); ++i)
        {
          state_.motor_lowpass_state[i] +=
            alpha * (motor_scaled[i] - state_.motor_lowpass_state[i]);
        }
      }
      return correct_vector(
        state_.motor_lowpass_state, {1.0, 1.0, 1.0}, cfg_.motor_deadband);
    }

    /** 用异步电机实测值更新轮速和轮位置；该路径不把 0x3B 当成命令 ACK。 */
    void update_feedback_wheel_state(const Vector3 & motor_filtered, double dt)
    {
      for (size_t i = 0; i < state_.wheel_vel.size(); ++i)
      {
        // motor_actual 是电机转子轴速度，除减速比后才是运动学所需轮输出轴速度。
        state_.wheel_vel[i] = motor_filtered[i] / cfg_.motor_ratio[i];
        if (dt > 0.0)
          state_.wheel_pos[i] += state_.wheel_vel[i] * dt;
      }
    }

    //============================================================================
    // 反馈处理
    //============================================================================

    /** 处理一帧异步反馈；发布反馈里程计、关节状态、IMU 和调试消息。 */
    void process_feedback(const FeedbackFrame &feedback)
    {
      // 计算时间间隔
      const auto now = Clock::now();
      const double dt = state_.have_prev_time ?
        std::chrono::duration<double>(now - state_.prev_time).count() : 0.0;
      state_.prev_time = now;
      state_.have_prev_time = true;

      // 断流恢复后的首帧不得与旧帧组成静止窗口；0.1 s 默认阈值远大于实测
      // 20~24 ms 帧间隔，同时小于 EKF 的 0.2 s sensor_timeout。
      const bool sample_contiguous =
        std::isfinite(dt) && dt > 0.0 && dt <= cfg_.imu_max_sample_gap;

      // 使用底盘绝对 yaw 的相邻变化检测“命令/电机都为零但机身被推动或抬起旋转”。
      // 这里只做粗门控，真正发布的航向仍由 YawDriftEstimator 完成展开与线性漂移补偿。
      double measured_yaw_rate = 0.0;
      const bool have_yaw_rate = sample_contiguous && state_.have_previous_feedback_yaw;
      if (have_yaw_rate)
      {
        const double yaw_delta = std::remainder(
          feedback.euler[2] - state_.previous_feedback_yaw, kTwoPi);
        measured_yaw_rate = std::abs(yaw_delta / dt);
      }
      state_.previous_feedback_yaw = feedback.euler[2];
      state_.have_previous_feedback_yaw = true;

      // 先完成符号和量程修正；协方差使用死区前测量，避免把被死区压成零的噪声
      // 错估为零方差。加速度安装方向未标定，但向量模长与安装姿态无关，可安全
      // 用于排除明显振动/碰撞样本，绝不把 ax/ay/az 送入当前 EKF。
      const Vector3 gyro_raw{feedback.gyro[0], feedback.gyro[1], feedback.gyro[2]};
      const auto gyro_corrected = correct_vector(
        gyro_raw, cfg_.imu_gyro_sign, {0.0, 0.0, 0.0}, cfg_.imu_gyro_scale);
      const double acceleration_norm = std::hypot(
        static_cast<double>(feedback.accel[0]),
        static_cast<double>(feedback.accel[1]),
        static_cast<double>(feedback.accel[2]));
      const bool inertial_stable =
        have_yaw_rate && std::isfinite(measured_yaw_rate) &&
        measured_yaw_rate <= cfg_.imu_stationary_yaw_rate_threshold &&
        std::isfinite(acceleration_norm) &&
        std::abs(acceleration_norm - kStandardGravityMps2) <=
          cfg_.imu_stationary_accel_tolerance;

      // 静止检测（用于 IMU 陀螺零漂和 yaw 漂移补偿）
      const bool cmd_zero =
        std::abs(state_.cmd.vx) < kStationaryVelocityEpsilon &&
        std::abs(state_.cmd.vy) < kStationaryVelocityEpsilon &&
        std::abs(state_.cmd.wz) < kStationaryVelocityEpsilon;
      const Vector3 motor_scaled{
        feedback.motor_actual[0] * cfg_.feedback_motor_scale,
        feedback.motor_actual[1] * cfg_.feedback_motor_scale,
        feedback.motor_actual[2] * cfg_.feedback_motor_scale};
      const auto motor_filtered = filter_motor_feedback(
        motor_scaled, dt, sample_contiguous);
      const double feedback_odom_dt = sample_contiguous ? dt : 0.0;
      // 断流恢复首帧只恢复速度，不跨越未知时间积分位移。
      update_feedback_wheel_state(motor_filtered, feedback_odom_dt);
      const bool wheel_zero = std::all_of(
        motor_filtered.begin(), motor_filtered.end(),
        [](double value) {return std::abs(value) < kStationaryVelocityEpsilon;});

      const bool stationary_evidence =
        cmd_zero && wheel_zero && sample_contiguous && inertial_stable;
      if (stationary_evidence && !state_.stationary_candidate)
      {
        state_.stationary_candidate = true;
        state_.stationary_since = now;
      }
      else if (!stationary_evidence)
      {
        state_.stationary_candidate = false;
      }

      const bool stationary = state_.stationary_candidate &&
                              std::chrono::duration<double>(now - state_.stationary_since).count() >=
                                  cfg_.imu_stationary_hold;

      // 【预处理 1】IMU 陀螺 → 角速度。
      const Vector3 gyro_processed = process_imu_gyro(gyro_corrected, stationary, dt);

      // 【预处理 2】IMU 姿态 → yaw 展开 + 漂移补偿
      const double yaw_unwrapped = yaw_drift_estimator_->update(
        feedback.euler[2], stationary, dt);
      if (cfg_.enable_yaw_drift_compensation && !state_.yaw_drift_logged &&
        yaw_drift_estimator_->calibrated())
      {
        const double drift_rate = yaw_drift_estimator_->drift_rate_rad_s();
        RCLCPP_INFO(
          get_logger(), "Yaw drift calibrated: %.6f rad/s (%.3f rad/hour)",
          drift_rate, drift_rate * 3600.0);
        state_.yaw_drift_logged = true;
      }

      covariance_estimator_->update(yaw_unwrapped, gyro_corrected[2], stationary, dt);
      if (covariance_estimator_->calibrated())
      {
        // 只有完整连续静止窗口能更新已提交值；EMA 避免协方差在窗口门槛处跳变。
        // 提交后立即开启非重叠新窗口，禁止同一批样本在后续每帧被重复加权。
        state_.committed_yaw_variance = smooth_variance(
          state_.committed_yaw_variance, covariance_estimator_->yaw_variance(),
          cfg_.covariance_update_alpha);
        state_.committed_gyro_z_variance = smooth_variance(
          state_.committed_gyro_z_variance, covariance_estimator_->gyro_z_variance(),
          cfg_.covariance_update_alpha);
        if (!state_.covariance_logged)
        {
          RCLCPP_INFO(
            get_logger(), "Stationary IMU variance calibration active: yaw=%.9g gyro_z=%.9g",
            state_.committed_yaw_variance, state_.committed_gyro_z_variance);
          state_.covariance_logged = true;
        }
        covariance_estimator_->begin_next_window();
      }

      // 三个估计器都完成首个连续静止窗口后才允许 SLAM 接收首帧。状态一旦
      // 就绪保持为 true；后续运动只暂停在线更新，不会使已经提交的可信标定失效。
      const bool yaw_ready = !cfg_.enable_yaw_drift_compensation ||
        yaw_drift_estimator_->calibrated();
      const bool imu_ready = gyro_filter_->calibrated() && yaw_ready &&
        state_.covariance_logged;
      if (imu_ready != state_.imu_ready)
      {
        state_.imu_ready = imu_ready;
        std_msgs::msg::Bool ready_message;
        ready_message.data = imu_ready;
        pub_imu_ready_->publish(ready_message);
        if (imu_ready)
        {
          RCLCPP_INFO(
            get_logger(), "IMU calibration ready; mapping may accept stable scans");
        }
      }

      const auto stamp = this->now();
      publish_odometry(stamp, feedback_odom_dt);
      publish_joint_states(stamp);
      publish_imu(
        feedback, gyro_processed, yaw_unwrapped, state_.committed_yaw_variance,
        state_.committed_gyro_z_variance, stamp);
      publish_debug(feedback, stamp);

      // 调试日志
      if (cfg_.log_frames)
      {
        RCLCPP_INFO(get_logger(),
                    "FB: motor=[%.2f %.2f %.2f] euler=[%.3f %.3f %.3f] "
                    "gyro_raw=[%.3f %.3f %.3f] gyro_ros=[%.3f %.3f %.3f]",
                    feedback.motor_actual[0], feedback.motor_actual[1], feedback.motor_actual[2],
                    feedback.euler[0], feedback.euler[1], feedback.euler[2],
                    feedback.gyro[0], feedback.gyro[1], feedback.gyro[2],
                    gyro_processed[0], gyro_processed[1], gyro_processed[2]);
      }
    }

    /** IMU 陀螺 → 角速度（零漂扣除 → 低通 → 死区）。 */
    Vector3 process_imu_gyro(const Vector3 & gyro_corrected, bool stationary, double dt)
    {
      // 零漂扣除 + 低通滤波
      const auto gyro_filtered = gyro_filter_->process(gyro_corrected, stationary, dt);

      // 日志零漂（仅首次）
      if (!state_.gyro_bias_logged && gyro_filter_->calibrated())
      {
        const auto &bias = gyro_filter_->bias();
        RCLCPP_INFO(get_logger(), "Gyro bias calibrated: [%.5f %.5f %.5f] rad/s",
                    bias[0], bias[1], bias[2]);
        state_.gyro_bias_logged = true;
      }

      // 死区过滤
      return correct_vector(gyro_filtered, {1.0, 1.0, 1.0}, cfg_.imu_gyro_deadband);
    }

    //============================================================================
    // 话题发布
    //============================================================================

    /** 发布轮式里程计（/chassis/odom/raw） */
    void publish_odometry(const rclcpp::Time &stamp, double dt)
    {
      // 正运动学
      const Twist2D raw_twist = kinematics_->forward(state_.wheel_vel);

      // 死区过滤
      const auto filtered = correct_vector(
          {raw_twist.vx, raw_twist.vy, raw_twist.wz},
          {1.0, 1.0, 1.0}, cfg_.wheel_twist_deadband);
      const Twist2D twist{filtered[0], filtered[1], filtered[2]};

      // 位姿积分（仅供调试，EKF 不融合位姿）
      if (dt > 0.0 && dt < 1.0)
      {
        const double c = std::cos(state_.odom_yaw);
        const double s = std::sin(state_.odom_yaw);
        state_.odom_x += (twist.vx * c - twist.vy * s) * dt;
        state_.odom_y += (twist.vx * s + twist.vy * c) * dt;
        state_.odom_yaw += twist.wz * dt;
      }

      // 构建消息
      nav_msgs::msg::Odometry msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cfg_.frame_odom;
      msg.child_frame_id = cfg_.frame_base;

      // 位姿（积分结果，仅供调试）
      msg.pose.pose.position.x = state_.odom_x;
      msg.pose.pose.position.y = state_.odom_y;
      msg.pose.pose.orientation.z = std::sin(state_.odom_yaw * 0.5);
      msg.pose.pose.orientation.w = std::cos(state_.odom_yaw * 0.5);
      msg.pose.covariance[0] = cfg_.wheel_pose_var[0];
      msg.pose.covariance[7] = cfg_.wheel_pose_var[1];
      msg.pose.covariance[35] = cfg_.wheel_pose_var[2];

      // 速度（EKF 融合此项）
      msg.twist.twist.linear.x = twist.vx;
      msg.twist.twist.linear.y = twist.vy;
      msg.twist.twist.angular.z = twist.wz;
      msg.twist.covariance[0] = cfg_.wheel_twist_var[0];
      msg.twist.covariance[7] = cfg_.wheel_twist_var[1];
      msg.twist.covariance[35] = cfg_.wheel_twist_var[2];

      pub_odom_->publish(msg);
    }

    /** 发布关节状态（/joint_states） */
    void publish_joint_states(const rclcpp::Time &stamp)
    {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = stamp;
      msg.name = cfg_.wheel_names;
      msg.position.assign(state_.wheel_pos.begin(), state_.wheel_pos.end());
      msg.velocity.assign(state_.wheel_vel.begin(), state_.wheel_vel.end());
      pub_joints_->publish(msg);
    }

    /** 发布底盘 IMU（/chassis/imu/data_raw） */
    void publish_imu(
        const FeedbackFrame &feedback,
        const Vector3 &gyro_ros,
        double yaw_unwrapped,
        double yaw_variance,
        double gyro_z_variance,
        const rclcpp::Time &stamp)
    {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cfg_.frame_imu;

      // 姿态（可选，取决于配置）
      if (cfg_.imu_publish_orientation)
      {
        const auto quat = euler_to_quat(feedback.euler[0], feedback.euler[1], yaw_unwrapped);
        msg.orientation.x = quat[0];
        msg.orientation.y = quat[1];
        msg.orientation.z = quat[2];
        msg.orientation.w = quat[3];
        msg.orientation_covariance[0] = cfg_.imu_orient_var[0];
        msg.orientation_covariance[4] = cfg_.imu_orient_var[1];
        msg.orientation_covariance[8] = yaw_variance;
      }
      else
      {
        msg.orientation_covariance[0] = -1.0; // 标记为不可用
      }

      // 角速度
      msg.angular_velocity.x = gyro_ros[0];
      msg.angular_velocity.y = gyro_ros[1];
      msg.angular_velocity.z = gyro_ros[2];
      msg.angular_velocity_covariance[0] = cfg_.imu_gyro_var[0];
      msg.angular_velocity_covariance[4] = cfg_.imu_gyro_var[1];
      msg.angular_velocity_covariance[8] = gyro_z_variance;

      // 加速度（原始值，未用于当前 EKF）
      msg.linear_acceleration.x = feedback.accel[0];
      msg.linear_acceleration.y = feedback.accel[1];
      msg.linear_acceleration.z = feedback.accel[2];
      msg.linear_acceleration_covariance[0] = cfg_.imu_accel_var[0];
      msg.linear_acceleration_covariance[4] = cfg_.imu_accel_var[1];
      msg.linear_acceleration_covariance[8] = cfg_.imu_accel_var[2];

      pub_imu_->publish(msg);
    }

    /** 发布调试数据（/chassis/feedback/raw） */
    void publish_debug(const FeedbackFrame &feedback, const rclcpp::Time &stamp)
    {
      kiwi_base::msg::FeedbackDebug msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = cfg_.frame_imu;

      msg.motor_1_actual_rad_s = feedback.motor_actual[0];
      msg.motor_2_actual_rad_s = feedback.motor_actual[1];
      msg.motor_3_actual_rad_s = feedback.motor_actual[2];

      msg.roll_rad = feedback.euler[0];
      msg.pitch_rad = feedback.euler[1];
      msg.yaw_rad = feedback.euler[2];

      msg.gyro_x_raw_rad_s = feedback.gyro[0];
      msg.gyro_y_raw_rad_s = feedback.gyro[1];
      msg.gyro_z_raw_rad_s = feedback.gyro[2];

      msg.accel_x_m_s2 = feedback.accel[0];
      msg.accel_y_m_s2 = feedback.accel[1];
      msg.accel_z_m_s2 = feedback.accel[2];

      pub_debug_->publish(msg);
    }

    //============================================================================
    // 成员变量
    //============================================================================

    Config cfg_;
    State state_;

    std::optional<KiwiKinematics> kinematics_;
    std::optional<GyroBiasFilter> gyro_filter_;
    std::optional<YawDriftEstimator> yaw_drift_estimator_;
    std::optional<StationaryCovarianceEstimator> covariance_estimator_;
    FrameParser parser_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_imu_ready_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
    rclcpp::Publisher<kiwi_base::msg::FeedbackDebug>::SharedPtr pub_debug_;
  };

} // namespace kiwi_base

//==============================================================================
// main 函数
//==============================================================================

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiwi_base::KiwiBaseNode>());
  rclcpp::shutdown();
  return 0;
}
