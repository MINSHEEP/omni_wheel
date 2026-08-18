/**
 * @file kiwi_base_node.cpp
 * @brief Kiwi 三轮全向底盘驱动节点
 *
 * 【数据流向】
 *   输入: /cmd_vel (Twist) → 速度限幅 → 逆运动学 → 电机指令 → USB CDC
 *   输出: USB CDC → 反馈解析 → 预处理 → 四个话题发布
 *
 * 【发布话题】
 *   /chassis/odom/raw      - 轮式里程计（位姿 + 速度）
 *   /chassis/imu/data_raw  - 底盘 IMU（姿态 + 角速度 + 加速度）
 *   /joint_states          - 三轮关节状态
 *   /chassis/feedback/raw  - 调试用原始反馈数据
 *
 * 【预处理链路】
 *   电机反馈: 倍率换算 → 死区过滤 → 除减速比 → 正运动学 → 里程计
 *   IMU 角速度: 符号修正 → 标定缩放 → 零漂扣除 → 低通滤波 → 死区过滤
 *   IMU 姿态: yaw 角度展开（处理 -π 到 π 跳变）→ 四元数转换
 */

#include "kiwi_base/kiwi_kinematics.hpp"
#include "kiwi_base/measurement_processing.hpp"
#include "kiwi_base/robomaster_protocol.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kiwi_base/msg/feedback_debug.hpp>

#include <fcntl.h>
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

    /** 读取正数参数 */
template<typename T>
T require_positive(rclcpp::Node &node, const std::string &name)
{
  const auto value = require_param<T>(node, name);
  if (!std::isfinite(value) || value <= 0.0)
    throw std::invalid_argument(name + " must be finite and positive");
  return value;
}

/** 读取非零向量参数 */
template<typename T>
std::array<T, 3> require_nonzero_vector3(rclcpp::Node &node, const std::string &name)
{
  const auto values = require_param<std::vector<T>>(node, name);
  if (values.size() != 3 || std::any_of(values.begin(), values.end(),
      [](T v) { return !std::isfinite(v) || v == 0.0; }))
    throw std::invalid_argument(name + " must be 3 finite non-zero values");
  return {values[0], values[1], values[2]};
}

/** 读取三轴方差参数（必须为正数） */
    std::array<double, 3> require_variance(rclcpp::Node &node, const std::string &name)
    {
      const auto values = require_param<std::vector<double>>(node, name);
      if (values.size() != 3 || std::any_of(values.begin(), values.end(),
                                            [](double v)
                                            { return !std::isfinite(v) || v <= 0.0; }))
        throw std::invalid_argument(name + " must be 3 finite positive values");
      return {values[0], values[1], values[2]};
    }

    /** 读取三轴向量参数（必须有限） */
    Vector3 require_vector3(rclcpp::Node &node, const std::string &name)
    {
      const auto values = require_param<std::vector<double>>(node, name);
      if (values.size() != 3 || std::any_of(values.begin(), values.end(),
                                            [](double v)
                                            { return !std::isfinite(v); }))
        throw std::invalid_argument(name + " must be 3 finite values");
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

  } // namespace

  //==============================================================================
  // KiwiBaseNode - 主节点类
  //==============================================================================

  /**
   * @brief Kiwi 底盘驱动节点
   *
   * 负责速度指令下发、反馈数据接收、预处理和话题发布。
   * 串口通信和数据处理都在固定周期定时器中完成。
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
      const auto period_ms = std::chrono::milliseconds(
          static_cast<int64_t>(1000.0 / cfg_.publish_rate));
      timer_ = create_wall_timer(period_ms, [this]()
                                 { update(); });

      RCLCPP_INFO(get_logger(), "Kiwi base node started (%.1f Hz)", cfg_.publish_rate);
    }

    ~KiwiBaseNode() override
    {
      // 退出时发送停止指令
      CommandFrame stop{};
      stop.enable = false;
      send_command(stop);
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

      // 机械参数
      std::array<double, 3> motor_ratio;
      double wheel_radius;
      double wheel_distance;

      // 速度限幅
      double max_vx;
      double max_vy;
      double max_wz;

      // 死区阈值
      Vector3 motor_deadband;
      Vector3 wheel_twist_deadband;
      Vector3 imu_gyro_deadband;

      // IMU 预处理
      Vector3 imu_gyro_sign;
      Vector3 imu_gyro_scale;
      double imu_stationary_hold;
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

      // 话题和坐标系
      std::string topic_cmd_vel;
      std::string topic_odom;
      std::string topic_imu;
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

      // 静止检测（用于陀螺零漂估计）
      bool stationary_candidate{false};
      Clock::time_point stationary_since;

      // yaw 角度展开（处理 -π 到 π 跳变）
      bool have_prev_yaw{false};
      double prev_yaw_raw{0.0};
      double yaw_offset{0.0};

      // yaw 零漂补偿
      bool yaw_drift_calibrated{false};
      size_t yaw_drift_sample_count{0};
      double yaw_drift_sum{0.0};
      double yaw_drift_rate{0.0};              // 估计的漂移率(rad/s)
      double yaw_drift_compensation{0.0};      // 累积的漂移补偿(rad)
      double yaw_drift_prev_unwrapped{0.0};    // 上一帧展开后的 yaw
      Clock::time_point yaw_drift_prev_time;   // 上一帧时间戳
      bool yaw_drift_have_prev{false};         // 是否有上一帧数据
      bool yaw_drift_logged{false};

      // cmd_vel 超时保护
      Clock::time_point last_cmd_time;
      bool have_cmd{false};

      // 其他
      bool gyro_bias_logged{false};
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

      // 机械参数
      const auto ratios = require_param<std::vector<double>>(*this, "motor_reduction_ratio");
      if (ratios.size() != 3)
        throw std::invalid_argument("motor_reduction_ratio must have 3 values");
      std::copy(ratios.begin(), ratios.end(), cfg_.motor_ratio.begin());

      cfg_.wheel_radius = require_positive<double>(*this, "wheel_radius_m");
      cfg_.wheel_distance = require_positive<double>(*this, "wheel_center_distance_m");

      // 速度限幅
      cfg_.max_vx = require_param<double>(*this, "max_linear_speed_m_s");
      cfg_.max_vy = require_positive<double>(*this, "max_lateral_speed_m_s");
      cfg_.max_wz = require_positive<double>(*this, "max_angular_speed_rad_s");

      // 死区
      cfg_.motor_deadband = require_vector3(*this, "motor_feedback_deadband_rad_s");
      cfg_.wheel_twist_deadband = require_vector3(*this, "wheel_twist_deadband");
      cfg_.imu_gyro_deadband = require_vector3(*this, "imu_angular_velocity_deadband_rad_s");

      // IMU 预处理
      cfg_.imu_gyro_sign = require_vector3(*this, "imu_angular_velocity_sign");
      cfg_.imu_gyro_scale = require_vector3(*this, "imu_angular_velocity_scale");
      cfg_.imu_stationary_hold = require_positive<double>(*this, "imu_stationary_hold_s");
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

      // 话题和坐标系
      cfg_.topic_cmd_vel = require_param<std::string>(*this, "cmd_vel_topic");
      cfg_.topic_odom = require_param<std::string>(*this, "wheel_odom_topic");
      cfg_.topic_imu = require_param<std::string>(*this, "imu_topic");
      cfg_.topic_joints = require_param<std::string>(*this, "joint_states_topic");
      cfg_.topic_debug = require_param<std::string>(*this, "feedback_debug_topic");
      cfg_.frame_odom = require_param<std::string>(*this, "odom_frame_id");
      cfg_.frame_base = require_param<std::string>(*this, "base_frame_id");
      cfg_.frame_imu = require_param<std::string>(*this, "imu_frame_id");
      cfg_.wheel_names = require_param<std::vector<std::string>>(*this, "wheel_names");

      if (cfg_.wheel_names.size() != 3)
        throw std::invalid_argument("wheel_names must have 3 entries");

      // 初始化运动学
      kinematics_.emplace(cfg_.wheel_radius, cfg_.wheel_distance);

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
      pub_joints_ = create_publisher<sensor_msgs::msg::JointState>(cfg_.topic_joints, 10);
      pub_debug_ = create_publisher<kiwi_base::msg::FeedbackDebug>(cfg_.topic_debug, 10);
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
      for (;;)
      {
        const ssize_t n = read(state_.fd, buffer, sizeof(buffer));

        if (n > 0)
        {
          // 根据配置选择帧解析或裸数据模式
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

        // 读取错误，关闭串口
        close_serial();
        return;
      }
    }

    /** 发送指令到串口 */
    void send_command(const CommandFrame &cmd)
    {
      if (state_.fd < 0)
        return;

      const auto frame = encode_command(cmd);
      const ssize_t written = write(state_.fd, frame.data(), frame.size());

      if (written != static_cast<ssize_t>(frame.size()))
        close_serial();
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
      read_serial();

      // 处理最新反馈帧
      if (state_.pending_feedback)
      {
        process_feedback(*state_.pending_feedback);
        state_.pending_feedback.reset();
      }

      // 发送电机指令
      const auto wheel_speed = kinematics_->inverse(state_.cmd);

      CommandFrame cmd{};
      cmd.enable = true;
      for (size_t i = 0; i < 3; ++i)
        cmd.motor_rad_s[i] = static_cast<float>(wheel_speed[i] * cfg_.motor_ratio[i]);

      send_command(cmd);

      // 调试日志
      if (cfg_.log_frames)
      {
        RCLCPP_INFO(get_logger(),
                    "CMD: vx=%.3f vy=%.3f wz=%.3f | WHEEL: %.2f %.2f %.2f | MOTOR: %.2f %.2f %.2f",
                    state_.cmd.vx, state_.cmd.vy, state_.cmd.wz,
                    wheel_speed[0], wheel_speed[1], wheel_speed[2],
                    cmd.motor_rad_s[0], cmd.motor_rad_s[1], cmd.motor_rad_s[2]);
      }
    }

    //============================================================================
    // 反馈处理
    //============================================================================

    /** 处理一帧反馈 - 预处理 + 发布四个话题 */
    void process_feedback(const FeedbackFrame &feedback)
    {
      // 计算时间间隔
      const auto now = Clock::now();
      const double dt = state_.have_prev_time ? std::chrono::duration<double>(now - state_.prev_time).count() : 0.0;
      state_.prev_time = now;
      state_.have_prev_time = true;

      // 【预处理 1】电机反馈 → 轮速
      process_motor_feedback(feedback, dt);

      // 静止检测（用于 IMU 陀螺零漂和 yaw 漂移补偿）
      const bool cmd_zero =
          state_.cmd.vx == 0.0 && state_.cmd.vy == 0.0 && state_.cmd.wz == 0.0;
      const bool wheel_zero =
          state_.wheel_vel[0] == 0.0 && state_.wheel_vel[1] == 0.0 && state_.wheel_vel[2] == 0.0;

      if (cmd_zero && wheel_zero && !state_.stationary_candidate)
      {
        state_.stationary_candidate = true;
        state_.stationary_since = now;
      }
      else if (!cmd_zero || !wheel_zero)
      {
        state_.stationary_candidate = false;
      }

      const bool stationary = state_.stationary_candidate &&
                              std::chrono::duration<double>(now - state_.stationary_since).count() >=
                                  cfg_.imu_stationary_hold;

      // 【预处理 2】IMU 陀螺 → 角速度
      const Vector3 gyro_processed = process_imu_gyro(feedback, stationary, dt);

      // 【预处理 3】IMU 姿态 → yaw 展开 + 漂移补偿
      const double yaw_unwrapped = unwrap_yaw(feedback.euler[2], stationary, dt);

      // 发布话题
      const auto stamp = this->now();
      publish_odometry(stamp, dt);
      publish_joint_states(stamp);
      publish_imu(feedback, gyro_processed, yaw_unwrapped, stamp);
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

    /** 【预处理 1】电机反馈 → 轮速 → 正运动学 → 里程计积分 */
    void process_motor_feedback(const FeedbackFrame &feedback, double dt)
    {
      // 倍率换算
      const Vector3 motor_scaled{
          feedback.motor_actual[0] * cfg_.feedback_motor_scale,
          feedback.motor_actual[1] * cfg_.feedback_motor_scale,
          feedback.motor_actual[2] * cfg_.feedback_motor_scale};

      // 死区过滤（电机轴）
      const auto motor_filtered = correct_vector(
          motor_scaled, {1.0, 1.0, 1.0}, cfg_.motor_deadband);

      // 除减速比得到轮速
      for (size_t i = 0; i < 3; ++i)
      {
        state_.wheel_vel[i] = motor_filtered[i] / cfg_.motor_ratio[i];
        if (dt > 0.0 && dt < 1.0)
          state_.wheel_pos[i] += state_.wheel_vel[i] * dt;
      }
    }

    /** 【预处理 2】IMU 陀螺 → 角速度（符号修正 → 零漂扣除 → 低通 → 死区） */
    Vector3 process_imu_gyro(const FeedbackFrame &feedback, bool stationary, double dt)
    {
      // 符号修正 + 标定缩放
      const Vector3 gyro_raw{feedback.gyro[0], feedback.gyro[1], feedback.gyro[2]};
      const auto gyro_corrected = correct_vector(
          gyro_raw, cfg_.imu_gyro_sign, {0.0, 0.0, 0.0}, cfg_.imu_gyro_scale);

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

  /** 【预处理 3】yaw 角度展开 - 处理 -π 到 π 的跳变 */
  double unwrap_yaw(double yaw_raw, bool stationary, double dt)
  {
    const auto now = Clock::now();

    // 步骤 1: 角度展开（处理跳变）
    if (state_.have_prev_yaw)
    {
      const double delta = yaw_raw - state_.prev_yaw_raw;

      // 检测跳变
      if (delta > M_PI)
        state_.yaw_offset -= 2.0 * M_PI;
      else if (delta < -M_PI)
        state_.yaw_offset += 2.0 * M_PI;
    }

    const double yaw_unwrapped = yaw_raw + state_.yaw_offset;

    // 步骤 2: yaw 零漂补偿（仅在启用时）
    if (!cfg_.enable_yaw_drift_compensation)
    {
      // 仅更新状态，不补偿
      state_.prev_yaw_raw = yaw_raw;
      state_.have_prev_yaw = true;
      return yaw_unwrapped;
    }

    // 初始化：在静止时采集样本估计初始漂移率
    if (!state_.yaw_drift_calibrated && stationary && state_.yaw_drift_have_prev && dt > 0.0)
    {
      // 计算真实的 yaw 增量（rad）
      const double yaw_delta = yaw_unwrapped - state_.yaw_drift_prev_unwrapped;
      const double time_elapsed = std::chrono::duration<double>(now - state_.yaw_drift_prev_time).count();

      if (time_elapsed > 0.0)
      {
        // 瞬时漂移率 = 增量 / 时间
        const double instant_drift = yaw_delta / time_elapsed;
        state_.yaw_drift_sum += instant_drift;
        state_.yaw_drift_sample_count++;

        if (state_.yaw_drift_sample_count >= cfg_.yaw_drift_calib_samples)
        {
          // 初始漂移率 = 采样均值
          state_.yaw_drift_rate = state_.yaw_drift_sum / state_.yaw_drift_sample_count;

          // 限制最大漂移率
          if (std::abs(state_.yaw_drift_rate) > cfg_.yaw_drift_max)
            state_.yaw_drift_rate = std::copysign(cfg_.yaw_drift_max, state_.yaw_drift_rate);

          state_.yaw_drift_calibrated = true;

          if (!state_.yaw_drift_logged)
          {
            RCLCPP_INFO(get_logger(),
                        "Yaw drift calibrated: %.6f rad/s (%.3f rad/hour)",
                        state_.yaw_drift_rate, state_.yaw_drift_rate * 3600.0);
            state_.yaw_drift_logged = true;
          }
        }
      }
    }

    // 运行时：持续更新漂移估计（仅在静止时）并补偿
    if (state_.yaw_drift_calibrated && state_.yaw_drift_have_prev && dt > 0.0)
    {
      const double yaw_delta = yaw_unwrapped - state_.yaw_drift_prev_unwrapped;
      const double time_elapsed = std::chrono::duration<double>(now - state_.yaw_drift_prev_time).count();

      // 在静止且时间间隔 > 1s 时更新漂移估计
      if (stationary && time_elapsed > 1.0)
      {
        const double observed_drift = yaw_delta / time_elapsed;

        // EMA 更新：alpha = min(time_elapsed / tau, 1.0)，避免 alpha > 1
        const double alpha = std::min(time_elapsed / cfg_.yaw_drift_tau, 1.0);
        state_.yaw_drift_rate = (1.0 - alpha) * state_.yaw_drift_rate + alpha * observed_drift;

        // 限制最大漂移率
        if (std::abs(state_.yaw_drift_rate) > cfg_.yaw_drift_max)
          state_.yaw_drift_rate = std::copysign(cfg_.yaw_drift_max, state_.yaw_drift_rate);

        // 重置参考点
        state_.yaw_drift_prev_unwrapped = yaw_unwrapped;
        state_.yaw_drift_prev_time = now;
      }

      // 累积漂移补偿（每帧都应用）
      state_.yaw_drift_compensation += state_.yaw_drift_rate * dt;
    }

    // 保存当前帧作为下一帧的参考
    if (!state_.yaw_drift_have_prev || stationary)
    {
      state_.yaw_drift_prev_unwrapped = yaw_unwrapped;
      state_.yaw_drift_prev_time = now;
      state_.yaw_drift_have_prev = true;
    }

    state_.prev_yaw_raw = yaw_raw;
    state_.have_prev_yaw = true;

    // 返回补偿后的 yaw
    return yaw_unwrapped - state_.yaw_drift_compensation;
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
        msg.orientation_covariance[8] = cfg_.imu_orient_var[2];
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
      msg.angular_velocity_covariance[8] = cfg_.imu_gyro_var[2];

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
    FrameParser parser_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
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
