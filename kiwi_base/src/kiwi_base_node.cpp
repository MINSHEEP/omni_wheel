#include "kiwi_base/kiwi_kinematics.hpp"
#include "kiwi_base/robomaster_protocol.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace kiwi_base
{
  namespace
  {
    using SteadyClock = std::chrono::steady_clock;
    constexpr uint16_t kStateEstop = 1U << 1;
    constexpr uint16_t kStateUsbOnline = 1U << 2;
    constexpr uint16_t kStateMotorOnline = 1U << 3;

    /** 对单个速度轴做有限值检查和硬限幅，非法输入按 0 处理。 */
    double clamp_finite(double value, double lower, double upper)
    {
      if (!std::isfinite(value))
        return 0.0;
      return std::clamp(value, lower, upper);
    }

    /** 根据当前值和目标值施加加/减速度限制，避免命令突变。 */
    double slew_axis(double current, double target, double dt, double accel, double decel)
    {
      if (!(dt > 0.0) || !std::isfinite(dt))
        return current;
      const double delta = target - current;
      const double rate = std::abs(target) > std::abs(current) ? accel : decel;
      const double step = std::max(0.0, rate) * dt;
      return current + std::clamp(delta, -step, step);
    }
  }

  class KiwiBaseNode final : public rclcpp::Node
  {
  public:
    /**
     * 创建底盘节点、读取安全参数、建立 ROS 话题并启动非阻塞 I/O 定时器。
     * 参数或几何不完整时节点仍可启动，但保持禁能，便于先检查诊断信息。
     */
    KiwiBaseNode()
        : Node("kiwi_base_node"),
          parser_([this](uint8_t version, FrameType type, const std::vector<uint8_t> &payload)
                  { on_frame(version, type, payload); })
    {
      device_ = declare_parameter<std::string>("device", "/dev/robomaster_c");
      transport_ = declare_parameter<std::string>("transport", "usb_cdc");
      protocol_version_ = declare_parameter<int>("protocol_version", kProtocolVersion);
      command_timeout_ms_ = static_cast<int>(declare_parameter<int>("command_timeout_ms", 100));
      feedback_timeout_ms_ = static_cast<int>(declare_parameter<int>("feedback_timeout_ms", 150));
      reconnect_interval_ms_ = static_cast<int>(declare_parameter<int>("reconnect_interval_ms", 1000));
      command_timeout_ms_ = std::max(1, command_timeout_ms_);
      feedback_timeout_ms_ = std::max(1, feedback_timeout_ms_);
      reconnect_interval_ms_ = std::max(100, reconnect_interval_ms_);
      publish_rate_ = std::max(1.0, declare_parameter<double>("publish_rate", 50.0));
      max_wheel_speed_ = declare_parameter<double>("max_wheel_speed_rad_s", 30.0);
      max_linear_speed_ = declare_parameter<double>("max_linear_speed_m_s", 0.6);
      max_lateral_speed_ = declare_parameter<double>("max_lateral_speed_m_s", 0.6);
      max_angular_speed_ = declare_parameter<double>("max_angular_speed_rad_s", 1.5);
      max_accel_ = declare_parameter<double>("max_accel_m_s2", 0.8);
      max_decel_ = declare_parameter<double>("max_decel_m_s2", 1.2);
      max_yaw_accel_ = declare_parameter<double>("max_yaw_accel_rad_s2", 2.0);
      max_yaw_decel_ = declare_parameter<double>("max_yaw_decel_rad_s2", 3.0);
      odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
      base_frame_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
      imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
      cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
      wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
      imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
      joint_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
      estop_topic_ = declare_parameter<std::string>("estop_topic", "/kiwi/estop");
      wheel_names_ = declare_parameter<std::vector<std::string>>(
          "wheel_names", {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"});

      const auto positions = declare_parameter<std::vector<double>>(
          "wheel_positions_m", std::vector<double>{});
      const auto angles = declare_parameter<std::vector<double>>(
          "wheel_tangent_angles_deg", std::vector<double>{});
      const auto radii = declare_parameter<std::vector<double>>(
          "wheel_radius_m", std::vector<double>{});
      const auto signs = declare_parameter<std::vector<int64_t>>(
          "wheel_sign", std::vector<int64_t>{1, 1, 1});
      const auto motor_ratios = declare_parameter<std::vector<double>>(
          "motor_reduction_ratio", std::vector<double>{19.0, 19.0, 19.0});

      bool motor_ratio_ok = motor_ratios.size() == 3;
      for (const double ratio : motor_ratios)
        motor_ratio_ok = motor_ratio_ok && std::isfinite(ratio) && ratio > 0.0;
      if (!motor_ratio_ok)
      {
        RCLCPP_ERROR(get_logger(), "motor_reduction_ratio must contain three positive finite values");
        motor_reduction_ratio_ = {19.0, 19.0, 19.0};
      }
      else
      {
        for (size_t i = 0; i < 3; ++i)
          motor_reduction_ratio_[i] = motor_ratios[i];
      }

      protocol_ok_ = protocol_version_ == static_cast<int>(kProtocolVersion);
      if (!protocol_ok_)
      {
        RCLCPP_ERROR(
            get_logger(), "protocol_version must be %u (0x%02x)",
            static_cast<unsigned>(kProtocolVersion), static_cast<unsigned>(kProtocolVersion));
      }
      if (!(max_wheel_speed_ > 0.0) || !std::isfinite(max_wheel_speed_))
      {
        RCLCPP_ERROR(get_logger(), "max_wheel_speed_rad_s must be positive and finite");
        max_wheel_speed_ = 30.0;
      }
      // 所有安全限值都必须是正数；错误配置回退到保守初值并保持诊断可见。
      max_linear_speed_ = std::isfinite(max_linear_speed_) && max_linear_speed_ > 0.0 ? max_linear_speed_ : 0.6;
      max_lateral_speed_ = std::isfinite(max_lateral_speed_) && max_lateral_speed_ > 0.0 ? max_lateral_speed_ : 0.6;
      max_angular_speed_ = std::isfinite(max_angular_speed_) && max_angular_speed_ > 0.0 ? max_angular_speed_ : 1.5;
      max_accel_ = std::isfinite(max_accel_) && max_accel_ > 0.0 ? max_accel_ : 0.8;
      max_decel_ = std::isfinite(max_decel_) && max_decel_ > 0.0 ? max_decel_ : 1.2;
      max_yaw_accel_ = std::isfinite(max_yaw_accel_) && max_yaw_accel_ > 0.0 ? max_yaw_accel_ : 2.0;
      max_yaw_decel_ = std::isfinite(max_yaw_decel_) && max_yaw_decel_ > 0.0 ? max_yaw_decel_ : 3.0;
      if (positions.size() == 6 && angles.size() == 3 && radii.size() == 3 &&
          signs.size() == 3 && wheel_names_.size() == 3)
      {
        std::array<WheelGeometry, 3> geometry{};
        for (size_t i = 0; i < 3; ++i)
        {
          geometry[i].x = positions[2 * i];
          geometry[i].y = positions[2 * i + 1];
          geometry[i].tangent_angle_rad = angles[i] * M_PI / 180.0;
          geometry[i].radius = radii[i];
          geometry[i].sign = static_cast<double>(signs[i]);
        }
        kinematics_ = std::make_unique<KiwiKinematics>(geometry);
        configured_ = kinematics_->valid() && protocol_ok_ && motor_ratio_ok;
        if (!kinematics_->valid())
        {
          RCLCPP_ERROR(get_logger(), "invalid Kiwi kinematics: %s", kinematics_->error().c_str());
        }
      }
      else
      {
        RCLCPP_ERROR(
            get_logger(),
            "wheel parameters are incomplete; node stays disabled until real geometry is configured");
      }

      wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 10);
      imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
      battery_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
      joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_topic_, 10);
      diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
      cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
          cmd_topic_, 10,
          [this](const geometry_msgs::msg::Twist::SharedPtr msg)
          {
            if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
                !std::isfinite(msg->angular.z))
            {
              last_cmd_ = {};
              have_cmd_ = false;
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "rejected non-finite cmd_vel");
              return;
            }
            last_cmd_ = {msg->linear.x, msg->linear.y, msg->angular.z};
            last_cmd_steady_ = SteadyClock::now();
            have_cmd_ = true;
          });
      // 软件急停只允许把输出置零；解除急停后仍需收到新鲜 cmd_vel 才能恢复。
      estop_sub_ = create_subscription<std_msgs::msg::Bool>(
          estop_topic_, 10,
          [this](const std_msgs::msg::Bool::SharedPtr msg)
          {
            host_estop_ = msg->data;
            if (host_estop_)
            {
              // 急停期间丢弃旧命令；解除后必须收到新的 cmd_vel 才能恢复。
              have_cmd_ = false;
              last_cmd_ = {};
              limited_command_ = {};
              have_limited_command_ = false;
            }
          });

      if (transport_ == "usb_cdc")
      {
        open_serial();
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "unsupported transport '%s'; only usb_cdc is implemented", transport_.c_str());
      }

      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / publish_rate_));
      timer_ = create_wall_timer(std::max(std::chrono::milliseconds(1), period), [this]()
                                 { update(); });
    }

    /** 发送一次禁能帧后关闭设备，避免节点退出时留下最后一条运动命令。 */
    ~KiwiBaseNode() override
    {
      CommandFrame stop{};
      stop.seq = static_cast<uint16_t>(tx_seq_++);
      send_command(stop);
      close_serial();
    }

  private:
    /** 打开 USB CDC 字符设备并配置为 raw、非阻塞模式；失败由 update() 重试。 */
    void open_serial()
    {
      last_open_attempt_ = SteadyClock::now();
      if (fd_ >= 0 || transport_ != "usb_cdc")
        return;

      fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
      if (fd_ < 0)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000, "cannot open %s: %s", device_.c_str(), std::strerror(errno));
        return;
      }

      struct termios tio{};
      // CDC 虚拟串口没有实际波特率，但保持 raw 配置以避免行规程改写二进制帧。
      if (tcgetattr(fd_, &tio) != 0)
      {
        RCLCPP_WARN(get_logger(), "cannot read terminal attributes for %s: %s", device_.c_str(), std::strerror(errno));
        close_serial();
        return;
      }
      cfmakeraw(&tio);
      tio.c_cflag |= CLOCAL | CREAD;
      if (tcsetattr(fd_, TCSANOW, &tio) != 0)
      {
        RCLCPP_WARN(get_logger(), "cannot configure %s: %s", device_.c_str(), std::strerror(errno));
        close_serial();
        return;
      }
      RCLCPP_INFO(get_logger(), "opened USB CDC device %s", device_.c_str());
    }

    /** 关闭文件描述符并清除在线状态、半帧缓存和反馈门控状态。 */
    void close_serial()
    {
      if (fd_ >= 0)
        close(fd_);
      fd_ = -1;
      online_ = false;
      mcu_ready_ = false;
      wheel_state_seen_ = false;
      parser_.reset();
    }

    /** 非阻塞读取当前可用字节，交给解析器处理；检测到拔线立即进入安全关闭。 */
    void read_serial()
    {
      if (fd_ < 0)
        return;
      struct pollfd descriptor{fd_, POLLIN, 0};
      const int poll_result = poll(&descriptor, 1, 0);
      if (poll_result < 0 && errno != EINTR)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "USB poll failed; reconnecting");
        close_serial();
        return;
      }
      if (poll_result <= 0)
        return;
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "USB disconnected; reconnecting");
        close_serial();
        return;
      }
      if ((descriptor.revents & POLLIN) == 0)
        return;

      uint8_t buffer[512];
      for (;;)
      {
        const ssize_t count = read(fd_, buffer, sizeof(buffer));
        if (count > 0)
        {
          parser_.feed(buffer, static_cast<size_t>(count));
          continue;
        }
        if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)))
          return;
        if (count < 0 && errno == EINTR)
          continue;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "USB read failed; reconnecting");
        close_serial();
        return;
      }
    }

    /**
     * 对机体速度先做绝对限幅，再做加速度斜坡限制。
     * 急停或命令超时由调用方传入零目标；host_estop_ 时直接清零，保证
     * 软件急停不等待减速度斜坡。时间计算使用 steady_clock，避免系统时间跳变。
     */
    Twist2D limit_command(const Twist2D &target, SteadyClock::time_point now)
    {
      Twist2D bounded{
        clamp_finite(target.vx, -max_linear_speed_, max_linear_speed_),
        clamp_finite(target.vy, -max_lateral_speed_, max_lateral_speed_),
        clamp_finite(target.wz, -max_angular_speed_, max_angular_speed_)};
      if (host_estop_)
      {
        limited_command_ = {};
        have_limited_command_ = false;
        last_limit_steady_ = now;
        return {};
      }
      if (!have_limited_command_)
      {
        limited_command_ = {};
        have_limited_command_ = true;
        last_limit_steady_ = now;
        return limited_command_;
      }
      const double dt = std::clamp(
        std::chrono::duration<double>(now - last_limit_steady_).count(), 0.0, 0.1);
      limited_command_.vx = slew_axis(limited_command_.vx, bounded.vx, dt, max_accel_, max_decel_);
      limited_command_.vy = slew_axis(limited_command_.vy, bounded.vy, dt, max_accel_, max_decel_);
      limited_command_.wz = slew_axis(
        limited_command_.wz, bounded.wz, dt, max_yaw_accel_, max_yaw_decel_);
      last_limit_steady_ = now;
      return limited_command_;
    }

    /** 定时执行安全门控、逆运动学、限幅、USB 命令发送和诊断发布。 */
    void update()
    {
      const auto steady_now = SteadyClock::now();
      if (fd_ < 0 && transport_ == "usb_cdc" &&
          std::chrono::duration_cast<std::chrono::milliseconds>(steady_now - last_open_attempt_).count() >=
              reconnect_interval_ms_)
      {
        open_serial();
      }
      read_serial();

      const double command_age_ms = have_cmd_ ? std::chrono::duration<double, std::milli>(steady_now - last_cmd_steady_).count() : std::numeric_limits<double>::infinity();
      const bool command_fresh = have_cmd_ && command_age_ms <= command_timeout_ms_;
      const bool feedback_fresh = wheel_state_seen_ && online_ &&
                                  std::chrono::duration_cast<std::chrono::milliseconds>(steady_now - last_state_steady_).count() <=
                                      feedback_timeout_ms_;
      if (!feedback_fresh)
      {
        online_ = false;
        mcu_ready_ = false;
        wheel_state_seen_ = false;
      }

      const Twist2D requested = command_fresh && !host_estop_ ? last_cmd_ : Twist2D{};
      const Twist2D command = limit_command(requested, steady_now);
      const bool enabled = configured_ && fd_ >= 0 && feedback_fresh && command_fresh &&
                           mcu_ready_ && !estop_ && !host_estop_;
      auto wheels = kinematics_ ? kinematics_->inverse(command) : std::array<double, 3>{};
      double scale = 1.0;
      for (const double wheel : wheels)
      {
        scale = std::max(scale, std::abs(wheel) / std::max(1e-6, max_wheel_speed_));
      }
      for (double &wheel : wheels)
        wheel /= scale;
      // 未通过全部在线/新鲜/急停门控时，帧内目标也必须为零，避免下位机误解残留速度。
      if (!enabled)
        wheels = {};

      CommandFrame frame{};
      frame.seq = static_cast<uint16_t>(tx_seq_++);
      frame.timestamp_ms = static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::milliseconds>(steady_now.time_since_epoch()).count());
      frame.enable = enabled;
      for (size_t i = 0; i < 3; ++i)
      {
        // 运动学输出是轮输出轴 rad/s；联调帧要求电机轴 rad/s，减速比只在主机转换一次。
        frame.motor_rad_s[i] = static_cast<float>(wheels[i] * motor_reduction_ratio_[i]);
      }
      send_command(frame);
      publish_diagnostics(command_age_ms, feedback_fresh, command_fresh);
    }

    /** 将一条已完成安全门控的三电机速度命令写入 USB；部分写入视为链路故障。 */
    void send_command(const CommandFrame &command)
    {
      if (fd_ < 0)
        return;
      const auto bytes = encode_command(command);
      const ssize_t count = write(fd_, bytes.data(), bytes.size());
      if (count == static_cast<ssize_t>(bytes.size()))
        return;
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
        return;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "USB write failed or was partial; reconnecting");
      close_serial();
    }

    /** 分发一帧反馈：轮速更新里程计/关节，IMU 更新传感器，电源更新电池。 */
    void on_frame(uint8_t version, FrameType type, const std::vector<uint8_t> &payload)
    {
      if (version != protocol_version_)
        return;
      if (type == FrameType::WheelState)
      {
        WheelStateFrame state;
        if (!decode_wheel_state(payload, state))
          return;
        wheel_state_seen_ = true;
        online_ = true;
        last_state_steady_ = SteadyClock::now();
        estop_ = (state.flags & kStateEstop) != 0U;
        fault_bits_ = state.fault_bits;
        mcu_flags_ = state.flags;
        mcu_ready_ = (state.flags & kStateUsbOnline) != 0U &&
                     (state.flags & kStateMotorOnline) != 0U && state.fault_bits == 0U;
        wheel_pos_ = state.position;
        wheel_vel_ = mcu_ready_ ? state.velocity : std::array<float, 3>{};
        publish_odom();
        publish_joints();
        PowerStateFrame power{};
        power.battery_mv = state.battery_mv;
        power.flags = state.flags;
        publish_battery(power);
      }
      else if (type == FrameType::ImuState)
      {
        ImuStateFrame state;
        if (decode_imu_state(payload, state))
          publish_imu(state);
      }
      else if (type == FrameType::PowerState)
      {
        PowerStateFrame state;
        if (!decode_power_state(payload, state))
          return;
        estop_ = (state.flags & kStateEstop) != 0U;
        mcu_flags_ = state.flags;
        publish_battery(state);
      }
    }

    /** 用实测轮速做平面积分并发布原始轮里程计；不发布 odom TF。 */
    void publish_odom()
    {
      if (!kinematics_)
        return;
      const auto steady_stamp = SteadyClock::now();
      const auto twist = kinematics_->forward({wheel_vel_[0], wheel_vel_[1], wheel_vel_[2]});
      if (have_odom_steady_)
      {
        const double dt = std::chrono::duration<double>(steady_stamp - last_odom_steady_).count();
        if (dt > 0.0 && dt < 1.0)
        {
          const double c = std::cos(odom_yaw_);
          const double s = std::sin(odom_yaw_);
          odom_x_ += (twist.vx * c - twist.vy * s) * dt;
          odom_y_ += (twist.vx * s + twist.vy * c) * dt;
          odom_yaw_ += twist.wz * dt;
        }
      }
      last_odom_steady_ = steady_stamp;
      have_odom_steady_ = true;

      nav_msgs::msg::Odometry message;
      message.header.stamp = now();
      message.header.frame_id = odom_frame_;
      message.child_frame_id = base_frame_;
      message.pose.pose.position.x = odom_x_;
      message.pose.pose.position.y = odom_y_;
      message.pose.pose.orientation.z = std::sin(odom_yaw_ / 2.0);
      message.pose.pose.orientation.w = std::cos(odom_yaw_ / 2.0);
      message.twist.twist.linear.x = twist.vx;
      message.twist.twist.linear.y = twist.vy;
      message.twist.twist.angular.z = twist.wz;
      /* 未完成实车标定前给出保守协方差，避免 EKF 将轮速视为绝对真值。 */
      message.pose.covariance[0] = 0.02;
      message.pose.covariance[7] = 0.02;
      message.pose.covariance[35] = 0.10;
      message.twist.covariance[0] = 0.02;
      message.twist.covariance[7] = 0.02;
      message.twist.covariance[35] = 0.10;
      wheel_odom_pub_->publish(message);
    }

    /** 发布三个轮子的累计位置和输出轴角速度，供 robot_state_publisher/调试使用。 */
    void publish_joints()
    {
      sensor_msgs::msg::JointState message;
      message.header.stamp = now();
      message.name = wheel_names_;
      message.position.assign(wheel_pos_.begin(), wheel_pos_.end());
      message.velocity.assign(wheel_vel_.begin(), wheel_vel_.end());
      joint_pub_->publish(message);
    }

    /** 将 C 板 IMU 快照转换成 sensor_msgs/msg/Imu；姿态未知时明确标为无效。 */
    void publish_imu(const ImuStateFrame &state)
    {
      sensor_msgs::msg::Imu message;
      message.header.stamp = now();
      message.header.frame_id = imu_frame_;
      message.angular_velocity.x = state.gyro[0];
      message.angular_velocity.y = state.gyro[1];
      message.angular_velocity.z = state.gyro[2];
      message.linear_acceleration.x = state.accel[0];
      message.linear_acceleration.y = state.accel[1];
      message.linear_acceleration.z = state.accel[2];
      message.orientation_covariance[0] = -1.0;
      imu_pub_->publish(message);
    }

    /** 发布电池电压；电压字段来自反馈帧，present 仅表示收到有效快照。 */
    void publish_battery(const PowerStateFrame &state)
    {
      sensor_msgs::msg::BatteryState message;
      message.header.stamp = now();
      message.voltage = static_cast<float>(state.battery_mv) / 1000.0F;
      message.present = true;
      battery_pub_->publish(message);
    }

    /** 按配置、USB、反馈、急停和命令新鲜度的优先级发布诊断状态。 */
    void publish_diagnostics(double command_age_ms, bool feedback_fresh, bool command_fresh)
    {
      diagnostic_msgs::msg::DiagnosticArray array;
      array.header.stamp = now();
      diagnostic_msgs::msg::DiagnosticStatus status;
      status.name = "kiwi_base";
      status.hardware_id = device_;
      if (host_estop_ || !configured_ || fd_ < 0 || !feedback_fresh || !mcu_ready_ || estop_ || fault_bits_ != 0U)
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        status.message = host_estop_ ? "host software emergency stop" :
          (!configured_ ? "invalid configuration" : (fd_ < 0 ? "USB disconnected" :
          (!feedback_fresh ? "feedback timeout" : (!mcu_ready_ ? "MCU not ready" :
          (estop_ ? "MCU emergency stop" : "MCU motor fault")))));
      }
      else if (!command_fresh)
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        status.message = "waiting for fresh command";
      }
      else
      {
        status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = "running";
      }
      auto add_value = [&status](const std::string &key, const std::string &value)
      {
        diagnostic_msgs::msg::KeyValue item;
        item.key = key;
        item.value = value;
        status.values.push_back(item);
      };
      add_value("command_age_ms", std::isfinite(command_age_ms) ? std::to_string(command_age_ms) : "inf");
      add_value("mcu_flags", std::to_string(mcu_flags_));
      add_value("fault_bits", std::to_string(fault_bits_));
      add_value("host_estop", host_estop_ ? "true" : "false");
      add_value("command_limited", have_limited_command_ ? "true" : "false");
      add_value("crc_errors", std::to_string(parser_.crc_errors()));
      add_value("dropped_bytes", std::to_string(parser_.dropped_bytes()));
      array.status.push_back(status);
      diagnostics_pub_->publish(array);
    }

    std::string device_;
    std::string transport_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;
    std::string cmd_topic_;
    std::string wheel_odom_topic_;
    std::string imu_topic_;
    std::string joint_topic_;
    std::string estop_topic_;
    int protocol_version_{kProtocolVersion};
    int command_timeout_ms_{100};
    int feedback_timeout_ms_{150};
    int reconnect_interval_ms_{1000};
    int fd_{-1};
    double publish_rate_{50.0};
    double max_wheel_speed_{30.0};
    double max_linear_speed_{0.6};
    double max_lateral_speed_{0.6};
    double max_angular_speed_{1.5};
    double max_accel_{0.8};
    double max_decel_{1.2};
    double max_yaw_accel_{2.0};
    double max_yaw_decel_{3.0};
    bool protocol_ok_{false};
    bool configured_{false};
    bool online_{false};
    bool mcu_ready_{false};
    bool wheel_state_seen_{false};
    bool estop_{false};
    bool host_estop_{false};
    bool have_cmd_{false};
    bool have_odom_steady_{false};
    bool have_limited_command_{false};
    uint16_t mcu_flags_{0};
    uint32_t fault_bits_{0};
    std::vector<std::string> wheel_names_;
    std::array<double, 3> motor_reduction_ratio_{{19.0, 19.0, 19.0}};
    std::unique_ptr<KiwiKinematics> kinematics_;
    FrameParser parser_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
    Twist2D last_cmd_{};
    Twist2D limited_command_{};
    std::array<float, 3> wheel_pos_{};
    std::array<float, 3> wheel_vel_{};
    uint32_t tx_seq_{0};
    double odom_x_{0.0};
    double odom_y_{0.0};
    double odom_yaw_{0.0};
    SteadyClock::time_point last_cmd_steady_{};
    SteadyClock::time_point last_state_steady_{};
    SteadyClock::time_point last_open_attempt_{};
    SteadyClock::time_point last_odom_steady_{};
    SteadyClock::time_point last_limit_steady_{};
  };
} // namespace kiwi_base

/** ROS 2 进程入口：初始化、运行底盘节点并在退出时触发安全析构。 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiwi_base::KiwiBaseNode>());
  rclcpp::shutdown();
  return 0;
}
