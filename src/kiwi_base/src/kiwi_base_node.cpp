/**
 * @file kiwi_base_node.cpp
 * @brief Kiwi 三轮底盘最小联调节点。
 *
 * 运行链路只有四步：
 *   /cmd_vel -> 三轴限幅 -> 三行物理公式 -> USB 0x3A -> C 板反馈话题。
 * 加速度斜坡、急停和反馈联锁暂不放入运行路径。
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

/** 把原始 USB 帧转换成便于联调复制的十六进制文本。 */
std::string to_hex(const std::vector<uint8_t> &frame)
{
  constexpr char digits[] = "0123456789abcdef";
  std::string text;
  for (size_t i = 0; i < frame.size(); ++i)
  {
    if (i != 0)
      text += ' ';
    text += digits[frame[i] >> 4];
    text += digits[frame[i] & 0x0f];
  }
  return text;
}

/** 将 C 板的 roll/pitch/yaw（弧度）转换为 ROS 四元数。 */
std::array<double, 4> rpy_to_quaternion(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy};
}

/** 声明必须由 YAML 提供的参数；缺少配置时由 rclcpp 直接终止启动。 */
template<typename T>
T declare_required(rclcpp::Node &node, const std::string &name)
{
  return node.declare_parameter<T>(name);
}

/** 读取三个轴的方差；方差必须是有限正数，避免滤波器把无效配置当成真值。 */
std::array<double, 3> declare_variance(rclcpp::Node &node, const std::string &name)
{
  const auto values = declare_required<std::vector<double>>(node, name);
  if (values.size() != 3 || std::any_of(values.begin(), values.end(), [](double value)
    { return !std::isfinite(value) || value <= 0.0; }))
    throw std::invalid_argument(name + " must contain three finite positive variances");
  return {values[0], values[1], values[2]};
}

/** 读取有限三轴参数；符号和死区的具体范围由调用处检查。 */
Vector3 declare_vector3(rclcpp::Node &node, const std::string &name)
{
  const auto values = declare_required<std::vector<double>>(node, name);
  if (values.size() != 3 || std::any_of(values.begin(), values.end(), [](double value)
    { return !std::isfinite(value); }))
    throw std::invalid_argument(name + " must contain three finite values");
  return {values[0], values[1], values[2]};
}
}  // namespace

/**
 * @brief USB CDC 与 ROS 之间的最小适配节点。
 *
 * 主机只负责目标速度转换和协议收发，C 板负责 M3508 速度 PID 和 CAN 输出。
 */
class KiwiBaseNode final : public rclcpp::Node
{
public:
  /** 读取参数、建立话题、打开串口并启动固定周期发送定时器。 */
  KiwiBaseNode()
  : Node("kiwi_base_node"),
    parser_(
      [this](uint8_t version, const std::vector<uint8_t> &payload)
      { queue_feedback(version, payload); }
      // 暂停原因：解码后的结构化日志已足够联调；需要检查 CRC 原始字节时再恢复此回调。
      // ,[this](const std::vector<uint8_t> &frame){ log_frame("RX", frame); }
    )
  {
    // 运行参数没有代码回退值，统一从 kiwi_base.yaml 读取，缺项立即报错。
    device_ = declare_required<std::string>(*this, "device");
    publish_rate_ = declare_required<double>(*this, "publish_rate");
    log_serial_frames_ = declare_required<bool>(*this, "log_serial_frames");
    // 现场 C 板只输出 48 字节反馈 payload，没有 0x33/0x3B/CRC 封装。
    feedback_framed_ = declare_required<bool>(*this, "feedback_framed");
    feedback_scale_ = declare_required<double>(*this, "feedback_motor_scale_to_rad_s");
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0 ||
        !std::isfinite(feedback_scale_) || feedback_scale_ <= 0.0)
      throw std::invalid_argument("publish rate and feedback scale must be finite and positive");

    const auto ratios = declare_required<std::vector<double>>(*this, "motor_reduction_ratio");
    if (ratios.size() != 3 || std::any_of(ratios.begin(), ratios.end(), [](double value)
      { return !std::isfinite(value) || value <= 0.0; }))
      throw std::invalid_argument("motor reduction ratio must contain three finite positive values");
    for (size_t i = 0; i < 3; ++i)
      motor_ratio_[i] = ratios[i];
    motor_feedback_deadband_rad_s_ = declare_vector3(
      *this, "motor_feedback_deadband_rad_s");

    // 运动学只从 kiwi_base.yaml 读取轮半径和中心到轮心距离；URDF/Nav2 的外形参数单独维护。
    const double wheel_radius_m = declare_required<double>(*this, "wheel_radius_m");
    const double wheel_center_distance_m =
      declare_required<double>(*this, "wheel_center_distance_m");
    kinematics_.emplace(wheel_radius_m, wheel_center_distance_m);

    // 三个 YAML 参数分别限制前后、左右和旋转速度，单位与 Twist 一致。
    command_limits_ = {
      declare_required<double>(*this, "max_linear_speed_m_s"),
      declare_required<double>(*this, "max_lateral_speed_m_s"),
      declare_required<double>(*this, "max_angular_speed_rad_s")};
    if (!std::isfinite(command_limits_.vx) || command_limits_.vx <= 0.0 ||
        !std::isfinite(command_limits_.vy) || command_limits_.vy <= 0.0 ||
        !std::isfinite(command_limits_.wz) || command_limits_.wz <= 0.0)
      throw std::invalid_argument("velocity limits must be finite and positive");

    // 这些是 EKF 的测量噪声初值，数组顺序均为 x、y、z；实测静止数据后只改 YAML。
    wheel_pose_variance_ = declare_variance(*this, "wheel_pose_variance");
    wheel_twist_variance_ = declare_variance(*this, "wheel_twist_variance");
    imu_orientation_variance_ = declare_variance(*this, "imu_orientation_variance");
    // C 板绝对欧拉角在静止时仍会漂移；默认不把它伪装成有效 ROS 姿态。
    imu_publish_orientation_ = declare_required<bool>(*this, "imu_publish_orientation");
    imu_angular_velocity_variance_ = declare_variance(*this, "imu_angular_velocity_variance");
    imu_linear_acceleration_variance_ = declare_variance(*this, "imu_linear_acceleration_variance");

    // 下位机 gyro_z 为逆时针负；按 ROS 右手系翻转 Z，并过滤实测静止噪声。
    imu_angular_velocity_sign_ = declare_vector3(*this, "imu_angular_velocity_sign");
    // 对已确认单位正确但量程偏小的 IMU 轴做独立比例修正，不影响轮速里程计。
    imu_angular_velocity_scale_ = declare_vector3(*this, "imu_angular_velocity_scale");
    imu_angular_velocity_deadband_ = declare_vector3(
      *this, "imu_angular_velocity_deadband_rad_s");
    wheel_twist_deadband_ = declare_vector3(*this, "wheel_twist_deadband");

    // 陀螺零漂/低通处理参数：标定样本数、EMA 时间常数、零漂上限、截止频率、静止确认时长。
    const auto calibration_samples =
      declare_required<int64_t>(*this, "imu_gyro_calibration_samples");
    GyroFilterConfig gyro_config{};
    gyro_config.bias_time_constant_s =
      declare_required<double>(*this, "imu_gyro_bias_time_constant_s");
    gyro_config.max_bias_rad_s = declare_required<double>(*this, "imu_gyro_max_bias_rad_s");
    gyro_config.lowpass_cutoff_hz =
      declare_required<double>(*this, "imu_gyro_lowpass_cutoff_hz");
    imu_stationary_hold_s_ = declare_required<double>(*this, "imu_stationary_hold_s");
    if (calibration_samples < 0 ||
        !std::isfinite(gyro_config.bias_time_constant_s) || gyro_config.bias_time_constant_s < 0.0 ||
        !std::isfinite(gyro_config.max_bias_rad_s) || gyro_config.max_bias_rad_s < 0.0 ||
        !std::isfinite(gyro_config.lowpass_cutoff_hz) || gyro_config.lowpass_cutoff_hz < 0.0 ||
        !std::isfinite(imu_stationary_hold_s_) || imu_stationary_hold_s_ < 0.0)
      throw std::invalid_argument("imu gyro filter parameters must be finite and non-negative");
    gyro_config.calibration_samples = static_cast<std::size_t>(calibration_samples);
    gyro_filter_.emplace(gyro_config);
    if (std::any_of(
        imu_angular_velocity_sign_.begin(), imu_angular_velocity_sign_.end(),
        [](double value) { return std::abs(value) != 1.0; }))
      throw std::invalid_argument("imu angular velocity signs must be +1 or -1");
    const auto negative_deadband = [](double value) { return value < 0.0; };
    if (std::any_of(
        imu_angular_velocity_scale_.begin(), imu_angular_velocity_scale_.end(),
        [](double value) { return value <= 0.0; }))
      throw std::invalid_argument("imu angular velocity scales must be positive");
    if (std::any_of(
        imu_angular_velocity_deadband_.begin(), imu_angular_velocity_deadband_.end(),
        negative_deadband) ||
        std::any_of(
          motor_feedback_deadband_rad_s_.begin(), motor_feedback_deadband_rad_s_.end(),
          negative_deadband) ||
        std::any_of(wheel_twist_deadband_.begin(), wheel_twist_deadband_.end(), negative_deadband))
      throw std::invalid_argument("measurement deadbands must be non-negative");

    cmd_topic_ = declare_required<std::string>(*this, "cmd_vel_topic");
    wheel_odom_topic_ = declare_required<std::string>(*this, "wheel_odom_topic");
    imu_topic_ = declare_required<std::string>(*this, "imu_topic");
    feedback_debug_topic_ = declare_required<std::string>(*this, "feedback_debug_topic");
    joint_topic_ = declare_required<std::string>(*this, "joint_states_topic");
    odom_frame_ = declare_required<std::string>(*this, "odom_frame_id");
    base_frame_ = declare_required<std::string>(*this, "base_frame_id");
    imu_frame_ = declare_required<std::string>(*this, "imu_frame_id");
    wheel_names_ = declare_required<std::vector<std::string>>(*this, "wheel_names");
    if (wheel_names_.size() != 3)
      throw std::invalid_argument("wheel_names must contain exactly three joints");

    wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    feedback_debug_pub_ = create_publisher<kiwi_base::msg::FeedbackDebug>(feedback_debug_topic_, 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_topic_, 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        // 唯一限幅入口：此后的运动学和 USB 只接触限幅后的目标速度。
        command_ = limit_twist(
          {msg->linear.x, msg->linear.y, msg->angular.z}, command_limits_);
      });

    open_serial();
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    timer_ = create_wall_timer(std::max(std::chrono::milliseconds(1), period), [this]() { update(); });
  }

  /** 退出时发送一次零速帧，随后关闭 USB 文件描述符。 */
  ~KiwiBaseNode() override
  {
    CommandFrame stop{};
    stop.enable = false;
    send_command(stop);
    close_serial();
  }

private:
  /** 打开 USB CDC；失败时保留 fd=-1，后续定时器会再次尝试。 */
  void open_serial()
  {
    if (fd_ >= 0)
      return;
    fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0)
      return;

    termios attributes{};
    if (tcgetattr(fd_, &attributes) != 0)
    {
      close_serial();
      return;
    }
    cfmakeraw(&attributes);
    attributes.c_cflag |= CLOCAL | CREAD;
    if (tcsetattr(fd_, TCSANOW, &attributes) != 0)
    {
      close_serial();
      return;
    }
    RCLCPP_INFO(get_logger(), "USB CDC opened: %s", device_.c_str());
  }

  /** 关闭 USB 并清空协议半帧缓存。 */
  void close_serial()
  {
    if (fd_ >= 0)
      close(fd_);
    fd_ = -1;
    parser_.reset();
    raw_feedback_buffer_.clear();
  }

  /** 读取当前可用的 USB 字节，按配置选择严格帧或无帧 payload 解码。 */
  void read_serial()
  {
    if (fd_ < 0)
    {
      open_serial();
      return;
    }

    uint8_t buffer[512];
    for (;;)
    {
      const ssize_t count = read(fd_, buffer, sizeof(buffer));
      if (count > 0)
      {
        if (feedback_framed_)
          parser_.feed(buffer, static_cast<size_t>(count));
        else
          queue_raw_feedback(buffer, static_cast<size_t>(count));
        continue;
      }
      if (count == 0 || (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)))
        return;
      if (count < 0 && errno == EINTR)
        continue;
      close_serial();
      return;
    }
  }

  /** 发送主循环：直接公式 -> 减速比 -> 0x3A -> USB。 */
  void update()
  {
    read_serial();
    // 串口可能一次读出多个反馈帧；只处理最新一帧，按定时器频率稳定发布。
    if (pending_feedback_)
    {
      const auto state = *pending_feedback_;
      pending_feedback_.reset();
      process_feedback(state);
    }
    // Twist -> 三轮轮输出轴速度。
    const auto wheel_speed = kinematics_->inverse(command_);

    CommandFrame frame{};
    // 线上没有 enable 字段；这里保持接口语义清楚，编码器只发送三个 float。
    frame.enable = true;
    for (size_t i = 0; i < 3; ++i)
      frame.motor_rad_s[i] = static_cast<float>(wheel_speed[i] * motor_ratio_[i]);
    if (log_serial_frames_)
    {
      RCLCPP_INFO(
        get_logger(),
        "\nTX 目标==[%6.4f %6.4f %6.4f]\n 轮子=[%6.4f %6.4f %6.4f]\n 电机=[%6.4f %6.4f %6.4f]",
        command_.vx, command_.vy, command_.wz,
        wheel_speed[0], wheel_speed[1], wheel_speed[2],
        frame.motor_rad_s[0], frame.motor_rad_s[1], frame.motor_rad_s[2]);
    }
    send_command(frame);
  }

  /** 将三个电机转子轴速度组装为 18 字节 0x3A 帧并写入 USB。 */
  void send_command(const CommandFrame &command)
  {
    if (fd_ < 0)
      return;
    const auto frame = encode_command(command);
    const ssize_t written = write(fd_, frame.data(), frame.size());
    if (written == static_cast<ssize_t>(frame.size()))
    {
      // 暂停原因：常态只打印结构化目标速度，避免 50 Hz 十六进制日志淹没终端。
      // log_frame("TX", frame);
      return;
    }
    close_serial();
  }

  /** 根据 log_serial_frames 参数打印完整 TX 帧。 */
  void log_frame(const char *direction, const std::vector<uint8_t> &frame)
  {
    if (log_serial_frames_)
      RCLCPP_INFO(get_logger(), "%s[%zu] %s", direction, frame.size(), to_hex(frame).c_str());
  }

  /** 只缓存最新的合法反馈，避免粘包时在一个 ROS 周期内连续发布多条测量。 */
  void queue_feedback(uint8_t version, const std::vector<uint8_t> &payload)
  {
    if (version != kProtocolVersion)
      return;

    FeedbackFrame state{};
    if (!decode_feedback(payload, state))
      return;
    pending_feedback_ = state;
  }

  /**
   * 现场协议没有头尾帧，按固定 48 字节（12 个 float32）拼接反馈。
   * read() 可能返回半包或多个包，因此不能直接把一次 read 当成一帧。
   */
  void queue_raw_feedback(const uint8_t *data, size_t size)
  {
    raw_feedback_buffer_.insert(raw_feedback_buffer_.end(), data, data + size);
    while (raw_feedback_buffer_.size() >= kFeedbackPayloadLength)
    {
      const auto payload = std::vector<uint8_t>(
        raw_feedback_buffer_.begin(),
        raw_feedback_buffer_.begin() + kFeedbackPayloadLength);
      FeedbackFrame state{};
      if (decode_feedback(payload, state))
        pending_feedback_ = state;
      else
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "raw feedback payload is not 12 finite float32 values; drop 48 bytes");
      raw_feedback_buffer_.erase(
        raw_feedback_buffer_.begin(),
        raw_feedback_buffer_.begin() + kFeedbackPayloadLength);
    }
  }

  /** 在固定 ROS 周期处理一帧反馈并发布轮速、IMU 和调试消息。 */
  void process_feedback(const FeedbackFrame &state)
  {
    const auto steady_now = Clock::now();
    const double dt = have_feedback_time_ ?
      std::chrono::duration<double>(steady_now - last_feedback_time_).count() : 0.0;
    const Vector3 scaled_motor_velocity{
      state.motor_actual[0] * feedback_scale_,
      state.motor_actual[1] * feedback_scale_,
      state.motor_actual[2] * feedback_scale_};
    const auto motor_velocity = correct_vector(
      scaled_motor_velocity,
      {1.0, 1.0, 1.0}, motor_feedback_deadband_rad_s_);
    // 除减速比得到真实的轮子转速
    for (size_t i = 0; i < 3; ++i)
    {
      // 先在电机轴 rad/s 上清零静止抖动，再除减速比；这样三轮噪声
      // 不会经过正运动学重新组合成假的 vx、vy 或 wz。
      wheel_velocity_[i] = motor_velocity[i] / motor_ratio_[i];
      if (dt > 0.0 && dt < 1.0)
        wheel_position_[i] += wheel_velocity_[i] * dt;
    }
    last_feedback_time_ = steady_now;
    have_feedback_time_ = true;

    // 陀螺零漂只能用真静止样本估计：要求目标速度为零且电机反馈过死区后
    // 也为零，并持续 imu_stationary_hold_s 以让停车后的机械振动衰减。
    const bool at_rest =
      command_.vx == 0.0 && command_.vy == 0.0 && command_.wz == 0.0 &&
      motor_velocity[0] == 0.0 && motor_velocity[1] == 0.0 && motor_velocity[2] == 0.0;
    if (at_rest && !stationary_candidate_)
    {
      stationary_candidate_ = true;
      stationary_since_ = steady_now;
    }
    else if (!at_rest)
      stationary_candidate_ = false;
    const bool stationary = stationary_candidate_ &&
      std::chrono::duration<double>(steady_now - stationary_since_).count() >=
      imu_stationary_hold_s_;

    // 陀螺管线：符号/比例修正 -> 零漂扣除 + 低通 -> 静止死区。
    // 死区放在最后，阈值作用于去零漂后的残余噪声，不再兼职吸收零偏。
    const auto gyro_corrected = correct_vector(
      {state.gyro[0], state.gyro[1], state.gyro[2]},
      imu_angular_velocity_sign_, Vector3{0.0, 0.0, 0.0}, imu_angular_velocity_scale_);
    const auto gyro_filtered = gyro_filter_->process(gyro_corrected, stationary, dt);
    const auto angular_velocity = correct_vector(
      gyro_filtered, {1.0, 1.0, 1.0}, imu_angular_velocity_deadband_);
    if (!gyro_bias_logged_ && gyro_filter_->calibrated())
    {
      const auto &bias = gyro_filter_->bias();
      RCLCPP_INFO(
        get_logger(), "陀螺零漂标定完成: [%.5f %.5f %.5f] rad/s",
        bias[0], bias[1], bias[2]);
      gyro_bias_logged_ = true;
    }

    // 数据已通过布局和有限值检查；这里打印解码后的 12 个字段，
    // 便于直接观察电机反馈和 IMU 原始角速度/加速度，避免手工拆 float32。
    if (log_serial_frames_)
    {
      RCLCPP_INFO(
        get_logger(),
        "\nRT 电机=[%6.4f %6.4f %6.4f]\n"
        "欧拉角=[%6.4f %6.4f %6.4f]\n"
        "角速度原始=[%6.4f %6.4f %6.4f]\n"
        "角速度ROS=[%6.4f %6.4f %6.4f]\n"
        "加速度=[%6.4f %6.4f %6.4f]",
        state.motor_actual[0], state.motor_actual[1], state.motor_actual[2],
        state.euler[0], state.euler[1], state.euler[2],
        state.gyro[0], state.gyro[1], state.gyro[2],
        angular_velocity[0], angular_velocity[1], angular_velocity[2],
        state.accel[0], state.accel[1], state.accel[2]);
    }
    // 同一 0x3B 包中的轮速和 IMU 使用同一 ROS 时间戳，便于 EKF 对齐测量。
    const auto stamp = this->now();
    publish_odom(stamp, dt);
    publish_joints(stamp);
    publish_feedback_debug(state, stamp);
    // 底盘 IMU 与 Mid-360S IMU 始终分别发布；由 EKF YAML 选择输入源，节点间不互相依赖。
    publish_imu(state, angular_velocity, stamp);
  }

  /** 用反馈轮速积分并发布 /chassis/odom/raw；原始里程计不发布 TF。 */
  void publish_odom(const rclcpp::Time &stamp, double dt)
  {
    const Twist2D raw_twist = kinematics_->forward(wheel_velocity_);
    const auto filtered_twist = correct_vector(
      {raw_twist.vx, raw_twist.vy, raw_twist.wz},
      {1.0, 1.0, 1.0}, wheel_twist_deadband_);
    const Twist2D twist{filtered_twist[0], filtered_twist[1], filtered_twist[2]};
    if (dt > 0.0 && dt < 1.0)
    {
      const double c = std::cos(odom_yaw_);
      const double s = std::sin(odom_yaw_);
      odom_x_ += (twist.vx * c - twist.vy * s) * dt;
      odom_y_ += (twist.vx * s + twist.vy * c) * dt;
      odom_yaw_ += twist.wz * dt;
    }

    nav_msgs::msg::Odometry message;
    // Header 指明该位姿属于 odom 坐标系，被描述对象是 base_footprint。
    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    // Pose 是轮速积分的原始平面位姿，仅供调试；EKF 配置不重复融合该位姿。
    message.pose.pose.position.x = odom_x_;
    message.pose.pose.position.y = odom_y_;
    message.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);
    message.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
    // 6x6 协方差只填写 x、y、yaw 对角项，其余维度不参与二维链路。
    message.pose.covariance[0] = wheel_pose_variance_[0];
    message.pose.covariance[7] = wheel_pose_variance_[1];
    message.pose.covariance[35] = wheel_pose_variance_[2];
    // Twist 是 EKF 实际使用的当前 ROS/雷达统一机体系 vx、vy、wz；
    // 180° 方向变换已在 KiwiKinematics::forward() 完成，此处不能再次取反。
    message.twist.twist.linear.x = twist.vx;
    message.twist.twist.linear.y = twist.vy;
    message.twist.twist.angular.z = twist.wz;
    message.twist.covariance[0] = wheel_twist_variance_[0];
    message.twist.covariance[7] = wheel_twist_variance_[1];
    message.twist.covariance[35] = wheel_twist_variance_[2];
    wheel_odom_pub_->publish(message);
  }

  /** 发布三个轮子的累计角度和角速度。 */
  void publish_joints(const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = stamp;
    message.name = wheel_names_;
    message.position.assign(wheel_position_.begin(), wheel_position_.end());
    message.velocity.assign(wheel_velocity_.begin(), wheel_velocity_.end());
    joint_pub_->publish(message);
  }

  /** 把反馈 payload 的姿态、符号修正角速度和原始加速度发布到独立底盘 IMU 话题。 */
  void publish_imu(
    const FeedbackFrame &state, const Vector3 &angular_velocity, const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::Imu message;
    // Header 使用整帧统一接收时间；frame_id 对应 URDF 中的 chassis_imu_link。
    message.header.stamp = stamp;
    message.header.frame_id = imu_frame_;
    if (imu_publish_orientation_)
    {
      // 只有完成安装轴和航向标定后才发布 C 板欧拉角转换的四元数。
      const auto quaternion = rpy_to_quaternion(state.euler[0], state.euler[1], state.euler[2]);
      message.orientation.x = quaternion[0];
      message.orientation.y = quaternion[1];
      message.orientation.z = quaternion[2];
      message.orientation.w = quaternion[3];
      // orientation_covariance：roll、pitch、yaw 的方差，单位 rad^2。
      message.orientation_covariance[0] = imu_orientation_variance_[0];
      message.orientation_covariance[4] = imu_orientation_variance_[1];
      message.orientation_covariance[8] = imu_orientation_variance_[2];
    }
    else
    {
      // ROS 约定 covariance[0] = -1 表示姿态不可用，防止下游误融合漂移 yaw。
      message.orientation_covariance[0] = -1.0;
    }
    // angular_velocity：已按 ROS 右手系修正符号并应用静止死区，单位 rad/s。
    message.angular_velocity.x = angular_velocity[0];
    message.angular_velocity.y = angular_velocity[1];
    message.angular_velocity.z = angular_velocity[2];
    // angular_velocity_covariance：三轴角速度方差，单位 (rad/s)^2。
    message.angular_velocity_covariance[0] = imu_angular_velocity_variance_[0];
    message.angular_velocity_covariance[4] = imu_angular_velocity_variance_[1];
    message.angular_velocity_covariance[8] = imu_angular_velocity_variance_[2];
    // linear_acceleration：下位机六轴原始加速度，单位 m/s^2，未用于当前二维 EKF。
    message.linear_acceleration.x = state.accel[0];
    message.linear_acceleration.y = state.accel[1];
    message.linear_acceleration.z = state.accel[2];
    // linear_acceleration_covariance：三轴加速度方差，单位 (m/s^2)^2。
    message.linear_acceleration_covariance[0] = imu_linear_acceleration_variance_[0];
    message.linear_acceleration_covariance[4] = imu_linear_acceleration_variance_[1];
    message.linear_acceleration_covariance[8] = imu_linear_acceleration_variance_[2];
    imu_pub_->publish(message);
  }

  /** 独立发布 C 板反馈 payload 的 12 个原始标量，便于 Foxglove 直接选字段绘图。 */
  void publish_feedback_debug(const FeedbackFrame &state, const rclcpp::Time &stamp)
  {
    kiwi_base::msg::FeedbackDebug debug;
    debug.header.stamp = stamp;
    debug.header.frame_id = imu_frame_;
    debug.motor_1_actual_rad_s = state.motor_actual[0];
    debug.motor_2_actual_rad_s = state.motor_actual[1];
    debug.motor_3_actual_rad_s = state.motor_actual[2];
    debug.roll_rad = state.euler[0];
    debug.pitch_rad = state.euler[1];
    debug.yaw_rad = state.euler[2];
    debug.gyro_x_raw_rad_s = state.gyro[0];
    debug.gyro_y_raw_rad_s = state.gyro[1];
    debug.gyro_z_raw_rad_s = state.gyro[2];
    debug.accel_x_m_s2 = state.accel[0];
    debug.accel_y_m_s2 = state.accel[1];
    debug.accel_z_m_s2 = state.accel[2];
    feedback_debug_pub_->publish(debug);
  }

  std::string device_;
  std::string cmd_topic_;
  std::string wheel_odom_topic_;
  std::string imu_topic_;
  std::string feedback_debug_topic_;
  std::string joint_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  int fd_{-1};
  double publish_rate_{};
  double feedback_scale_{};
  bool log_serial_frames_{};
  bool feedback_framed_{};
  bool imu_publish_orientation_{};
  std::array<double, 3> motor_ratio_{};
  Vector3 motor_feedback_deadband_rad_s_{};
  std::array<double, 3> wheel_pose_variance_{};
  std::array<double, 3> wheel_twist_variance_{};
  std::array<double, 3> imu_orientation_variance_{};
  std::array<double, 3> imu_angular_velocity_variance_{};
  std::array<double, 3> imu_linear_acceleration_variance_{};
  Vector3 imu_angular_velocity_sign_{};
  Vector3 imu_angular_velocity_scale_{};
  Vector3 imu_angular_velocity_deadband_{};
  Vector3 wheel_twist_deadband_{};
  double imu_stationary_hold_s_{};
  std::optional<GyroBiasFilter> gyro_filter_;
  bool stationary_candidate_{false};
  Clock::time_point stationary_since_{};
  bool gyro_bias_logged_{false};
  std::vector<std::string> wheel_names_;
  std::optional<KiwiKinematics> kinematics_;
  Twist2D command_limits_{};
  Twist2D command_{};
  std::array<double, 3> wheel_position_{};
  std::array<double, 3> wheel_velocity_{};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool have_feedback_time_{false};
  Clock::time_point last_feedback_time_{};
  std::optional<FeedbackFrame> pending_feedback_;
  std::vector<uint8_t> raw_feedback_buffer_;
  FrameParser parser_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<kiwi_base::msg::FeedbackDebug>::SharedPtr feedback_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};
}  // namespace kiwi_base

/** ROS 2 进程入口。 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kiwi_base::KiwiBaseNode>());
  rclcpp::shutdown();
  return 0;
}
