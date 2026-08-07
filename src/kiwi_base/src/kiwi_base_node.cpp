/**
 * @file kiwi_base_node.cpp
 * @brief Kiwi 三轮底盘最小联调节点。
 *
 * 运行链路只有四步：
 *   /cmd_vel_safe -> 三轴限幅 -> 三行物理公式 -> USB 0x3A -> C 板反馈发布。
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

/** 读取三个轴的方差；方差必须是有限正数，避免滤波器把无效配置当成真值。 */
std::array<double, 3> declare_variance(
  rclcpp::Node &node, const std::string &name, const std::array<double, 3> &defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
    name, {defaults[0], defaults[1], defaults[2]});
  if (values.size() != 3 || std::any_of(values.begin(), values.end(), [](double value)
    { return !std::isfinite(value) || value <= 0.0; }))
    throw std::invalid_argument(name + " must contain three finite positive variances");
  return {values[0], values[1], values[2]};
}

/** 读取有限三轴参数；符号和死区的具体范围由调用处检查。 */
Vector3 declare_vector3(
  rclcpp::Node &node, const std::string &name, const Vector3 &defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
    name, {defaults[0], defaults[1], defaults[2]});
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
      { on_feedback(version, payload); }
      // ,[this](const std::vector<uint8_t> &frame){ log_frame("RX", frame); }
    )
  {
    device_ = declare_parameter<std::string>("device", "/dev/robomaster_c");
    publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
    // C++ 默认值只作为未加载参数文件时的回退，正常由 YAML 控制日志开关。
    log_serial_frames_ = declare_parameter<bool>("log_serial_frames", false);
    feedback_scale_ = declare_parameter<double>("feedback_motor_scale_to_rad_s", 1.0);
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0 ||
        !std::isfinite(feedback_scale_) || feedback_scale_ <= 0.0)
      throw std::invalid_argument("publish rate and feedback scale must be finite and positive");

    const auto ratios = declare_parameter<std::vector<double>>(
      "motor_reduction_ratio", {19.0, 19.0, 19.0});
    if (ratios.size() != 3 || std::any_of(ratios.begin(), ratios.end(), [](double value)
      { return !std::isfinite(value) || value <= 0.0; }))
      throw std::invalid_argument("motor reduction ratio must contain three finite positive values");
    for (size_t i = 0; i < 3; ++i)
      motor_ratio_[i] = ratios[i];

    // 几何参数统一由 kiwi_base.yaml 提供，默认值对应 203 mm 轮径和 322.56 mm 轮心距。
    const double wheel_radius_m = declare_parameter<double>("wheel_radius_m", 0.1015);
    const double wheel_center_distance_m = declare_parameter<double>("wheel_center_distance_m", 0.32256);
    kinematics_ = KiwiKinematics(wheel_radius_m, wheel_center_distance_m);

    // 三个 YAML 参数分别限制前后、左右和旋转速度，单位与 Twist 一致。
    command_limits_ = {
      declare_parameter<double>("max_linear_speed_m_s", 0.6),
      declare_parameter<double>("max_lateral_speed_m_s", 0.6),
      declare_parameter<double>("max_angular_speed_rad_s", 1.5)};
    if (!std::isfinite(command_limits_.vx) || command_limits_.vx <= 0.0 ||
        !std::isfinite(command_limits_.vy) || command_limits_.vy <= 0.0 ||
        !std::isfinite(command_limits_.wz) || command_limits_.wz <= 0.0)
      throw std::invalid_argument("velocity limits must be finite and positive");

    // 这些是 EKF 的测量噪声初值，数组顺序均为 x、y、z；实测静止数据后只改 YAML。
    wheel_pose_variance_ = declare_variance(*this, "wheel_pose_variance", {0.10, 0.10, 0.20});
    wheel_twist_variance_ = declare_variance(*this, "wheel_twist_variance", {0.02, 0.02, 0.04});
    imu_orientation_variance_ = declare_variance(
      *this, "imu_orientation_variance", {0.05, 0.05, 0.05});
    imu_angular_velocity_variance_ = declare_variance(
      *this, "imu_angular_velocity_variance", {0.01, 0.01, 0.01});
    imu_linear_acceleration_variance_ = declare_variance(
      *this, "imu_linear_acceleration_variance", {0.10, 0.10, 0.10});

    // 下位机 gyro_z 为逆时针负；按 ROS 右手系翻转 Z，并过滤实测静止噪声。
    imu_angular_velocity_sign_ = declare_vector3(
      *this, "imu_angular_velocity_sign", {1.0, 1.0, -1.0});
    imu_angular_velocity_deadband_ = declare_vector3(
      *this, "imu_angular_velocity_deadband_rad_s", {0.015, 0.015, 0.015});
    wheel_twist_deadband_ = declare_vector3(
      *this, "wheel_twist_deadband", {0.015, 0.015, 0.015});
    if (std::any_of(
        imu_angular_velocity_sign_.begin(), imu_angular_velocity_sign_.end(),
        [](double value) { return std::abs(value) != 1.0; }))
      throw std::invalid_argument("imu angular velocity signs must be +1 or -1");
    const auto negative_deadband = [](double value) { return value < 0.0; };
    if (std::any_of(
        imu_angular_velocity_deadband_.begin(), imu_angular_velocity_deadband_.end(),
        negative_deadband) ||
        std::any_of(wheel_twist_deadband_.begin(), wheel_twist_deadband_.end(), negative_deadband))
      throw std::invalid_argument("measurement deadbands must be non-negative");

    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    feedback_debug_topic_ = declare_parameter<std::string>(
      "feedback_debug_topic", "/kiwi_base/feedback_debug");
    joint_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    wheel_names_ = declare_parameter<std::vector<std::string>>(
      "wheel_names", {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"});
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
  }

  /** 读取当前可用的 USB 字节，交给 FrameParser 处理半帧和粘包。 */
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
        parser_.feed(buffer, static_cast<size_t>(count));
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
    // Twist -> 三轮轮输出轴速度。
    const auto wheel_speed = kinematics_.inverse(command_);

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

  /** 解码 C 板唯一的 0x3B/48 字节反馈，并发布轮速和 IMU。 */
  void on_feedback(uint8_t version, const std::vector<uint8_t> &payload)
  {
    if (version != kProtocolVersion)
      return;

    FeedbackFrame state{};
    if (!decode_feedback(payload, state))
      return;

    const auto angular_velocity = correct_vector(
      {state.gyro[0], state.gyro[1], state.gyro[2]},
      imu_angular_velocity_sign_, imu_angular_velocity_deadband_);

    // 0x3B 已通过版本、长度、CRC 和有限值检查；这里打印解码后的 12 个字段，
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

    const auto steady_now = Clock::now();
    const double dt = have_feedback_time_ ?
      std::chrono::duration<double>(steady_now - last_feedback_time_).count() : 0.0;
    // 除减速比得到真实的轮子转速
    for (size_t i = 0; i < 3; ++i)
    {
      wheel_velocity_[i] = state.motor_actual[i] * feedback_scale_ / motor_ratio_[i];
      if (dt > 0.0 && dt < 1.0)
        wheel_position_[i] += wheel_velocity_[i] * dt;
    }
    last_feedback_time_ = steady_now;
    have_feedback_time_ = true;
    // 同一 0x3B 包中的轮速和 IMU 使用同一 ROS 时间戳，便于 EKF 对齐测量。
    const auto stamp = this->now();
    publish_odom(stamp, dt);
    publish_joints(stamp);
    publish_imu(state, angular_velocity, stamp);
  }

  /** 用反馈轮速积分并发布 /wheel/odom_raw；原始里程计不发布 TF。 */
  void publish_odom(const rclcpp::Time &stamp, double dt)
  {
    const Twist2D raw_twist = kinematics_.forward(wheel_velocity_);
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
    message.header.stamp = stamp;//里程计数据的 ROS 时间戳
    message.header.frame_id = odom_frame_;//位姿参考坐标系，当前通常是 odom
    message.child_frame_id = base_frame_;//被描述的机器人坐标系，当前通常是 base_footprint
    message.pose.pose.position.x = odom_x_;//机器人在 odom 坐标系中的 X 位置，单位 m
    message.pose.pose.position.y = odom_y_;//机器人在 odom 坐标系中的 Y 位置，单位 m
    message.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);//ROS 姿态必须使用四元数。当前机器人只考虑平面旋转，即只有 yaw
    message.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
    message.pose.covariance[0] = wheel_pose_variance_[0];//位姿协方差
    message.pose.covariance[7] = wheel_pose_variance_[1];
    message.pose.covariance[35] = wheel_pose_variance_[2];
    message.twist.twist.linear.x = twist.vx;//线速度和角速度
    message.twist.twist.linear.y = twist.vy;
    message.twist.twist.angular.z = twist.wz;
    message.twist.covariance[0] = wheel_twist_variance_[0];//速度协方差
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

  /** 发布 0x3B 中带欧拉角的 IMU 数据。 */
  void publish_imu(
    const FeedbackFrame &state, const Vector3 &angular_velocity, const rclcpp::Time &stamp)
  {
    const auto quaternion = rpy_to_quaternion(state.euler[0], state.euler[1], state.euler[2]);
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp; //数据采样/接收时间
    message.header.frame_id = imu_frame_; //数据所属坐标系，当前是 imu_link
    message.orientation.x = quaternion[0]; //姿态四元数，顺序 x、y、z、w
    message.orientation.y = quaternion[1];
    message.orientation.z = quaternion[2];
    message.orientation.w = quaternion[3];
    message.orientation_covariance[0] = imu_orientation_variance_[0];//姿态方差，单位近似 rad²
    message.orientation_covariance[4] = imu_orientation_variance_[1];
    message.orientation_covariance[8] = imu_orientation_variance_[2];
    message.angular_velocity.x = angular_velocity[0]; //三轴角速度，单位 rad/s
    message.angular_velocity.y = angular_velocity[1];
    message.angular_velocity.z = angular_velocity[2];
    message.angular_velocity_covariance[0] = imu_angular_velocity_variance_[0];// 角速度方差，单位 (rad/s)²
    message.angular_velocity_covariance[4] = imu_angular_velocity_variance_[1];
    message.angular_velocity_covariance[8] = imu_angular_velocity_variance_[2];
    message.linear_acceleration.x = state.accel[0]; //三轴加速度，单位 m/s²
    message.linear_acceleration.y = state.accel[1];
    message.linear_acceleration.z = state.accel[2];
    message.linear_acceleration_covariance[0] = imu_linear_acceleration_variance_[0];//加速度方差，单位 (m/s²)²
    message.linear_acceleration_covariance[4] = imu_linear_acceleration_variance_[1];
    message.linear_acceleration_covariance[8] = imu_linear_acceleration_variance_[2];
    imu_pub_->publish(message);

    // 独立调试接口：保留 C 板 0x3B 的 12 个已解码字段，便于 Foxglove 绘图。
    kiwi_base::msg::FeedbackDebug debug;
    debug.header = message.header;
    debug.motor_actual_rad_s = {
      state.motor_actual[0], state.motor_actual[1], state.motor_actual[2]};
    debug.euler_rad = {state.euler[0], state.euler[1], state.euler[2]};
    debug.gyro_raw_rad_s = {state.gyro[0], state.gyro[1], state.gyro[2]};
    debug.accel_m_s2 = {state.accel[0], state.accel[1], state.accel[2]};
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
  double publish_rate_{50.0};
  double feedback_scale_{1.0};
  bool log_serial_frames_{false};
  std::array<double, 3> motor_ratio_{{19.0, 19.0, 19.0}};
  std::array<double, 3> wheel_pose_variance_{{0.10, 0.10, 0.20}};
  std::array<double, 3> wheel_twist_variance_{{0.02, 0.02, 0.04}};
  std::array<double, 3> imu_orientation_variance_{{0.05, 0.05, 0.05}};
  std::array<double, 3> imu_angular_velocity_variance_{{0.01, 0.01, 0.01}};
  std::array<double, 3> imu_linear_acceleration_variance_{{0.10, 0.10, 0.10}};
  Vector3 imu_angular_velocity_sign_{{1.0, 1.0, -1.0}};
  Vector3 imu_angular_velocity_deadband_{{0.015, 0.015, 0.015}};
  Vector3 wheel_twist_deadband_{{0.015, 0.015, 0.015}};
  std::vector<std::string> wheel_names_;
  KiwiKinematics kinematics_;
  Twist2D command_limits_{0.6, 0.6, 1.5};
  Twist2D command_{};
  std::array<double, 3> wheel_position_{};
  std::array<double, 3> wheel_velocity_{};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool have_feedback_time_{false};
  Clock::time_point last_feedback_time_{};
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
