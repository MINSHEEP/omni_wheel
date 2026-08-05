/**
 * @file kiwi_base_node.cpp
 * @brief Kiwi 三轮底盘最小联调节点。
 *
 * 运行链路只有四步：
 *   /cmd_vel -> 三行固定公式 -> USB 0x3A -> C 板反馈发布。
 * 速度限幅、斜坡、急停和反馈联锁暂不放入运行路径，待基础通信确认后再恢复。
 */
#include "kiwi_base/kiwi_kinematics.hpp"
#include "kiwi_base/robomaster_protocol.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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

    const auto ratios = declare_parameter<std::vector<double>>(
      "motor_reduction_ratio", {19.0, 19.0, 19.0});
    if (ratios.size() == 3)
      for (size_t i = 0; i < 3; ++i)
        motor_ratio_[i] = ratios[i];

    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    wheel_odom_topic_ = declare_parameter<std::string>("wheel_odom_topic", "/wheel/odom");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    joint_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_footprint");
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    wheel_names_ = declare_parameter<std::vector<std::string>>(
      "wheel_names", {"wheel_joint_1", "wheel_joint_2", "wheel_joint_3"});

    wheel_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(wheel_odom_topic_, 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_topic_, 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg)
      {
        // 键盘和联调脚本发布当前底盘控制坐标；Nav2 接入前需先做轴转换。
        command_ = {msg->linear.x, msg->linear.y, msg->angular.z};
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
        "键盘命令 vx=%.4f vy=%.4f wz=%.4f -> 轮子转速=[%.4f %.4f %.4f] 电机转速=[%.4f %.4f %.4f]",
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
      log_frame("TX", frame);
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

    // 0x3B 已通过版本、长度、CRC 和有限值检查；这里打印解码后的 12 个字段，
    // 便于直接观察电机反馈和 IMU 原始角速度/加速度，避免手工拆 float32。
    if (log_serial_frames_)
    {
      RCLCPP_INFO(
        get_logger(),
        "反馈电机转速=[%.4f %.4f %.4f] 欧拉角(rpy rad)=[%.4f %.4f %.4f] "
        "IMU原始角速度(rad/s)=[%.4f %.4f %.4f] "
        "IMU原始加速度(m/s^2)=[%.4f %.4f %.4f]",
        state.motor_actual[0], state.motor_actual[1], state.motor_actual[2],
        state.euler[0], state.euler[1], state.euler[2],
        state.gyro[0], state.gyro[1], state.gyro[2],
        state.accel[0], state.accel[1], state.accel[2]);
    }

    const auto now = Clock::now();
    const double dt = have_feedback_time_ ?
      std::chrono::duration<double>(now - last_feedback_time_).count() : 0.0;
    for (size_t i = 0; i < 3; ++i)
    {
      wheel_velocity_[i] = state.motor_actual[i] * feedback_scale_ / motor_ratio_[i];
      if (dt > 0.0 && dt < 1.0)
        wheel_position_[i] += wheel_velocity_[i] * dt;
    }
    last_feedback_time_ = now;
    have_feedback_time_ = true;
    publish_odom();
    publish_joints();
    publish_imu(state);
  }

  /** 用反馈轮速积分并发布 /wheel/odom；不发布 TF。 */
  void publish_odom()
  {
    const Twist2D twist = kinematics_.forward(wheel_velocity_);
    const auto now = Clock::now();
    if (have_odom_time_)
    {
      const double dt = std::chrono::duration<double>(now - last_odom_time_).count();
      const double c = std::cos(odom_yaw_);
      const double s = std::sin(odom_yaw_);
      odom_x_ += (twist.vx * c - twist.vy * s) * dt;
      odom_y_ += (twist.vx * s + twist.vy * c) * dt;
      odom_yaw_ += twist.wz * dt;
    }
    last_odom_time_ = now;
    have_odom_time_ = true;

    nav_msgs::msg::Odometry message;
    message.header.stamp = this->now();
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;
    message.pose.pose.position.x = odom_x_;
    message.pose.pose.position.y = odom_y_;
    message.pose.pose.orientation.z = std::sin(odom_yaw_ * 0.5);
    message.pose.pose.orientation.w = std::cos(odom_yaw_ * 0.5);
    message.twist.twist.linear.x = twist.vx;
    message.twist.twist.linear.y = twist.vy;
    message.twist.twist.angular.z = twist.wz;
    wheel_odom_pub_->publish(message);
  }

  /** 发布三个轮子的累计角度和角速度。 */
  void publish_joints()
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = wheel_names_;
    message.position.assign(wheel_position_.begin(), wheel_position_.end());
    message.velocity.assign(wheel_velocity_.begin(), wheel_velocity_.end());
    joint_pub_->publish(message);
  }

  /** 发布 0x3B 中带欧拉角的 IMU 数据。 */
  void publish_imu(const FeedbackFrame &state)
  {
    const auto quaternion = rpy_to_quaternion(state.euler[0], state.euler[1], state.euler[2]);
    sensor_msgs::msg::Imu message;
    message.header.stamp = now();
    message.header.frame_id = imu_frame_;
    message.orientation.x = quaternion[0];
    message.orientation.y = quaternion[1];
    message.orientation.z = quaternion[2];
    message.orientation.w = quaternion[3];
    message.angular_velocity.x = state.gyro[0];
    message.angular_velocity.y = state.gyro[1];
    message.angular_velocity.z = state.gyro[2];
    message.linear_acceleration.x = state.accel[0];
    message.linear_acceleration.y = state.accel[1];
    message.linear_acceleration.z = state.accel[2];
    // 0x3B 已包含欧拉角，因此保留 orientation 字段；协方差暂未标定。
    imu_pub_->publish(message);
  }

  std::string device_;
  std::string cmd_topic_;
  std::string wheel_odom_topic_;
  std::string imu_topic_;
  std::string joint_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string imu_frame_;
  int fd_{-1};
  double publish_rate_{50.0};
  double feedback_scale_{1.0};
  bool log_serial_frames_{false};
  std::array<double, 3> motor_ratio_{{19.0, 19.0, 19.0}};
  std::vector<std::string> wheel_names_;
  KiwiKinematics kinematics_;
  Twist2D command_{};
  std::array<double, 3> wheel_position_{};
  std::array<double, 3> wheel_velocity_{};
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};
  bool have_feedback_time_{false};
  bool have_odom_time_{false};
  Clock::time_point last_feedback_time_{};
  Clock::time_point last_odom_time_{};
  FrameParser parser_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
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
