// 双路 PWM 履带底盘 USB CDC 桥接：0x3A motor1/2 命令，0x3B 仅作 IMU。
// 无编码器反馈时不发布轮式里程计或关节状态。
#include "kiwi_base/robomaster_protocol.hpp"
#include "kiwi_base/tracked_kinematics.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace kiwi_base
{

class TrackedBaseNode final : public rclcpp::Node
{
public:
  TrackedBaseNode()
  : Node("tracked_base_node"),
    parser_([this](uint8_t version, const std::vector<uint8_t> & payload) {
      on_feedback(version, payload);
    })
  {
    device_ = declare_parameter<std::string>("device", "/dev/robomaster_c");
    cmd_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/chassis/imu/data_raw");
    imu_frame_ = declare_parameter<std::string>("imu_frame_id", "chassis_imu_link");
    const double separation = declare_parameter<double>("track_separation_m", 0.0);
    const double command_gain = declare_parameter<double>("command_per_m_s", 0.0);
    const double max_command = declare_parameter<double>("max_motor_command", 0.0);
    max_linear_ = declare_parameter<double>("max_linear_speed_m_s", 0.3);
    max_angular_ = declare_parameter<double>("max_angular_speed_rad_s", 0.8);
    timeout_ = declare_parameter<double>("cmd_vel_timeout_s", 0.5);
    const double rate = declare_parameter<double>("publish_rate", 50.0);
    gyro_z_sign_ = declare_parameter<double>("gyro_z_sign", -1.0);
    gyro_z_scale_ = declare_parameter<double>("gyro_z_scale", 1.69);
    gyro_z_variance_ = declare_parameter<double>("gyro_z_variance", 0.01);
    const bool calibrated = declare_parameter<bool>("hardware_calibrated", false);
    motion_enabled_ = declare_parameter<bool>("enable_motion", false);
    trial_enabled_ = declare_parameter<bool>("keyboard_trial_enabled", false);
    const double trial_gain = declare_parameter<double>("keyboard_trial_command_gain", 0.0);
    const double trial_limit = declare_parameter<double>("keyboard_trial_limit_rad_s", 0.0);
    test_enabled_ = declare_parameter<bool>("open_loop_test_enabled", false);
    test_topic_ = declare_parameter<std::string>(
      "open_loop_test_topic", "/chassis/open_loop_motor_cmd");
    test_limit_ = declare_parameter<double>("open_loop_test_limit_rad_s", 0.0);
    if (test_enabled_ && motion_enabled_)
      throw std::invalid_argument("open-loop test and calibrated cmd_vel cannot run together");
    if (trial_enabled_ && (calibrated || motion_enabled_ || test_enabled_))
      throw std::invalid_argument("keyboard trial must be separate from calibrated and raw motor modes");
    if (trial_enabled_ && (!std::isfinite(trial_gain) || trial_gain <= 0.0 ||
                          !std::isfinite(trial_limit) || trial_limit <= 0.0 ||
                          !std::isfinite(max_command) || trial_limit > max_command))
      throw std::invalid_argument("invalid keyboard trial command gain or limit");
    if (test_enabled_ && (test_topic_.empty() ||
                         !std::isfinite(test_limit_) || test_limit_ <= 0.0 ||
                         !std::isfinite(max_command) || test_limit_ > max_command))
      throw std::invalid_argument("invalid open-loop test limit");
    if (motion_enabled_ && !calibrated)
      throw std::invalid_argument("enable_motion requires hardware_calibrated=true");
    if (!std::isfinite(max_linear_) || max_linear_ <= 0.0 ||
        !std::isfinite(max_angular_) || max_angular_ <= 0.0 ||
        !std::isfinite(timeout_) || timeout_ <= 0.0 ||
        !std::isfinite(rate) || rate <= 0.0 ||
        !std::isfinite(gyro_z_scale_) || gyro_z_scale_ <= 0.0 ||
        (gyro_z_sign_ != 1.0 && gyro_z_sign_ != -1.0) ||
        !std::isfinite(gyro_z_variance_) || gyro_z_variance_ <= 0.0)
      throw std::invalid_argument("invalid tracked base limits or IMU calibration");
    // 临时键盘倍率只把 Twist 换成限幅电机命令，不代表实测 m/s，且不能解锁导航。
    if (calibrated)
      kinematics_ = std::make_unique<TrackedKinematics>(
        separation, command_gain, max_command);
    else if (trial_enabled_)
    {
      kinematics_ = std::make_unique<TrackedKinematics>(
        separation, trial_gain, trial_limit);
      RCLCPP_WARN(get_logger(),
                  "Uncalibrated keyboard trial enabled: gain %.1f, motor limit %.1f rad/s",
                  trial_gain, trial_limit);
    }
    else
      RCLCPP_WARN(get_logger(), "Tracked geometry/command gain TODO: motion locked");

    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_topic_, 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        on_cmd_vel(*msg);
      });
    if (test_enabled_)
    {
      sub_test_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        test_topic_, 10, [this](std_msgs::msg::Float32MultiArray::SharedPtr msg) {
          on_test_motor_cmd(*msg);
        });
      RCLCPP_WARN(get_logger(),
                  "Open-loop motor test enabled on %s (limit %.1f rad/s per track); cmd_vel locked",
                  test_topic_.c_str(), test_limit_);
    }
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() { update(); });
    RCLCPP_INFO(get_logger(), "Tracked USB bridge ready; motion=%s",
                motion_enabled_ ? "calibrated" : trial_enabled_ ? "keyboard_trial" : "locked");
  }

  ~TrackedBaseNode() override
  {
    if (fd_ >= 0)
    {
      const auto stop = encode_command(CommandFrame{true, {0.0F, 0.0F, 0.0F}});
      const ssize_t written = ::write(fd_, stop.data(), stop.size());
      if (written != static_cast<ssize_t>(stop.size()))
        RCLCPP_WARN(get_logger(), "Failed to send complete stop frame on shutdown");
      ::close(fd_);
    }
  }

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist & msg)
  {
    if (!std::isfinite(msg.linear.x) || !std::isfinite(msg.linear.y) ||
        !std::isfinite(msg.angular.z) || std::abs(msg.linear.y) > 1e-4)
    {
      vx_ = 0.0;
      wz_ = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Rejected non-finite or lateral tracked cmd_vel");
    }
    else
    {
      vx_ = std::clamp(msg.linear.x, -max_linear_, max_linear_);
      wz_ = std::clamp(msg.angular.z, -max_angular_, max_angular_);
    }
    last_cmd_ = std::chrono::steady_clock::now();
    have_cmd_ = true;
  }

  void on_test_motor_cmd(const std_msgs::msg::Float32MultiArray & msg)
  {
    const bool valid = msg.data.size() == 2 &&
      std::isfinite(msg.data[0]) && std::isfinite(msg.data[1]) &&
      std::abs(msg.data[0]) <= test_limit_ &&
      std::abs(msg.data[1]) <= test_limit_;
    if (valid)
    {
      test_left_ = msg.data[0];
      test_right_ = msg.data[1];
    }
    else
    {
      test_left_ = test_right_ = 0.0F;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Rejected invalid or over-limit open-loop motor command");
    }
    last_test_cmd_ = std::chrono::steady_clock::now();
    have_test_cmd_ = true;
  }

  void open_device()
  {
    if (fd_ >= 0)
      return;
    const int fd = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Cannot open %s: %s", device_.c_str(), std::strerror(errno));
      return;
    }
    if (::ioctl(fd, TIOCEXCL) != 0)
    {
      RCLCPP_ERROR(get_logger(), "USB device %s already claimed", device_.c_str());
      ::close(fd);
      return;
    }
    termios settings{};
    if (::tcgetattr(fd, &settings) != 0)
    {
      ::close(fd);
      return;
    }
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    if (::tcsetattr(fd, TCSANOW, &settings) != 0)
    {
      ::close(fd);
      return;
    }
    fd_ = fd;
    parser_.reset();
    RCLCPP_INFO(get_logger(), "USB device opened: %s", device_.c_str());
  }

  void close_device()
  {
    if (fd_ >= 0)
      ::close(fd_);
    fd_ = -1;
    parser_.reset();
  }

  void read_feedback()
  {
    uint8_t buffer[512];
    for (int i = 0; i < 4; ++i)
    {
      const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
      if (n > 0)
      {
        parser_.feed(buffer, static_cast<size_t>(n));
        continue;
      }
      if (n == 0 || (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)))
        return;
      if (n < 0 && errno == EINTR)
        continue;
      close_device();
      return;
    }
  }

  void on_feedback(uint8_t version, const std::vector<uint8_t> & payload)
  {
    FeedbackFrame feedback;
    if (version != kProtocolVersion || !decode_feedback(payload, feedback))
      return;
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = now();
    imu.header.frame_id = imu_frame_;
    // C 板绝对 yaw 与安装轴尚未标定；只给 EKF 融合已验证符号的 gyro_z。
    imu.orientation_covariance[0] = -1.0;
    imu.angular_velocity.x = feedback.gyro[0];
    imu.angular_velocity.y = feedback.gyro[1];
    imu.angular_velocity.z = feedback.gyro[2] * gyro_z_sign_ * gyro_z_scale_;
    imu.angular_velocity_covariance[0] = 1e3;
    imu.angular_velocity_covariance[4] = 1e3;
    imu.angular_velocity_covariance[8] = gyro_z_variance_;
    imu.linear_acceleration.x = feedback.accel[0];
    imu.linear_acceleration.y = feedback.accel[1];
    imu.linear_acceleration.z = feedback.accel[2];
    imu.linear_acceleration_covariance[0] = -1.0;
    pub_imu_->publish(imu);
  }

  void update()
  {
    open_device();
    if (fd_ < 0)
      return;
    read_feedback();
    if (fd_ < 0)
      return;
    CommandFrame command;
    command.enable = true;
    const bool fresh = have_cmd_ &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_).count()
      <= timeout_;
    if (fresh && (motion_enabled_ || trial_enabled_) && kinematics_)
    {
      const auto motors = kinematics_->command(vx_, wz_);
      command.motor_rad_s = {motors.first, motors.second, 0.0F};
    }
    else if (test_enabled_ && have_test_cmd_ &&
             std::chrono::duration<double>(
               std::chrono::steady_clock::now() - last_test_cmd_).count() <= timeout_)
    {
      command.motor_rad_s = {test_left_, test_right_, 0.0F};
    }
    const auto frame = encode_command(command);
    const ssize_t written = ::write(fd_, frame.data(), frame.size());
    if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
      close_device();
    // 0x3A 无 ACK；绝不从命令积分里程计。短写在下一周期重发完整帧。
  }

  std::string device_, cmd_topic_, imu_topic_, imu_frame_, test_topic_;
  double max_linear_{}, max_angular_{}, timeout_{}, test_limit_{};
  double gyro_z_sign_{}, gyro_z_scale_{}, gyro_z_variance_{};
  bool motion_enabled_{false}, trial_enabled_{false}, have_cmd_{false};
  bool test_enabled_{false}, have_test_cmd_{false};
  double vx_{0.0}, wz_{0.0};
  float test_left_{0.0F}, test_right_{0.0F};
  std::chrono::steady_clock::time_point last_cmd_{}, last_test_cmd_{};
  int fd_{-1};
  std::unique_ptr<TrackedKinematics> kinematics_;
  FrameParser parser_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_test_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kiwi_base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<kiwi_base::TrackedBaseNode>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("tracked_base_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
