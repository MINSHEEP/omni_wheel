/**
 * @file kiwi_teleop_keyboard.cpp
 * @brief 读取终端键盘并发布机体 Twist 指令。
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>

namespace
{
/** 判断调用方是否已经指定参数文件；显式参数文件优先于包内默认值。 */
bool has_params_file(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i)
  {
    const std::string argument(argv[i] == nullptr ? "" : argv[i]);
    if (argument == "--params-file" || argument.rfind("--params-file=", 0) == 0)
      return true;
  }
  return false;
}

}  // namespace

/**
 * @brief 键盘遥操作节点，仅产生 ROS 速度命令，不直接访问底盘串口。
 */
class KeyboardTeleop final : public rclcpp::Node
{
public:
  /**
   * 创建键盘速度源：将终端切换为 raw/non-blocking，并以 50 ms 周期发布 Twist。
   */
  KeyboardTeleop() : Node("kiwi_teleop_keyboard")
  {
    // 控制话题、速度和超时只由 kiwi_base.yaml 提供，避免代码与文档出现两套数值。
    const auto topic = declare_parameter<std::string>("cmd_vel_topic");
    speed_ = declare_parameter<double>("speed_m_s");
    yaw_ = declare_parameter<double>("yaw_rad_s");
    // SSH 终端没有按键释放事件，以最后一次有效运动按键的时间作为停车依据。
    deadman_timeout_ms_ = static_cast<int>(declare_parameter<int>("deadman_timeout_ms"));
    deadman_timeout_ms_ = std::max(1, deadman_timeout_ms_);
    pub_ = create_publisher<geometry_msgs::msg::Twist>(topic, 10);

    terminal_ready_ = isatty(STDIN_FILENO) == 1 && tcgetattr(STDIN_FILENO, &old_) == 0;
    if (terminal_ready_)
    {
      auto raw = old_;
      raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
      raw.c_cc[VMIN] = 0;
      raw.c_cc[VTIME] = 0;
      if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
      {
        RCLCPP_ERROR(get_logger(), "failed to configure terminal: %s", std::strerror(errno));
        terminal_ready_ = false;
      }
      else
      {
        termios_changed_ = true;
      }
      flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
      if (flags_ < 0 || fcntl(STDIN_FILENO, F_SETFL, flags_ | O_NONBLOCK) != 0)
      {
        RCLCPP_ERROR(get_logger(), "failed to configure non-blocking input: %s", std::strerror(errno));
        terminal_ready_ = false;
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "stdin is not a usable terminal; teleop will remain stopped");
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]()
                               { tick(); });
    RCLCPP_INFO(
        get_logger(),
        "keyboard: w=forward s=backward a=left d=right q=ccw e=cw, "
        "space/x=stop, timeout=%d ms, Ctrl-C=quit",
        deadman_timeout_ms_);
  }

  /** 发布多帧零速度并恢复终端属性，防止退出后底盘继续保持运动命令。 */
  ~KeyboardTeleop() override
  {
    publish_zero();
    /* 即使非阻塞设置失败，也必须恢复已经切换过的终端属性。 */
    if (termios_changed_)
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_);
      if (flags_ >= 0)
        fcntl(STDIN_FILENO, F_SETFL, flags_);
    }
  }

private:
  /** 读取所有待处理按键、更新速度状态并发布 Twist。 */
  void tick()
  {
    bool refreshed = false;
    if (terminal_ready_)
    {
      char key = 0;
      for (;;)
      {
        const ssize_t count = read(STDIN_FILENO, &key, 1);
        if (count == 1)
        {
          // 直接发布 ROS 平面坐标：+x 前进、+y 左移、+z 逆时针。
          if (key == 'd')
          {
            vy_ = -speed_;
            vx_ = wz_ = 0.0;
            refreshed = true;
          }
          else if (key == 'a')
          {
            vy_ = speed_;
            vx_ = wz_ = 0.0;
            refreshed = true;
          }
          else if (key == 's')
          {
            vx_ = -speed_;
            vy_ = wz_ = 0.0;
            refreshed = true;
          }
          else if (key == 'w')
          {
            vx_ = speed_;
            vy_ = wz_ = 0.0;
            refreshed = true;
          }
          else if (key == 'q')
          {
            wz_ = yaw_;
            vx_ = vy_ = 0.0;
            refreshed = true;
          }
          else if (key == 'e')
          {
            wz_ = -yaw_;
            vx_ = vy_ = 0.0;
            refreshed = true;
          }
          else if (key == ' ' || key == 'x')
          {
            vx_ = vy_ = wz_ = 0.0;
            have_key_ = false;
            refreshed = false;
          }
          else
            continue;
        }
        else if (count == 0 || (count < 0 &&
                                (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)))
        {
          break;
        }
        else
        {
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000, "keyboard read failed: %s", std::strerror(errno));
          terminal_ready_ = false;
          vx_ = vy_ = wz_ = 0.0;
          have_key_ = false;
          break;
        }
      }
    }
    if (refreshed)
    {
      last_key_steady_ = std::chrono::steady_clock::now();
      have_key_ = true;
    }
    const auto age = have_key_ ? std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - last_key_steady_)
                                     .count()
                               : static_cast<int64_t>(deadman_timeout_ms_) + 1;
    if (age > deadman_timeout_ms_)
    {
      vx_ = vy_ = wz_ = 0.0;
      have_key_ = false;
    }

    geometry_msgs::msg::Twist message;
    message.linear.x = vx_;
    message.linear.y = vy_;
    message.angular.z = wz_;
    pub_->publish(message);
  }

  /** 连续发布零 Twist，覆盖下游队列中可能残留的最后一条命令。 */
  void publish_zero()
  {
    if (!pub_)
      return;
    geometry_msgs::msg::Twist message;
    for (int i = 0; i < 3; ++i)
      pub_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  termios old_{};
  int flags_{-1};
  int deadman_timeout_ms_{};
  bool terminal_ready_{false};
  bool termios_changed_{false};
  bool have_key_{false};
  double speed_{};
  double yaw_{};
  double vx_{0.0};
  double vy_{0.0};
  double wz_{0.0};
  std::chrono::steady_clock::time_point last_key_steady_{};
};

/** ROS 2 进程入口：运行键盘节点，退出时由析构函数负责安全停车。 */
int main(int argc, char **argv)
{
  // 官方 kiwi_teleop_keyboard 包装程序先读取活动底盘档案并传入参数文件。
  // 直接调用底层二进制却没有参数时立即失败，避免悄悄回退到 Kiwi 参数。
  if (!has_params_file(argc, argv))
  {
    std::fprintf(stderr, "缺少 --params-file；请使用 ros2 run kiwi_base kiwi_teleop_keyboard\n");
    return 2;
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
