/**
 * @file kiwi_odom_test.cpp
 * @brief 订阅 EKF 输出，并在终端打印滤波后的 Pose 和 Twist。
 */
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <memory>
#include <string>

/** @brief 用于联调 /odometry/filtered 的只读监视节点。 */
class KiwiOdomTest final : public rclcpp::Node
{
public:
  KiwiOdomTest() : Node("kiwi_odom_test")
  {
    // 调试话题只从 kiwi_base.yaml 读取，避免测试工具暗含另一套默认接口。
    const auto topic = declare_parameter<std::string>("odom_topic");
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, 10, std::bind(&KiwiOdomTest::on_odometry, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "等待滤波里程计: %s", topic.c_str());
  }

private:
  /** @brief 按 Pose 和 Twist 分组打印一帧滤波里程计。 */
  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message) const
  {
    const auto &position = message->pose.pose.position;
    const auto &orientation = message->pose.pose.orientation;
    const auto &linear = message->twist.twist.linear;
    const auto &angular = message->twist.twist.angular;

    RCLCPP_INFO(
      get_logger(),
      "RX 滤波里程计 frame=%s child=%s\n"
      " Pose 位置=[%6.4f %6.4f %6.4f]\n"
      " 姿态四元数=[%6.4f %6.4f %6.4f %6.4f]\n"
      " Twist 线速度=[%6.4f %6.4f %6.4f]\n"
      " 角速度=[%.4f %6.4f %6.4f]",
      message->header.frame_id.c_str(), message->child_frame_id.c_str(),
      position.x, position.y, position.z,
      orientation.x, orientation.y, orientation.z, orientation.w,
      linear.x, linear.y, linear.z,
      angular.x, angular.y, angular.z);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

/** @brief ROS 2 进程入口。 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KiwiOdomTest>());
  rclcpp::shutdown();
  return 0;
}
