# Kiwi 三轮全向底盘 ROS 2 功能包的构建入口，安装驱动、遥操作、参数和启动文件。
cmake_minimum_required(VERSION 3.8)
project(kiwi_base)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 0x3B 调试和 0x3C 遥控状态消息；遥控输入不自动生成运动命令。
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeedbackDebug.msg"
  "msg/RemoteControl.msg"
  DEPENDENCIES std_msgs
)
include_directories(include)
add_library(
  kiwi_base_core
  src/kiwi_kinematics.cpp
  src/measurement_processing.cpp
  src/robomaster_protocol.cpp
  src/stationary_covariance_estimator.cpp
  src/yaw_drift_estimator.cpp)
add_executable(kiwi_base_node src/kiwi_base_node.cpp)
target_link_libraries(kiwi_base_node kiwi_base_core)
ament_target_dependencies(kiwi_base_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)
add_executable(tracked_base_node src/tracked_base_node.cpp)
target_link_libraries(tracked_base_node kiwi_base_core)
ament_target_dependencies(tracked_base_node rclcpp geometry_msgs sensor_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kiwi_base_node "${cpp_typesupport_target}")
add_executable(kiwi_teleop_keyboard src/kiwi_teleop_keyboard.cpp)
ament_target_dependencies(kiwi_teleop_keyboard ament_index_cpp rclcpp geometry_msgs)
add_executable(kiwi_odom_test src/kiwi_odom_test.cpp)
ament_target_dependencies(kiwi_odom_test rclcpp nav_msgs)
install(
  TARGETS kiwi_base_node tracked_base_node kiwi_teleop_keyboard kiwi_odom_test kiwi_base_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY msg/ DESTINATION share/${PROJECT_NAME}/msg)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  FILES
    launch/kiwi_base.launch.py
    launch/kiwi_base_ekf.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
# 安装命令发送工具；0x3B 工具只做离线协议自检，不与节点争用串口。
install(
  PROGRAMS tools/kiwi_serial_test.py tools/kiwi_feedback_receiver.py
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # 仅注册 package.xml 中声明且已适配当前代码风格的基础检查。
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics kiwi_base_core)
  ament_add_gtest(test_tracked_kinematics test/test_tracked_kinematics.cpp)
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol kiwi_base_core)
  ament_add_gtest(test_measurement_processing test/test_measurement_processing.cpp)
  target_link_libraries(test_measurement_processing kiwi_base_core)
  ament_add_gtest(test_yaw_drift_estimator test/test_yaw_drift_estimator.cpp)
  target_link_libraries(test_yaw_drift_estimator kiwi_base_core)
  ament_add_gtest(
    test_stationary_covariance_estimator
    test/test_stationary_covariance_estimator.cpp)
  target_link_libraries(test_stationary_covariance_estimator kiwi_base_core)
  ament_add_pytest_test(test_ekf_config test/test_ekf_config.py)
  ament_add_pytest_test(test_cmd_vel_watchdog test/test_cmd_vel_watchdog.py TIMEOUT 30)
  ament_add_pytest_test(test_remote_control test/test_remote_control.py TIMEOUT 30)
endif()
ament_package()
