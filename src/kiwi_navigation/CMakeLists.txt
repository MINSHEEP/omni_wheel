# Kiwi Nav2 集成包：安装 Python 速度中继、启动文件、参数和 RViz 配置。
cmake_minimum_required(VERSION 3.8)
project(kiwi_navigation)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS scripts/kiwi_nav_cmd_relay
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_cmd_relay test/test_cmd_relay.py)
  ament_add_pytest_test(test_config test/test_config.py)
endif()

ament_package()
