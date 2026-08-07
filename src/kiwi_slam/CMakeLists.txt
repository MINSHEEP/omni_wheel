# Kiwi 建图集成包：安装 Python 诊断工具、启动文件、参数与机器人描述。
cmake_minimum_required(VERSION 3.8)
project(kiwi_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
    scripts/kiwi_slam_monitor
    scripts/kiwi_slam_compatibility
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_monitor test/test_monitor.py)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_compatibility test/test_compatibility.py)
endif()

ament_package()
