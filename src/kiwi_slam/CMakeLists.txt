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

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  FILES
    launch/kiwi_robot.launch.py
    launch/kiwi_slam.launch.py
    launch/m10p_uart_lidar.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(
  FILES urdf/kiwi.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_monitor test/test_monitor.py)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_compatibility test/test_compatibility.py)
  ament_add_pytest_test(test_runtime_contract test/test_runtime_contract.py)
endif()

ament_package()
