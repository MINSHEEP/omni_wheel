# Kiwi Nav2 集成包：安装启动文件、参数和 RViz 配置。
cmake_minimum_required(VERSION 3.8)
project(kiwi_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY behavior_trees/ DESTINATION share/${PROJECT_NAME}/behavior_trees)
install(
  FILES
    launch/localization_navigation.launch.py
    launch/mapping_navigation.launch.py
    launch/navigation_core.launch.py
    launch/platform.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(
    test_cross_package_contract
    test/test_cross_package_contract.py)
endif()

ament_package()
