# Kiwi 建图集成包：安装 Python 诊断工具、启动文件、参数与机器人描述。
cmake_minimum_required(VERSION 3.8)
project(kiwi_slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# symlink-install 会继承源码的执行位；复制/解压工作区后脚本可能变成 0644。
set(KIWI_SLAM_PROGRAMS)
foreach(program kiwi_slam_monitor kiwi_slam_compatibility kiwi_mapping_scan_gate rf2o_covariance_adapter lio_odom_adapter mid360_scan_projector mid360_motion_gate tracked_imu_acceleration tracked_estimation_guard tracked_inertial_odometry calibrate_planar_imu)
  set(installed_program "${CMAKE_CURRENT_BINARY_DIR}/${program}")
  configure_file("scripts/${program}" "${installed_program}" COPYONLY)
  file(CHMOD "${installed_program}" PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
  list(APPEND KIWI_SLAM_PROGRAMS "${installed_program}")
endforeach()
install(PROGRAMS ${KIWI_SLAM_PROGRAMS} DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(
  FILES
    launch/kiwi_robot.launch.py
    launch/kiwi_slam.launch.py
    launch/m10p_uart_lidar.launch.py
    launch/mid360_pipeline.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)
install(
  FILES urdf/kiwi.urdf.xacro urdf/tracked.urdf.xacro urdf/tracked_mid360.urdf.xacro
  DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_monitor test/test_monitor.py)
  ament_add_pytest_test(test_config test/test_config.py)
  ament_add_pytest_test(test_compatibility test/test_compatibility.py)
  ament_add_pytest_test(test_runtime_contract test/test_runtime_contract.py)
  ament_add_pytest_test(test_mapping_gate test/test_mapping_gate.py)
  ament_add_pytest_test(test_odom_covariance test/test_odom_covariance.py)
  ament_add_pytest_test(test_planar_imu test/test_planar_imu.py)
  ament_add_pytest_test(test_tracked_estimation test/test_tracked_estimation.py)
  ament_add_pytest_test(test_laser_recovery test/test_laser_recovery.py)
  ament_add_pytest_test(test_inertial_odometry test/test_inertial_odometry.py)
  ament_add_pytest_test(test_estimation_guard test/test_estimation_guard.py)
  ament_add_pytest_test(test_lio_odom_adapter test/test_lio_odom_adapter.py)
  ament_add_pytest_test(test_mid360_scan_projector test/test_mid360_scan_projector.py)
  ament_add_pytest_test(test_mid360_contract test/test_mid360_contract.py)
  ament_add_pytest_test(test_mid360_motion_gate test/test_mid360_motion_gate.py)
  ament_add_pytest_test(test_mid360_web_launch test/test_mid360_web_launch.py)
  ament_add_pytest_test(test_workspace_paths test/test_workspace_paths.py)
endif()

ament_package()
