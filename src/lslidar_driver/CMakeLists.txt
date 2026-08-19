cmake_minimum_required(VERSION 3.5)
project(lslidar_driver VERSION 5.1.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
# 本机集成修改（有编译失败证据）：工控机未安装 libpcap-dev 且无 root 权限，
# 上游硬编码 -lpcap 会因缺少 pcap.h（input.hpp:24）与 libpcap.so 失败。
# 优先查找系统开发包；缺失时回退到 ~/.local/pcap-dev 下解包的官方 deb。
# 正式修复方式仍是 `sudo apt-get install libpcap-dev`，装好后本回退自动失效。
find_path(libpcap_INCLUDE_DIR pcap.h
  HINTS $ENV{HOME}/.local/pcap-dev/usr/include)
find_library(libpcap_LIBRARIES NAMES pcap
  HINTS $ENV{HOME}/.local/pcap-dev/usr/lib/x86_64-linux-gnu)
if(NOT libpcap_INCLUDE_DIR OR NOT libpcap_LIBRARIES)
  message(FATAL_ERROR "libpcap headers/library not found; install libpcap-dev")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lslidar_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(yaml-cpp REQUIRED)
# 驱动只使用点类型、PointCloud 和坐标变换；限制组件可避免无关 IO/pcap 探测。
find_package(PCL REQUIRED COMPONENTS common)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

#指定 PCL 路径
# set(PCL_INCLUDE_DIRS /usr/local/pcl-1.11/include/pcl-1.11 /usr/local/pcl-1.11/include/pcl-1.11/pcl)
# set(PCL_LIBRARIES /usr/local/pcl-1.11/lib/libpcl_common.so
#                    /usr/local/pcl-1.11/lib/libpcl_io.so
#                    /usr/local/pcl-1.11/lib/libpcl_filters.so
#                    /usr/local/pcl-1.11/lib/libpcl_kdtree.so
#                    /usr/local/pcl-1.11/lib/libpcl_segmentation.so)


#指定置 Eigen 路径
# set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)
# include_directories(${EIGEN_INCLUDE_DIRS})

add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\"")

add_library(lslidar_driver
  src/input.cpp
  src/lsiosr.cpp
  src/lslidar_ch_driver.cpp
  src/lslidar_cx_driver.cpp
  src/lslidar_ls_driver.cpp
  src/lslidar_x10_driver.cpp
  src/lslidar_services.cpp
  src/lslidar_device_info.cpp
)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(/usr/local/pcl-1.11/lib)

target_include_directories(lslidar_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${libpcap_INCLUDE_DIR}
)

target_link_libraries(lslidar_driver
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${libpcap_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

ament_target_dependencies(lslidar_driver
  rclcpp
  sensor_msgs
  std_msgs
  lslidar_msgs
  ament_index_cpp
  pcl_conversions
  Eigen3
)

add_executable(lslidar_driver_node src/lslidar_driver_node.cpp)
target_link_libraries(lslidar_driver_node
  lslidar_driver
  ${Boost_LIBRARIES}
)

ament_target_dependencies(lslidar_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  lslidar_msgs
  ament_index_cpp
  pcl_conversions
)

install(TARGETS 
  lslidar_driver
  lslidar_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include
)

install(DIRECTORY 
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 验证 M10P 缺失槽位不会压缩角度，并覆盖全无效空包。
  ament_add_gtest(test_m10p_decode test/test_m10p_decode.cpp)
  target_include_directories(test_m10p_decode PRIVATE include)
  target_link_libraries(test_m10p_decode lslidar_driver)
endif()

ament_export_include_directories(include)
ament_export_libraries(lslidar_driver)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_msgs
  lslidar_msgs
  pcl_conversions
  yaml-cpp
  Boost
  PCL
)

ament_package()
