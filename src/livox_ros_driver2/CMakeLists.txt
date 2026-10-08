# judge which cmake codes to use 
if(ROS_EDITION STREQUAL "ROS1")

  # Copyright(c) 2019 livoxtech limited.

  cmake_minimum_required(VERSION 3.0)


  #---------------------------------------------------------------------------------------
  # Start livox_ros_driver2 project
  #---------------------------------------------------------------------------------------
  include(cmake/version.cmake)
  project(livox_ros_driver2 VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
  message(STATUS "livox_ros_driver2 version: ${LIVOX_ROS_DRIVER2_VERSION}")

  #---------------------------------------------------------------------------------------
  # Add ROS Version MACRO
  #---------------------------------------------------------------------------------------
  add_definitions(-DBUILDING_ROS1)

  #---------------------------------------------------------------------------------------
  # find package and the dependecy
  #---------------------------------------------------------------------------------------
  find_package(Boost 1.54 REQUIRED COMPONENTS
    system
    thread
    chrono
  )

  ## Find catkin macros and libraries
  ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
  ## is used, also find other catkin packages
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_generation
    rosbag
    pcl_ros
  )

  ## Find pcl lib
  find_package(PCL REQUIRED)

  ## Generate messages in the 'msg' folder
  add_message_files(FILES
    CustomPoint.msg
    CustomMsg.msg
  #   Message2.msg
  )

  ## Generate added messages and services with any dependencies listed here
  generate_messages(DEPENDENCIES
    std_msgs
  )

  find_package(PkgConfig)
  pkg_check_modules(APR apr-1)
  if (APR_FOUND)
    message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES})
  endif (APR_FOUND)

  ###################################
  ## catkin specific configuration ##
  ###################################
  ## The catkin_package macro generates cmake config files for your package
  ## Declare things to be passed to dependent projects
  ## INCLUDE_DIRS: uncomment this if your package contains header files
  ## LIBRARIES: libraries you create in this project that dependent projects als    o need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## DEPENDS: system dependencies of this project that dependent projects also n    eed
  catkin_package(CATKIN_DEPENDS
    roscpp rospy std_msgs message_runtime
    pcl_ros
  )

  #---------------------------------------------------------------------------------------
  # Set default build to release
  #---------------------------------------------------------------------------------------
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
  endif()

  #---------------------------------------------------------------------------------------
  # Compiler config
  #---------------------------------------------------------------------------------------
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  ## make sure the livox_lidar_sdk_static library is installed
  find_library(LIVOX_LIDAR_SDK_LIBRARY  liblivox_lidar_sdk_static.a    /usr/local/lib)

  ## PCL library
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  #---------------------------------------------------------------------------------------
  # generate excutable and add libraries
  #---------------------------------------------------------------------------------------
  add_executable(${PROJECT_NAME}_node
    ""
  )

  #---------------------------------------------------------------------------------------
  # precompile macro and compile option
  #---------------------------------------------------------------------------------------
  target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
  )

  #---------------------------------------------------------------------------------------
  # add projects that depend on
  #---------------------------------------------------------------------------------------
  add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

  #---------------------------------------------------------------------------------------
  # source file
  #---------------------------------------------------------------------------------------
  target_sources(${PROJECT_NAME}_node
    PRIVATE
    src/driver_node.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/lddc.cpp
    src/livox_ros_driver2.cpp

    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp

    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp

    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )

  #---------------------------------------------------------------------------------------
  # include file
  #---------------------------------------------------------------------------------------
  target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    3rdparty
    src
  )

  #---------------------------------------------------------------------------------------
  # link libraries
  #---------------------------------------------------------------------------------------
  target_link_libraries(${PROJECT_NAME}_node
    ${LIVOX_LIDAR_SDK_LIBRARY}
    ${Boost_LIBRARY}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
  )


  #---------------------------------------------------------------------------------------
  # Install
  #---------------------------------------------------------------------------------------

  install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY launch_ROS1/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch_ROS1
  )

  #---------------------------------------------------------------------------------------
  # end of CMakeList.txt
  #---------------------------------------------------------------------------------------


else()
  # Copyright(c) 2020 livoxtech limited.
  # ROS 2 is the default for direct colcon builds in this Humble workspace.
  cmake_minimum_required(VERSION 3.14)
  project(livox_ros_driver2)

  set(CMAKE_C_STANDARD 99)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  include(cmake/version.cmake)
  project(${PROJECT_NAME} VERSION ${LIVOX_ROS_DRIVER2_VERSION} LANGUAGES CXX)
  message(STATUS "${PROJECT_NAME} version: ${LIVOX_ROS_DRIVER2_VERSION}")

  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()
  find_package(PCL REQUIRED COMPONENTS common)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  set(LIVOX_INTERFACES livox_interfaces2)
  rosidl_generate_interfaces(${LIVOX_INTERFACES}
    "msg/CustomPoint.msg"
    "msg/CustomMsg.msg"
    DEPENDENCIES builtin_interfaces std_msgs
    LIBRARY_NAME ${PROJECT_NAME}
  )

  # COLCON_IGNORE excludes SDK package discovery; add_subdirectory builds its
  # static target as part of this ROS package, without a global SDK install.
  get_filename_component(LIVOX_SDK_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/livox_sdk2" ABSOLUTE)
  if(NOT EXISTS "${LIVOX_SDK_SOURCE_DIR}/sdk_core/CMakeLists.txt")
    message(FATAL_ERROR "Missing bundled SDK: ${LIVOX_SDK_SOURCE_DIR}")
  endif()
  set(LIVOX_SDK2_BUILD_SHARED OFF)
  set(LIVOX_SDK2_BUILD_SAMPLES OFF)
  set(LIVOX_SDK2_INSTALL OFF)
  add_subdirectory("${LIVOX_SDK_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2")

  add_library(${PROJECT_NAME} SHARED
    src/livox_ros_driver2.cpp
    src/lddc.cpp
    src/driver_node.cpp
    src/lds.cpp
    src/lds_lidar.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp
    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp
    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )
  target_compile_definitions(${PROJECT_NAME} PRIVATE BUILDING_ROS2)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE
      -Wall -Wextra -Wpedantic -Wno-unused-parameter)
  endif()
  ament_target_dependencies(${PROJECT_NAME} SYSTEM PUBLIC
    rclcpp rclcpp_components std_msgs sensor_msgs rcutils pcl_conversions
    rcl_interfaces builtin_interfaces)
  target_include_directories(${PROJECT_NAME} PRIVATE
    ${PCL_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/3rdparty"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
  )
  target_compile_options(${PROJECT_NAME} PRIVATE ${PCL_DEFINITIONS})

  # Detect the available ROS API instead of relying on build.sh arguments.
  if(COMMAND rosidl_get_typesupport_target)
    rosidl_get_typesupport_target(cpp_typesupport_target
      ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
    target_link_libraries(${PROJECT_NAME} PRIVATE "${cpp_typesupport_target}")
  else()
    rosidl_target_interfaces(${PROJECT_NAME}
      ${LIVOX_INTERFACES} "rosidl_typesupport_cpp")
  endif()
  # PRIVATE keeps the SDK target out of the installed component's interface.
  target_link_libraries(${PROJECT_NAME} PRIVATE
    livox_lidar_sdk_static ${PCL_LIBRARIES})

  rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "livox_ros::DriverNode"
    EXECUTABLE ${PROJECT_NAME}_node
  )
  install(TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  ament_export_libraries(${PROJECT_NAME})

  if(BUILD_TESTING)
    # This vendored package retains upstream ROS1 sources and formatting.
    # Verify linkage and message typesupport rather than reformat upstream.
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_bundled_sdk test/test_bundled_sdk.py
      ENV
        "LIVOX_DRIVER_LIBRARY=$<TARGET_FILE:${PROJECT_NAME}>"
        "LIVOX_SDK_HEADER=${LIVOX_SDK_SOURCE_DIR}/include/livox_lidar_def.h"
      TIMEOUT 30
    )
  endif()

  ament_auto_package(INSTALL_TO_SHARE config launch_ROS2)
endif()
