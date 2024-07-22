cmake_minimum_required(VERSION 3.8)
project(swiftnav_ros2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

################################################
# Check the ROS2 version

set(ROS2_FOUND FALSE)
if(DEFINED ENV{ROS_DISTRO})
  set(FOUND_ROS2_DISTRO $ENV{ROS_DISTRO})
  set(ROS2_FOUND TRUE)
else()
  set(ROS2_DISTROS "ardent;crystal;dashing;eloquent;foxy;galactic;humble;rolling")
  set(ROS2_FOUND FALSE)
  foreach(distro ${ROS2_DISTROS})
    if(NOT ROS2_FOUND)
      find_path(RCLCPP_H rclcpp.hpp PATHS /opt/ros/${distro}/include/rclcpp)
      if(RCLCPP_H)
        set(FOUND_ROS2_DISTRO ${distro})
        set(ROS2_FOUND TRUE)
      endif()
    endif()
  endforeach()
endif()

if(${FOUND_ROS2_DISTRO} STREQUAL "foxy")
  add_definitions(-DFOUND_FOXY)
elseif((${FOUND_ROS2_DISTRO} STREQUAL "galactic") OR (${FOUND_ROS2_DISTRO} STREQUAL "humble"))
  add_definitions(-DFOUND_NEWER)
endif()
################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)

link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

add_executable(sbp-to-ros
  src/sbp-to-ros.cpp
  src/data_sources/sbp_file_datasource.cpp
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/sbp_data_sources.cpp
  src/data_sources/serial.cpp
  src/data_sources/tcp.cpp
  src/utils/utils.cpp
  src/utils/config.cpp
  src/logging/ros_logger.cpp
  src/logging/sbp_to_ros2_logger.cpp
  src/logging/sbp_file_logger.cpp
  src/publishers/navsatfix_publisher.cpp
  src/publishers/twistwithcovariancestamped_publisher.cpp
  src/publishers/timereference_publisher.cpp
  src/publishers/gpsfix_publisher.cpp
  src/publishers/baseline_publisher.cpp
  src/publishers/imu_publisher.cpp
  src/publishers/publisher_factory.cpp
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Baseline.msg"
    DEPENDENCIES std_msgs
  )

if((${FOUND_ROS2_DISTRO} STREQUAL "foxy") OR (${FOUND_ROS2_DISTRO} STREQUAL "galactic"))
  rosidl_target_interfaces(sbp-to-ros ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

target_link_libraries(sbp-to-ros sbp serialport "${cpp_typesupport_target}")
target_include_directories(sbp-to-ros PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
target_compile_features(sbp-to-ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(sbp-to-ros rclcpp sensor_msgs geometry_msgs nav_msgs gps_msgs tf2)
ament_export_dependencies(rosidl_default_runtime)

install(TARGETS
  sbp-to-ros
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(gps_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(tf2 REQUIRED)
  link_directories("/usr/local/lib/")
  include_directories("/usr/local/include/")

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gmock(${PROJECT_NAME}_test
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/serial.cpp
  src/data_sources/tcp.cpp
  src/utils/utils.cpp
  src/utils/config.cpp
  src/publishers/baseline_publisher.cpp
  src/publishers/gpsfix_publisher.cpp
  src/publishers/imu_publisher.cpp
  src/publishers/navsatfix_publisher.cpp
  src/publishers/twistwithcovariancestamped_publisher.cpp
  src/publishers/publisher_factory.cpp
  src/publishers/timereference_publisher.cpp
  test/mocked_logger.cpp
  test/test_network.cpp
  test/test_serial.cpp
  test/publishers/test_nav_sat_fix_publisher.cpp
  test/publishers/test_time_reference_publisher.cpp
  test/publishers/test_gps_fix_publisher.cpp
  test/publishers/test_custom_publishers.cpp
  test/test_main.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  if((${FOUND_ROS2_DISTRO} STREQUAL "foxy") OR (${FOUND_ROS2_DISTRO} STREQUAL "galactic"))
    rosidl_target_interfaces(${PROJECT_NAME}_test ${PROJECT_NAME} "rosidl_typesupport_cpp")
  else()
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  endif()

  target_compile_features(${PROJECT_NAME}_test PRIVATE c_std_99 cxx_std_17)  # Require C99 and C++17
  target_link_libraries(${PROJECT_NAME}_test sbp serialport "${cpp_typesupport_target}")
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp sensor_msgs geometry_msgs nav_msgs gps_msgs tf2)
  ament_export_dependencies(rosidl_default_runtime)
endif()

ament_package()
