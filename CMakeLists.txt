cmake_minimum_required(VERSION 3.8)
project(swiftnav_ros2_driver)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
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
  src/logging/ros_logger.cpp
  src/logging/sbp_to_ros2_logger.cpp
  src/logging/sbp_file_logger.cpp
  src/publishers/navsatfix_publisher.cpp
  src/publishers/timereference_publisher.cpp
  src/publishers/gpsfix_publisher.cpp
  src/publishers/posestamped_publisher.cpp
  src/publishers/baseline_heading_publisher.cpp
  src/publishers/angular_rate_publisher.cpp
  src/publishers/orient_euler_publisher.cpp
  src/publishers/orient_quat_publisher.cpp
  src/publishers/gnss_time_offset_publisher.cpp
  src/publishers/imu_aux_publisher.cpp
  src/publishers/imu_raw_publisher.cpp
  src/publishers/odometry_publisher.cpp
  src/publishers/publisher_factory.cpp
  src/publishers/wheeltick_publisher.cpp
  src/subscribers/imu_subscriber.cpp
  src/subscribers/odometry_subscriber.cpp
  src/subscribers/subscriber_factory.cpp
  )


  rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AngularRate.msg"
  "msg/BaselineHeading.msg"
  "msg/GnssTimeOffset.msg"
  "msg/ImuAux.msg"
  "msg/ImuRaw.msg"
  "msg/Odometry.msg"
  "msg/OrientEuler.msg"
  "msg/OrientQuat.msg"
  "msg/Wheeltick.msg"
  DEPENDENCIES # Add packages that above messages depend on
  )

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sbp-to-ros sbp serialport "${cpp_typesupport_target}")
target_include_directories(sbp-to-ros PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
  # $<INSTALL_INTERFACE:include>
  # $<INSTALL_INTERFACE:include/swiftnav_ros2_driver>)
target_compile_features(sbp-to-ros PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(sbp-to-ros rclcpp sensor_msgs nav_msgs gps_msgs tf2)
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
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  link_directories("/usr/local/lib/")
  include_directories("/usr/local/include/")

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
#  ament_add_gtest(${PROJECT_NAME}_test
  ament_add_gmock(${PROJECT_NAME}_test
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/serial.cpp
  src/data_sources/tcp.cpp
  src/publishers/navsatfix_publisher.cpp
  src/publishers/timereference_publisher.cpp
  src/subscribers/imu_subscriber.cpp
  src/subscribers/odometry_subscriber.cpp
  test/mocked_logger.cpp
  test/test_network.cpp
  test/test_serial.cpp
  test/test_subscribers.cpp
  test/publishers/test_nav_sat_fix_publisher.cpp
  test/publishers/test_time_reference_publisher.cpp
  test/test_main.cpp
  )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  )

  target_compile_features(${PROJECT_NAME}_test PRIVATE c_std_99 cxx_std_17)  # Require C99 and C++17
  # target_compile_options(${PROJECT_NAME}_test PRIVATE -fprofile-arcs -ftest-coverage)
  # target_link_options(${PROJECT_NAME}_test PRIVATE --coverage)
  # target_link_libraries(${PROJECT_NAME}_test sbp serialport gcov)
  target_link_libraries(${PROJECT_NAME}_test sbp serialport)
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp sensor_msgs nav_msgs)
endif()

ament_package()
