# **swiftnav-ros2**
Swift Navigation's ROS2 SBP Driver for Piksi Multi/Duro, PGM, and PGM EVK

# **Table of contents**
- [Building the ROS2 driver](#building-the-ros2-driver)
- [Building the ROS2 driver using docker](#building-the-ros2-driver-using-docker)
- [ROS2 driver configuration](#ros2-driver-configuration)
- [Adding a new SBP message to ROS2 topic translation](#adding-a-new-sbp-message-to-ros2-topic-translation)
    - [Step 1 (Add a new class to publishers)](#step-1-add-a-new-class-to-publishers)
    - [Step 2 (Add the new cpp file to CMakeLists.txt)](#step-2-add-the-new-cpp-file-to-cmakeliststxt)
    - [Step 3 (Add the new publisher type)](#step-3-add-the-new-publisher-type)
    - [Step 4 (Add the new values to enabled_publishers_ids and enabled_publishers_topics in params.yaml)](#step-4-add-the-new-values-to-enabledpublishersids-and-enabledpublisherstopics-in-paramsyaml)
- [Adding a new ROS2 topic to SBP message transaltion](#adding-a-new-ros2-topic-to-sbp-message-transaltion)
    - [Step 1 (Add a new class to subscribers)](#step-1-add-a-new-class-to-subscribers)
    - [Step 2 (Add the new subscriber to subscriber_factory)](#step-2-add-the-new-subscriber-to-subscriberfactory)
    - [Step 3 (Add the new subscriber parameters to params.yaml)](#step-3-add-the-new-subscriber-parameters-to-paramsyaml)
    - [Step 4 (Add the new cpp file to CMakeLists.txt)](#step-4-add-the-new-cpp-file-to-cmakeliststxt)
- [Adding custom ROS2 messages (transcribe to ROS2 an existing SBP message)](#adding-custom-ros2-messages-transcribe-to-ros2-an-existing-sbp-message)
    - [Step 1 (Add a new custom msg)](#step-1-add-a-new-custom-msg)
    - [Step 2 (Add the new msg file to CMakeLists.txt)](#step-2-add-the-new-msg-file-to-cmakeliststxt)
    - [Step 3 (Add a new class to publishers)](#step-3-add-a-new-class-to-publishers)
    - [Step 4 (Add the new cpp file to CMakeLists.txt)](#step-4-add-the-new-cpp-file-to-cmakeliststxt)
    - [Step 5 (Add the new publisher type)](#step-5-add-the-new-publisher-type)
    - [Step 6 (Add the new values to enabled_publishers_ids and enabled_publishers_topics in params.yaml)](#step-6-add-the-new-values-to-enabledpublishersids-and-enabledpublisherstopics-in-paramsyaml)


# Building the ROS2 driver

## Step 1 (Install ROS 2 Humble):
 Follow [instructions to install Ros2 Humble](https://docs.ros.org/en/humble/Installation.html)

## Step 2 (Install libspb):
  - In any directory you wish, clone libsp v4.4.0, init the repo and install it.
    ```
      git clone https://github.com/swift-nav/libsbp.git
      cd libsbp
      git checkout v4.4.0
      cd c
      git submodule init
      mkdir build
      cd build
      cmake DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF ../ 
      make
      sudo make install
    ```
## Step 3 (download driver code)
  - Navigate to workspace directory (ie: ~/workspace)
    ```
     cd ~/workspace
     mkdir src
     cd src
     git clone https://github.com/swift-nav/swiftnav-ros2.git

## Step 4 (install dependencies)
  - Navigate to workspace directory (ie: ~/workspace)
  ```
    cd ~/workspace
    source /opt/ros/humble/setup.bash
    sudo apt-get update
    sudo apt-get install libserialport-dev
    rosdep install --from-paths src --ignore-src -r -y
  ```

## Step 5 (edit configuration)
  - Edit configurationo file. See [ROS2 driver configuration](#ros2-driver-configuration)
  ```
    vi config/params.yaml
  ```

## Step 6 (build)
  - Navigate to workspace directory (ie: ~/workspace)
  ```
    cd ~/workspace
    source /opt/ros/humble/setup.bash
    colcon build
  ```

## Step 7 (launching)
  - Source installed Swift driver and launch driver.
  ```
    source install/setup.bash
    ros2 launch swiftnav_ros2_driver sbpros2_driver.py
  ```

## Step 8 (change configuration & viewing topics)
  - Changing the configuration files can be done from the driver source, but the driver will need to be rebuilt. Alternatively the configuration file can be changed in the installed folder.
  ```
    vi install/swiftnav_ros2_driver/share/swiftnav_ros2_driver/config/params.yaml
  ```
  - Swift specific SBP messages are not part of the ROS2 standard library, thus the following command must be run in any terminal that is used for intergacing with this driver. (ie: echoing the angular_rate message in a new terminal)
  ```
    source install/setup.bash
    ros2 topic echo /angular_rate
  ```

# Building the ROS2 driver using docker

## Step 1 (clone and build docker image)
  - Clone the repo, build Docker image, run docker image.
  ```
    git clone https://github.com/swift-nav/swiftnav-ros2.git
    cd swiftnav-ros2
    docker build -t swiftnav-ros2 .
    docker run -it -v :/mnt/workspace/src/swiftnav-ros2 swiftnav-ros2:latest /bin/bash
  ```

## Step 2 (edit configuration)
  - Edit configurationo file. See [ROS2 driver configuration](#ros2-driver-configuration)
  ```
    vi config/params.yaml
  ```

## Step 3 (build)
  - Build driver inside docker image.
  ```
    cd /mnt/workspace/
    colcon build
  ```

## Step 4 (launching)
  - Launching the driver inside the docker image may require access to serial device or tcp ports inside the docker.
  ```
    source install/setup.bash
    ros2 launch swiftnav_ros2_driver sbpros2_driver.py
  ```

## Step 6 (change configuration)
  - Changing the configuration files can be done from the driver source, but the driver will need to be rebuilt. Alternatively the configuration file can be changed in the installed folder.
  ```
    vi install/swiftnav_ros2_driver/share/swiftnav_ros2_driver/config/params.yaml
  ```

# ROS2 driver configuration
The driver offers the following configuration options:

| Parameter | Accepted values | Description |
| :--- | :--- | :--- |
| interface | See [Interface values table](#interface-values-table) | Interface from which the driver will communicate with the device |
| sbp_file | Example: /logs/sbp_example_file.sbp | Path to the SBP file (Only used if interface is 1)|
| device_name | Example: COM1 (Windows), /dev/ttyS0 (Linux) | A valid serial device name for the OS (Only used if interface is 2)|
| connection_str | Example: "115200&#124;N&#124;8&#124;1&#124;N" (See [Connection string description](#connection-string-description)) | A connection string that describes the parameters needed for the serial communication (Only used if interface is 2)|
| host_ip | Example: 192.168.1.45 | A valid IP address (Only used if interface is 3)|
| host_port | Example: 8082 | A valid TCP port value (Only used if interface is 3)|
| timeout | Example: 10000 | A timeout for reading operations in milliseconds (used for interfaces 2 and 3) |
| navsatfix | True: Publish, False: Don't publish | Flag to enable/disable the publication of the navsatfix topic (std_msgs::NavSatFix)|
| timereference | True: Publish, False: Don't publish | Flag to enable/disable the publication of the timereference topic (std_msgs::TimeReference)|
| log_sbp_messages | True: Log messages, False: Don't Log messages | Dump all the SBP received messages into an SBP file |
| log_sbp_filepath | Example: /logs/sbp_files/ | Path (without file name) in which the sbp file for log should be created |


### Interface values table
| Value | Description |
| :--- | :--- |
| 0 | Invalid |
| 1 | Data from file |
| 2 | Serial port |
| 3 | TCP |

## Connection string description
The connection string for the serial interface has the form:
BAUD RATE&#124;PARITY&#124;DATA BITS&#124;STOP BITS&#124;FLOW CONTROL

### Available baud rates
1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800. 921600.
Please remember that this baud rates are subject of your hardware and OS capabilities.

### Parities table
| Value | Description |
|:--- | :--- |
| N | No parity |
| E | Even parity |
| O | Odd parity |
| M | Mark parity (Not available in some linux distributions) |
| S | Space parity (Not available in some linux distributions) |

### Data bits
Usually 7 or 8

### Stop Bits
1 or 2

### Flow control table
| Value | Description |
|:--- | :--- |
| N | No flow control |
| X | Xon/Xoff flow control |
| R | RTS/CTS flow control |
| D | DTR/DSR flow control |


# Adding a new SBP message to ROS2 topic translation
In order to add a new SBP to ROS2 translation unit, you should (let's assume that you want to translate MSG_VEL_ECEF_GNSS to geometry_msgs/Pose2D):

## Step 1 (Add a new class to publishers)
```
// Header file (pose_2d_publisher.h)
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>
#include <publishers/dummy_publisher.h>
#include <publishers/sbp2ros2_publisher.h>
#include <geometry_msgs/msg/pose2_d.hpp>

class Pose2DPublisher
    : public DummyPublisher,
      public SBP2ROS2Publisher<geometry_msgs::Pose2D, sbp_msg_vel_ecef_t> {
 public:
  Pose2DPublisher() = delete;
  Pose2DPublisher(sbp::State* state, const std::string& topic_name,
                  rclcpp::Node* node, const LoggerPtr& logger,
                  const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_vel_ecef_t& msg);

 protected:
  void publish() override;
};

// CPP file (pose_2d_publisher.cpp)
#include <publishers/pose_2d_publisher.h>

Pose2DPublisher::Pose2DPublisher(sbp::State* state,
                                const std::string& topic_name,
                                rclcpp::Node* node,
                                const LoggerPtr& logger,
                                const std::string& frame)
    : SBP2ROS2Publisher<geometry_msgs::Pose2D, sbp_msg_vel_ecef_t>(state, topic_name, node, logger, frame) {}

void Pose2DPublisher::handle_sbp_msg(uint16_t sender_id,
                                      const sbp_msg_vel_ecef_t& msg) {
  (void)sender_id;

  // Here you do the mappings / calculations needed
  // The following is just an example
  msg_.x = msg.x;
  msg_.y = msg.y;

  publish();
}

void Pose2DPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = geometry_msgs::Pose2D();
}
```
### Step 2 (Add the new cpp file to CMakeLists.txt)
```
add_executable(sbp-to-ros
  src/sbp-to-ros.cpp
  src/publishers/NavSatFixPublisher.cpp
  src/publishers/TimeReferencePublisher.cpp

  src/publishers/pose_2d_publisher.cpp # new file

  src/data_sources/sbp_file_datasource.cpp
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/sbp_data_sources.cpp
  src/logging/ros_logger.cpp
  src/logging/sbp_to_ros2_logger.cpp
  src/logging/sbp_file_logger.cpp
  )
```

### Step 3 (Add the new publisher type)
Modify the file publisher_factory.h
```
enum class Publishers {
  Invalid,          // Id to use in params.yaml list of enabled publishers
  AngularRate,      //  1
  BaselineHeading,  //  2
  GnssTimeOffset,   //  3
  GpsFix,           //  4
  ImuAux,           //  5
  ImuRaw,           //  6
  NavSatFix,        //  7
  Odometry,         //  8
  OrientEuler,      //  9
  OrientQuat,       // 10
  TimeReference,    // 11
  Wheeltick,        // 12
  PoseStamped,      // 13
  Pose2D,           // 14  <--- New publisher type ID
};
```

Modify the file publisher_factory.cpp
```
// Add the new include file
#include <publishers/pose_2d_publisher.h>

// Modify the publisherFactory function

  switch (pub_type) {
    case Publishers::AngularRate:
      pub = std::make_shared<AngularRatePublisher>(state, topic_name, node,
                                                   logger, frame);
      break;

    case Publishers::BaselineHeading:
      pub = std::make_shared<BaselineHeadingPublisher>(state, topic_name, node,
                                                       logger, frame);
      break;

    case Publishers::GnssTimeOffset:
      pub = std::make_shared<GnssTimeOffsetPublisher>(state, topic_name, node,
                                                      logger, frame);
      break;

    // New publisher
    case Publishers::Pose2D:
      pub = std::make_shared<Pose2DPublisher>(state, topic_name, node, logger,
                                              frame);
      break;

```

### Step 4 (Add the new values to enabled_publishers_ids and enabled_publishers_topics in params.yaml)
At this point you must add a new publisher id (added in step 3) to enabled_publishers_ids (note that it does not need to be in order) and a name with which to publish the topic.
Take care in not to mix topic ids and names, for example:
```
    enabled_publishers_ids: [1, 2, 3]
    enabled_publishers_topics:
      [
        "angular_rate",
        "baseline_heading",
        "gnss_time_offset"
      ]

    enabled_publishers_ids: [3, 1, 2]
    enabled_publishers_topics:
      [
        "gnss_time_offset",
        "angular_rate",
        "baseline_heading"
      ]
```

are correct because the ids correspond to topic names, but:
```
    enabled_publishers_ids: [1, 2, 3]
    enabled_publishers_topics:
      [
        "angular_rate",
        "gnss_time_offset",
        "baseline_heading"
      ]
```
is not, because id 2 corresponds to BaselineHeading publisher and id 3 to GnssTimeOffset (the topic names in this example will misslead users).

The new parameters are the 14 and the "pose_2d" topic name
```
SBPRos2Driver:
  ros__parameters:
    interface: 3
    sbp_file: "/workspaces/Swift/24-185316.sbp"
    device_name: "/dev/ttyS0"
    connection_str: "115200|N|8|1|N"
    host_ip: "127.0.0.1"
    host_port: 8082
    timeout: 2000
    enabled_publishers_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
    enabled_publishers_topics:
      [
        "angular_rate",
        "baseline_heading",
        "gnss_time_offset",
        "gpsfix",
        "imu_aux",
        "imu_raw",
        "navsatfix",
        "odometry",
        "orient_euler",
        "orient_quat",
        "timereference",
        "wheeltick",
        "posestamped",
        "pose_2d"
      ]

    log_sbp_messages: True
    log_sbp_filepath: "/workspaces/Swift"
```


# Adding a new ROS2 topic to SBP message transaltion
In order to add a new ROS2 to SBP translation unit, you should (let's assume that you want to translate sensor_msgs/Imu to MSG_IMU_RAW):

## Step 1 (Add a new class to subscribers)
```
// Header file (imu_subscriber.h)
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <subscribers/dummy_subscriber.h>
#include <subscribers/ros2_2_sbp_subscriber.h>

class IMUSubscriber : public DummySubscriber, public ROS22SBPSubscriber{
    public:
     IMUSubscriber() = delete;

     IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                   const std::string& topic_name,
                   const LoggerPtr& logger);

    protected:
     virtual void topic_callback(const sensor_msgs::msg::Imu & msg);

     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;   /** @brief ROS 2 publisher */
};

// CPP file (imu_subscriber.cpp)
#include <subscribers/imu_subscriber.h>

IMUSubscriber::IMUSubscriber(rclcpp::Node* node, sbp::State* state,
                             const std::string& topic_name,
                             const LoggerPtr& logger)
    : ROS22SBPSubscriber(node, state, logger),
      subscriber_(node_->create_subscription<sensor_msgs::msg::Imu>(
            topic_name, 10 ,std::bind(&IMUSubscriber::topic_callback, this, _1)))
      {}

void IMUSubscriber::topic_callback(const sensor_msgs::msg::Imu & msg)
{
  // sbp_msg_imu_aux_t sbp_imu_aux_msg; // TODO how to fill this?
  sbp_msg_t sbp_msg;

  sbp_msg.imu_raw.tow =
      msg.header.stamp.sec * 1000;  //+ msg.header.stamp.nsec / 10;
  sbp_msg.imu_raw.tow_f =
      msg.header.stamp.sec * 1000;  //+ msg.header.stamp.nsec / 10;
  sbp_msg.imu_raw.acc_x = msg.linear_acceleration.x;
  sbp_msg.imu_raw.acc_y = msg.linear_acceleration.y;
  sbp_msg.imu_raw.acc_z = msg.linear_acceleration.z;
  sbp_msg.imu_raw.gyr_x = msg.angular_velocity.x;
  sbp_msg.imu_raw.gyr_y = msg.angular_velocity.y;
  sbp_msg.imu_raw.gyr_z = msg.angular_velocity.z;

  send_message(SbpMsgImuRaw, sbp_msg);
}
```

## Step 2 (Add the new subscriber to subscriber_factory)
Add the new subscriber Id to the Subscribers enumeration (subscriber_factory.h)
```
enum class Subscribers {
  Invalid,   // Id to use in params.yaml list of enabled subscribers
  Imu,       //  1  <--- New Subscriber
};
```
Add the creation of the new subscriber to subscriberFactory function (subscriber_factory.cpp)
```
#include <subscribers/imu_subscriber.h>
...

  switch (sub_type) {
    // New subscriber
    case Subscribers::Imu:
      sub = std::make_shared<IMUSubscriber>(node, state, topic_name, logger);
      break;

    default:
      LOG_ERROR(logger, "Subscriber id: %d isn't valid",
                static_cast<int>(sub_type));
      break;
  }
```

## Step 3 (Add the new subscriber parameters to params.yaml)
```
    enabled_subscribers_ids: [1]
    enabled_subscribers_topics: ["/imudata"]
```
Note that in case you don't want any subscriber, you must set the parameters this way:
```
    enabled_subscribers_ids: [0]
    enabled_subscribers_topics: [""]
```


### Step 4 (Add the new cpp file to CMakeLists.txt)
```
add_executable(sbp-to-ros
  src/sbp-to-ros.cpp
  src/publishers/navsatfix_publisher.cpp
  src/publishers/timereference_publisher.cpp

  src/subscribers/imu_subscriber.cpp # new file

  src/data_sources/sbp_file_datasource.cpp
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/sbp_data_sources.cpp
  src/logging/ros_logger.cpp
  src/logging/sbp_to_ros2_logger.cpp
  src/logging/sbp_file_logger.cpp
  )
```

# Adding custom ROS2 messages (transcribe to ROS2 an existing SBP message)
In order to add a new ROS2 custom msg (one that directly transcribes an existing SBP message), you should (let's assume that you want to transcribe MSG_ORIENT_EULER into /orient_euler):

## Step 1 (Add a new custom msg)
In the msg folder create a new file for the message, lets name it OrientEuler.msg
```
uint32 tow
int32 roll
int32 pitch
int32 yaw
float32 roll_accuracy
float32 pitch_accuracy
float32 yaw_accuracy
uint8 flags
```

## Step 2 (Add the new msg file to CMakeLists.txt)
```
  rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AngularRate.msg"
  "msg/BaselineHeading.msg"
  "msg/GnssTimeOffset.msg"
  "msg/ImuAux.msg"
  "msg/ImuRaw.msg"
  "msg/Odometry.msg"

  "msg/OrientEuler.msg"  # New custom msg file

  "msg/OrientQuat.msg"
  "msg/Wheeltick.msg"
  DEPENDENCIES # Add packages that above messages depend on
  )
```

## Step 3 (Add a new class to publishers)
```
// Header file (orient_euler_publisher.h)
#include <rclcpp/rclcpp.hpp>
#include <swiftnav_ros2_driver/msg/orient_euler.hpp>  // Note that we're including the custom msg auto-generated header

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>

#include <publishers/SBP2ROS2Publisher.h>

class OrientEulerPublisher
    : public SBP2ROS2Publisher<swiftnav_ros2_driver::msg::OrientEuler,
                               sbp_msg_orient_euler_t> {
 public:
  OrientEulerPublisher() = delete;
  OrientEulerPublisher(sbp::State* state, const std::string& topic_name,
                       rclcpp::Node* node, const LoggerPtr& logger,
                       const std::string& frame);

  void handle_sbp_msg(uint16_t sender_id, const sbp_msg_orient_euler_t& msg);

 protected:
  void publish() override;
};

// CPP file (orient_euler_publisher.cpp)
OrientEulerPublisher::OrientEulerPublisher(
    sbp::State* state, const std::string& topic_name, rclcpp::Node* node,
    const LoggerPtr& logger, const std::string& frame)
    : SBP2ROS2Publisher<swiftnav_ros2_driver::msg::OrientEuler,
                        sbp_msg_orient_euler_t>(state, topic_name, node, logger, frame) {}

void OrientEulerPublisher::handle_sbp_msg(uint16_t sender_id,
                                          const sbp_msg_orient_euler_t& msg) {
  (void)sender_id;

  // We just map the contents of the SBP msg into the custom ROS2 msg
  msg_.tow = msg.tow;
  msg_.pitch = msg.pitch;
  msg_.pitch_accuracy = msg.pitch_accuracy;
  msg_.roll = msg.roll;
  msg_.roll_accuracy = msg.roll_accuracy;
  msg_.yaw = msg.yaw;
  msg_.yaw_accuracy = msg.yaw_accuracy;
  msg_.flags = msg.flags;
  publish();
}

void OrientEulerPublisher::publish() {
  publisher_->publish(msg_);
  msg_ = swiftnav_ros2_driver::msg::OrientEuler();
}
```

### Step 4 (Add the new cpp file to CMakeLists.txt)
```
add_executable(sbp-to-ros
  src/sbp-to-ros.cpp
  src/publishers/NavSatFixPublisher.cpp
  src/publishers/TimeReferencePublisher.cpp

  src/publishers/orient_euler_publisher.cpp   # New cpp file

  src/data_sources/sbp_file_datasource.cpp
  src/data_sources/sbp_serial_datasource.cpp
  src/data_sources/sbp_tcp_datasource.cpp
  src/data_sources/sbp_data_sources.cpp
  src/logging/ros_logger.cpp
  src/logging/sbp_to_ros2_logger.cpp
  src/logging/sbp_file_logger.cpp
  )
```

## Step 5 (Add the new publisher type)
Modify the file publisher_factory.h
```
enum class Publishers {
  Invalid,          // Id to use in params.yaml list of enabled publishers
  AngularRate,      //  1
  BaselineHeading,  //  2
  GnssTimeOffset,   //  3
  GpsFix,           //  4
  ImuAux,           //  5
  ImuRaw,           //  6
  NavSatFix,        //  7
  Odometry,         //  8
  OrientQuat,       //  9
  TimeReference,    // 10
  Wheeltick,        // 11
  PoseStamped,      // 12
  OrientEuler,      // 13   <-- Newly added
};
```

Modify the file publisher_factory.cpp
```
// Add the new include file
#include <publishers/pose_2d_publisher.h>

// Modify the publisherFactory function

  switch (pub_type) {
    case Publishers::AngularRate:
      pub = std::make_shared<AngularRatePublisher>(state, topic_name, node,
                                                   logger, frame);
      break;

    case Publishers::BaselineHeading:
      pub = std::make_shared<BaselineHeadingPublisher>(state, topic_name, node,
                                                       logger, frame);
      break;

    case Publishers::GnssTimeOffset:
      pub = std::make_shared<GnssTimeOffsetPublisher>(state, topic_name, node,
                                                      logger, frame);
      break;

    // New publisher
    case Publishers::OrientEuler:
      pub = std::make_shared<OrientEulerPublisher>(state, topic_name, node,
                                                   logger, frame);
      break;

```

## Step 6 (Add the new values to enabled_publishers_ids and enabled_publishers_topics in params.yaml)
At this point you must add a new publisher id (added in step 5) to enabled_publishers_ids (note that it does not need to be in order) and a name with which to publish the topic.
Take care in not to mix topic ids and names, for example:
```
    enabled_publishers_ids: [1, 2, 3]
    enabled_publishers_topics:
      [
        "angular_rate",
        "baseline_heading",
        "gnss_time_offset"
      ]

    enabled_publishers_ids: [3, 1, 2]
    enabled_publishers_topics:
      [
        "gnss_time_offset",
        "angular_rate",
        "baseline_heading"
      ]
```

are correct because the ids correspond to topic names, but:
```
    enabled_publishers_ids: [1, 2, 3]
    enabled_publishers_topics:
      [
        "angular_rate",
        "gnss_time_offset",
        "baseline_heading"
      ]
```
is not, because id 2 corresponds to BaselineHeading publisher and id 3 to GnssTimeOffset (the topic names in this example will misslead users).

The new parameters are the 13 and the "orient_euler" topic name
```
SBPRos2Driver:
  ros__parameters:
    interface: 3
    sbp_file: "/workspaces/Swift/24-185316.sbp"
    device_name: "/dev/ttyS0"
    connection_str: "115200|N|8|1|N"
    host_ip: "127.0.0.1"
    host_port: 8082
    timeout: 2000
    enabled_publishers_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]
    enabled_publishers_topics:
      [
        "angular_rate",
        "baseline_heading",
        "gnss_time_offset",
        "gpsfix",
        "imu_aux",
        "imu_raw",
        "navsatfix",
        "odometry",
        "orient_quat",
        "timereference",
        "wheeltick",
        "posestamped",
        "orient_euler"
      ]

    log_sbp_messages: True
    log_sbp_filepath: "/workspaces/Swift"
```
Note that in case you don't want any publisher, you must set the parameters this way:
```
    enabled_publishers_ids: [0]
    enabled_publishers_topics: [""]
```
