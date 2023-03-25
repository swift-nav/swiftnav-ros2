# **swiftnav-ros2**
ROS2 Driver for Swift Navigation's GNSS/INS Receivers and Starling Positioning Engine.

# **Table of Contents**
- [Features](#features)
- [Published ROS Topics](#published-ros2-topics)
- [Building Driver](#building-the-ros2-driver)
- [Building Driver Using Docker](#building-the-ros2-driver-using-docker)
- [Driver Configuration](#ros2-driver-configuration)

# Features
- Designed for ROS2 Humble but also works with Foxy. 
- Developed and tested on Ubuntu 22.04 (ROS2 Humble) and Ubuntu 20.04 (ROS2 Foxy) platforms.
- Supports Swift Navigation receivers and Starling Positioning Engine in Swift Binary Protocol (SBP).
- Ethernet, Serial and File inputs.
- SBP data logging.
- Publishes ROS2 standard and Swift Navigation proprietary topics.
- Configurable time stamps.
- Written in C++.

# Published ROS2 Topics
The driver parses Swift binary (SBP) messages and publishes the following ROS topics:
 - [`GpsFix`](#gpsfix)
 - [`NavSatFix`](#navsatfix)
 - [`TwistWithCovarianceStamped`](#twistwithcovariancestamped)
 - [`Baseline` (proprietary)](#baseline)
 - [`TimeReference`](#timereference)
 - [`Imu`](#imu)

## GpsFix
 
## NavSatFix
 
sensor_msgs/msg/NavSatFix

### SBP Messages Used
- `UTC TIME` (ID: 259) - UTC time of reported position.
- `POS LLH COV` (ID: 529) - GNSS position data with covariance.
- `MEASUREMENT STATE` (ID: 97) - GNSS constellations data.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting in the configuration file:  
- True: the topic is published upon receiving SBP `UTC TIME` and `POS LLH COV` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- False: the topic is published upon receiving SBP `POS LLH COV` message. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Text from `frame_name` in the config `params.yaml` file|
|`status.status`|`POS LLH COV`|Dead Reckoning (DR) position is reported as `STATUS_FIX` (0)|
|`status.service`|`MEASUREMENT STATE`|GNSS constellations from the last `MEASUREMENT STATE` message. Reports zero when message is not present.|
|`latitude`<br>`longitude`<br>`altitude`<br>`position_covariance`<br>`position_covariance_type`|`POS LLH COV`|Zeros when the fix is invalid. If position is valid altitude is always present (i.e. never NaN). Covariance, if valid, is always `TYPE_KNOWN` (full matrix).|

 
 ## TwistWithCovarianceStamped
 
 ## Baseline
 
 ## TimeReference
 
 sensor_msgs/msg/TimeReference

### SBP Messages Used
- `UTC TIME` (ID: 259) - UTC time of reported position.
- `GPS TIME` (ID: 258) - GPS time of reported position.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting in the configuration file:  
- True: the topic is published upon receiving SBP `UTC TIME` and `GPS TIME` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- False: the topic is published upon receiving SBP `GPS TIME` message. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Not used|
|`time_ref`|`GPS TIME`|GPS time in seconds since 1980-01-06. `sec` value is set to -1 if the GPS time is not available.|
|`source`|--|Text from `frame_name` in the config `params.yaml` file|

 
 ## Imu


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


