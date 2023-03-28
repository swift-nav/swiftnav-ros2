# **swiftnav-ros2**
ROS2 driver for Swift Navigation's GNSS/INS receivers and Starling Positioning Engine software.

# **Table of Contents**
- [Features](#features)
- [ROS Topics](#ros-topics)
- [Building Driver](#building-driver)
- [Building Driver Using Docker](#building-driver-using-docker)
- [Driver Configuration](#driver-configuration)
- [GNSS Receiver Configuration](#gnss-receiver-configuration)
- [Technical Support](#technical-support)

# Features
- Designed for ROS2 Humble but also works with Foxy. 
- Developed and tested on Ubuntu 22.04 (ROS2 Humble) and Ubuntu 20.04 (ROS2 Foxy) platforms.
- Supports Swift Navigation receivers and Starling Positioning Engine in Swift Binary Protocol (SBP).
- Ethernet, Serial and File inputs.
- SBP data logging.
- Publishes ROS2 standard and Swift Navigation proprietary topics.
- Configurable time stamps.
- Written in C++.

# ROS Topics
The driver receives Swift binary (SBP) messages and publishes the following ROS topics:
 - [`GpsFix`](#gpsfix)
 - [`NavSatFix`](#navsatfix)
 - [`TwistWithCovarianceStamped`](#twistwithcovariancestamped)
 - [`Baseline` (proprietary)](#baseline)
 - [`TimeReference`](#timereference)
 - [`Imu`](#imu)

## GpsFix

`gps_msgs/msg/GPSFix`

### SBP Messages Used
- `UTC TIME` (ID: 259, *required/optional*) - UTC time of reported position. Required when `timestamp_source_gnss` is `True`.
- `GPS TIME` (ID: 258, *required*) - GPS time of reported position.
- `POS LLH COV` (ID: 529, *required*) - GNSS position with covariance.
- `VEL NED COV` (ID: 530, *required*) - GNSS velocity with covariance.
- `ORIENT EULER` (ID: 545, *optional*) - GNSS/INS orientation with estimated errors.
- `DOPS` (ID: 520, *optional*) - GNSS DOP (Dilution Of Precision) data.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting flag in the configuration file:  
- `True`: The topic is published upon receiving SBP `UTC TIME`, `GPS TIME`, `POS LLH COV`, `VEL NED COV` and, if present, `ORIENT EULER` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- `False`: The topic is published upon receiving SBP `GPS TIME`, `POS LLH COV`, `VEL NED COV` and, if present, `ORIENT EULER` messages with the same TOW. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Text from `frame_name` in the config `params.yaml` file|
|`status.satellites_used`|`POS LLH COV`||
|`status.satellite_used_prn[]`|--|Not populated|
|`status.satellites_visible`<br>`status.satellite_visible_prn[]`<br>`status.satellite_visible_z[]`<br>`status.satellite_visible_azimuth[]`<br>`status.satellite_visible_snr[]`|--|Not populated|
|`status.status`|`POS LLH COV`|Dead Reckoning (DR) position is reported as `STATUS_FIX` (0)|
|`status.motion_source`|`VEL NED COV`||
|`status.orientation_source`|`POS LLH COV`||
|`status.position_source`|`POS LLH COV`||
|`latitude`<br>`longitude`<br>`altitude`<br>|`POS LLH COV`|Zeros when the fix is invalid. If position is valid altitude is always present (i.e. never NaN).|
|`track`|`VEL NED COV`<br>or<br>`ORIENT EULER`|If message is present and data valid, reports `yaw` from `ORIENT EULER`. If `yaw` is not valid reports computed Course Over Ground from `VEL NED COV` message. `VEL NED COV` updates `track` only if horizontal speed is above `track_update_min_speed_mps` set in the settings file. When the track becomes invalid the last valid track is reported.  |
|`speed`|`POS LLH COV`|Computed horizontal (2D) speed|
|`climb`|`POS LLH COV`||
|`pitch`<br>`roll`|`ORIENT EULER`||
|`dip`|--|Not populated|
|`time`|`GPS TIME`|GPS time in seconds since 1980-01-06 |
|`gdop`<br>`pdop`<br>`hdop`<br>`vdop`<br>`tdop`|`DOPS`||
|`err`<br>`err_horz`<br>`err_vert`|`POS LLH COV`||
|`err_track`|`VEL NED COV`<br>or<br>`ORIENT EULER`||
|`err_speed`<br>`err_climb`|`VEL NED COV`||
|`err_time`|--|Not populated|
|`err_pitch`<br>`err_roll`|`ORIENT EULER`||
|`err_dip`|--|Not populated|
|`position_covariance`<br>`position_covariance_type`|`POS LLH COV`|Covariance, if valid, is always `TYPE_KNOWN` (full matrix).|


## NavSatFix
 
`sensor_msgs/msg/NavSatFix`

### SBP Messages Used
- `UTC TIME` (ID: 259, *required/optional*) - UTC time of reported position. Required when `timestamp_source_gnss` is `True`.
- `POS LLH COV` (ID: 529, *required*) - GNSS position data with covariance.
- `MEASUREMENT STATE` (ID: 97, *optional*) - GNSS constellations data.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting flag in the configuration file:  
- `True`: the topic is published upon receiving SBP `UTC TIME` and `POS LLH COV` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- `False`: the topic is published upon receiving SBP `POS LLH COV` message. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Text from `frame_name` in the config `params.yaml` file|
|`status.status`|`POS LLH COV`|Dead Reckoning (DR) position is reported as `STATUS_FIX` (0)|
|`status.service`|`MEASUREMENT STATE`|GNSS constellations from the last `MEASUREMENT STATE` message. Reports zero when message is not present.|
|`latitude`<br>`longitude`<br>`altitude`<br>`position_covariance`<br>`position_covariance_type`|`POS LLH COV`|Zeros when the fix is invalid. If position is valid altitude is always present (i.e. never NaN). Covariance, if valid, is always `TYPE_KNOWN` (full matrix).|

 
 ## TwistWithCovarianceStamped
 
`geometry_msgs/msg/TwistWithCovarianceStamped`

### SBP Messages Used
- `UTC TIME` (ID: 259, *required/optional*) - UTC time of reported velocity. Required when `timestamp_source_gnss` is `True`.
- `VEL NED COV` (ID: 530, *required*) - GNSS velocity data with covariance.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting flag in the configuration file:  
- `True`: the topic is published upon receiving SBP `UTC TIME` and `VEL NED COV` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- `False`: the topic is published upon receiving SBP `VEL NED COV` message. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Text from `frame_name` in the config `params.yaml` file|
|`linear.x`<br>`linear.y`<br>`linear.z`|`VEL NED COV`|Conversion from NED frame:<br>`x` = `east`<br>`y` = `north`<br>`z` = `-down`<br>Zeros when velocity is invalid.|
|`angular.x`<br>`angular.y`<br>`angular.z`|--|Not populated. Always zero.|
|`covariance`|`VEL NED COV`|If velocity is valid, linear velocity covariance is full matrix. `covariance[0]` is set to -1 when linear velocity is invalid. `covariance[21]` is always -1.|

 
## Baseline

`swiftnav-ros2/msg/Baseline`   *Proprietary message*

### SBP Messages Used
- `UTC TIME` (ID: 259, *required/optional*) - UTC time of reported baseline. Required when `timestamp_source_gnss` is `True`.
- `BASELINE NED` (ID: 524, *required*) - RTK baseline NED vector.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting flag in the configuration file:  
- `True`: the topic is published upon receiving SBP `UTC TIME` and `BASELINE NED` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- `False`: the topic is published upon receiving SBP `BASELINE NED` message. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Text from `frame_name` in the config `params.yaml` file|
|`mode`|`BASELINE NED`|Solution mode:<br>`0` - Invalid<br>`3` - Float RTK<br>`4` - Fixed RTK|
|`satellites_used`|`BASELINE NED`|Number of satellites used in the solution|
|`baseline_n_m`<br>`baseline_e_m`<br>`baseline_d_m`|`BASELINE NED`|Baseline NED vectors in [m]. Zeros when invalid. Vectors origin is at the base location.|
|`baseline_err_h_m`|`BASELINE NED`|Estimated (95%) horizontal error of baseline in [m]. Zero when invalid.|
|`baseline_err_v_m`|`BASELINE NED`|Estimated (95%) vertical error of baseline in [m]. Zero when invalid.|
|`baseline_length_m`|`BASELINE NED`|Computed 3D baseline length. Zero when invalid.|
|`baseline_length_h_m`|`BASELINE NED`|Computed horizontal baseline length. Zero when invalid.|
|`baseline_orientation_valid`|`BASELINE NED`|`True` when baseline orientation (dir and dip) is valid. `False` when invalid.|
|`baseline_dir_deg`|`BASELINE NED`|Computed horizontal angle (bearing/heading) from base to rover in [degrees]. Valid only in RTK fixed mode. Range [0..360). Zero when invalid.|
|`baseline_dir_err_deg`|`BASELINE NED`|Estimated (95%) error of `baseline_dir_deg` in [degrees]. Range [0..180]. Zero when invalid.|
|`baseline_dip_deg`|`BASELINE NED`|Computed vertical angle from base to rover in [degrees]. Valid only in RTK fixed mode. Range [-90..90]. Zero when invalid.|
|`baseline_dip_err_deg`|`BASELINE NED`|Estimated (95%) error of `baseline_dip_deg` in [degrees]. Range [0..90]. Zero when invalid.|


## TimeReference
 
`sensor_msgs/msg/TimeReference`

### SBP Messages Used
- `UTC TIME` (ID: 259, *required/optional*) - UTC time. Required when `timestamp_source_gnss` is `True`.
- `GPS TIME` (ID: 258, *required*) - GPS time.

### Topic Publication
Topic publication depends on `timestamp_source_gnss` setting flag in the configuration file:  
- `True`: the topic is published upon receiving SBP `UTC TIME` and `GPS TIME` messages with the same TOW. The topic timestamp contains the UTC time reported by the GNSS receiver. If the UTC time is not available the current platform time is reported.
- `False`: the topic is published upon receiving SBP `GPS TIME` message. The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Not used|
|`time_ref`|`GPS TIME`|GPS time in seconds since 1980-01-06. `sec` value is set to -1 if the GPS time is not available.|
|`source`|--|Text from `frame_name` in the config `params.yaml` file|
 
 
## Imu

`sensor_msgs/msg/Imu`

### SBP Messages Used
- `UTC TIME` (ID: 259, *required/optional*) - UTC time. Required when `timestamp_source_gnss` is `True`.
- `GPS TIME` (ID: 258, *required*) - GPS time
- `GNSS TIME OFFSET` (ID: 65287, *required/optional*) - Offset of the IMU local time with respect to GNSS time. Required when the original IMU time source is a local time.
- `IMU AUX` (ID: 2305, *required*) - Auxiliary IMU data
- `IMU RAW` (ID: 2304, *required*) - Raw IMU data

### Topic Publication
Topic is published upon receiving `IMU RAW` SBP message.
Time stamp depends on `timestamp_source_gnss` setting flag in the configuration file:  
- `True`: The topic timestamp contains the UTC time of the measurement computed from `UTC TIME`, `GPS TIME`, `GNSS TIME OFFSET` and `IMU RAW` SBP messages depending on original IMU time stamping source. If the UTC time is not available the current platform time is reported.
- `False`: The topic timestamp contains the current platform time.

### Topic Fields
| ROS2 Message Field | SBP Message Data Source | Notes |
| :--- | :---: | :--- |
|`header.stamp`|`UTC TIME`<br>`GPS TIME`<br>`GNSS TIME OFFSET`|See Topic Publication for time stamping details|
|`header.frame_id`|--|Text from `frame_name` in the config `params.yaml` file|
|`orientation`<br>`orientation_covariance`|--|Not populated. Always zero. `orientation_covariance[0]` is always -1.|
|`angular_velocity`|`IMU RAW`<br>`IMU AUX`|Reported in sensor frame. Zeros when invalid.|
|`angular_velocity_covariance`|--|Not populated. `angular_velocity_covariance[0]` is set to -1 when angular velocity is not valid or when the time stamping source has changed|
|`linear_acceleration`|`IMU RAW`<br>`IMU AUX`|Reported in sensor frame. Zeros when invalid.|
|`linear_acceleration_covariance`|--|Not populated. `linear_acceleration_covariance[0]` is set to -1 when linear acceleration is not valid or when the time stamping source has changed |


# Building Driver

### Dependencies:
- `libsbp` - Swift Binary Protocol library
- `libserialport` - Serial Port communication library


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

# Building Driver Using Docker

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

# Driver Configuration
The driver configuration is stored in `/config/params.yaml` file and  provides the following configuration options:

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

# GNSS Receiver Configuration
TBD


# Technical Support
TBD
