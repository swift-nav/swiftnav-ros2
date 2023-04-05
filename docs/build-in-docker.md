# Building Driver In Docker

## Step 1 (clone and build docker image)
  - Clone the repo, build Docker image, run docker image.
  ```
    git clone https://github.com/swift-nav/swiftnav-ros2.git
    cd swiftnav-ros2
    docker build -t swiftnav-ros2 .
    docker run -it -v <path to swiftnav-ros2 directory>:/mnt/workspace/src/swiftnav-ros2 swiftnav-ros2:latest /bin/bash
  ```

## Step 2 (edit configuration)
  - Edit configuration file, if required.
  ```
    nano config/params.yaml
  ```

## Step 3 (build)
  - Build driver inside docker image.
  ```
    cd /mnt/workspace/
    colcon build
  ```

## Step 4 (launch)
  - Launching the driver inside the docker image may require access to serial device or TCP ports inside the docker.
  ```
    source install/setup.bash
    ros2 launch swiftnav_ros2_driver start.py
  ```
