# swiftnav-ros2
Swift Navigation's ROS2 SBP Driver for Piksi multi/Duro, PGM, STEP

## Table of contents
- [Setting the environment for Visual Studio Code](#setting-the-environment-for-visual-Studio-Code)
    - [Setup a Docker container that works with VSCode](#docker-container-for-vscode)
- [Compiling the ROS2 driver](#compiling-the-ros2-driver)
- [ROS2 driver configuration](#ros2-driver-configuration)
- [Adding a new SBP message to ROS2 topic translation](#adding-a-new-sbp-message-to-ros2-topic-translation)
- [Adding a new ROS2 topic to SBP message transaltion](#adding-a-new-ros2-topic-to-sbp-message-transaltion)

## Setting the environment for Visual Studio Code
Let's start creating the folder for the project:

```
mkdir swift
cd swift
```
### Docker container for VSCode
Once we have the folder for our project, we need to add a special folder in order for VSCode to automaticaly connect and use the docker conatiner.
```
mkdir .devcontainer
cd .devcontainer
```
Here we should create a couple of files. First the Dockerfile:
```
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

ENV CC=gcc-11
ENV CXX=g++-11

ARG UID=1000

RUN apt-get update && apt-get install --yes \
    build-essential \
    pkg-config \
    cmake \
    doxygen \
    check \
    clang-format-13 \
    clang-tidy \
    libserialport-dev \
    python3-pip

# Add a "dockerdev" user with sudo capabilities
# 1000 is the first user ID issued on Ubuntu; might
# be different for Mac users. Might need to add more.
RUN \
     useradd -u ${UID} -ms /bin/bash -G sudo dockerdev \
  && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >>/etc/sudoers

WORKDIR /home/dockerdev
RUN chown -R dockerdev:dockerdev /home/dockerdev
USER dockerdev

RUN git clone https://github.com/swift-nav/libsbp.git
WORKDIR /home/dockerdev/libsbp/c
RUN git submodule update --init
RUN mkdir build &&  \
    cd build && \
    cmake DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF ../ && \
    make && \
    sudo make install

WORKDIR /workspaces/swift
```
and then, the file named "devcontainer.json"; this file is the one that VSCode will use to do the magic

```
{
    "context": "../",
    "name": "swift-driver",
    "dockerFile": "Dockerfile",
    "runArgs": [
        "--security-opt", "seccomp=unconfined",
        "--name=swift-driver",
        "--volume=/dev:/dev",
        "--privileged",
        "--network=host",
        "--gpus", "all",
        "-e", "DISPLAY",
        "-e", "QT_GRAPHICSSYSTEM=native",
        "-e", "QT_X11_NO_MITSHM=1",
        "-v", "/tmp/.X11-unix:/tmp/.X11-unix"
    ],
    "extensions": [
		"ms-python.python",
		"ms-vscode.cpptools",
		"ms-iot.vscode-ros",
		"xaver.clang-format",
		"twxs.cmake",
		"ms-vscode.cmake-tools",
		"cschlosser.doxdocgen",
		"eamodio.gitlens",
		"ms-python.vscode-pylance",
		"himanoa.Python-autopep8",
		"shardulm94.trailing-spaces",
        "notskm.clang-tidy"
	]
}
```
Now we're ready. Let's go back to out project dir and start VSCode

```
cd ..
code .
```
Visual studio code should open and present you the following dialog:

![Reopen in container](docs/images/reopen_in_container.png)

Click "Reopen in container button" and then you will be asked to rebuild the container:

![Rebuild](docs/images/rebuild.png)

Click the "Rebuild" button and, after a while, you will be ready.

## Compiling the ROS2 driver

## ROS2 driver configuration

## Adding a new SBP message to ROS2 topic translation

## Adding a new ROS2 topic to SBP message transaltion
