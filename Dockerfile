FROM osrf/ros:humble-desktop

ARG SONAR_SCANNER_VERSION=4.7.0.2747

ARG DEBIAN_FRONTEND=noninteractive

ENV CC=gcc-11
ENV CXX=g++-11
ENV HOME /home/dockerdev

ARG UID=1000

RUN apt-get update && apt-get install --yes \
    build-essential \
    pkg-config \
    cmake \
    doxygen \
    check \
    clang-format-13 \
    libserialport-dev

# Add a "dockerdev" user with sudo capabilities
# 1000 is the first user ID issued on Ubuntu; might
# be different for Mac users. Might need to add more.
RUN \
     useradd -u ${UID} -ms /bin/bash -G sudo dockerdev \
  && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >>/etc/sudoers 

RUN chown -R dockerdev:dockerdev $HOME/
USER dockerdev

RUN mkdir -p $HOME/dev_ws/src && cd $HOME/dev_ws/src && git clone https://github.com/swri-robotics/gps_umd && cd gps_umd && git checkout ros2-devel
WORKDIR $HOME/dev_ws/
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select gps_msgs

WORKDIR $HOME/
RUN git clone https://github.com/swift-nav/libsbp.git && cd libsbp && git checkout v4.4.0
WORKDIR $HOME/libsbp/c
RUN git submodule update --init
RUN mkdir build &&  \
    cd build && \
    cmake DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF ../ && \
    make && \
    sudo make install

# Install code coverage tool
RUN sudo apt-get -y install gcovr

# Download and set up sonar-scanner
RUN sudo apt-get -y install unzip
RUN mkdir -p $HOME/.sonar
RUN curl -sSLo $HOME/.sonar/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-${SONAR_SCANNER_VERSION}-linux.zip
RUN unzip -o $HOME/.sonar/sonar-scanner.zip -d $HOME/.sonar/
ENV PATH="${PATH}:/home/dockerdev/.sonar/sonar-scanner-${SONAR_SCANNER_VERSION}-linux/bin"

WORKDIR /mnt/workspace/src/swiftnav-ros2
RUN sudo chown -R dockerdev:dockerdev /mnt/workspace/

#CMD ["make", "all"]
