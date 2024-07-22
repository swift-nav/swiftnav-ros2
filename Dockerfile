FROM osrf/ros:humble-desktop

ARG SONAR_SCANNER_VERSION=6.1.0.4477

ARG DEBIAN_FRONTEND=noninteractive

ENV CC=gcc-11
ENV CXX=g++-11
ENV HOME /home/dockerdev

ARG UID=1000

RUN apt-get update && apt-get install --yes \
    apt-utils \
    build-essential \
    pkg-config \
    cmake \
    doxygen \
    check \
    clang-format-13 \
    libserialport-dev \
    ros-humble-gps-msgs

# Add a "dockerdev" user with sudo capabilities
# 1000 is the first user ID issued on Ubuntu; might
# be different for Mac users. Might need to add more.
# Create the user with the specified UID
RUN useradd -u $UID -m dockerdev && \
    # Set the user's shell to /bin/bash
    chsh -s /bin/bash dockerdev && \
    # Add the user to the sudo group
    usermod -aG sudo dockerdev && \
    # Add the user to sudoers with no password prompt for sudo commands
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers && \
    # Change ownership of the home directory to the new user
    chown -R dockerdev:dockerdev $HOME

USER dockerdev

WORKDIR $HOME/

RUN git clone --depth=1 --branch v4.11.0 --single-branch --recursive --jobs=4 https://github.com/swift-nav/libsbp.git && \
    mkdir -p libsbp/c/build &&  \
    cd libsbp/c/build && \
    cmake -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF ../ && \
    make -j4 && \
    sudo make install

# Install code coverage tool
RUN sudo apt-get -y install gcovr

# Download and set up sonar-scanner
RUN sudo apt-get -y install unzip && \
    mkdir -p $HOME/.sonar && \
    curl -sSLo $HOME/.sonar/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-${SONAR_SCANNER_VERSION}-linux-x64.zip && \
    unzip -o $HOME/.sonar/sonar-scanner.zip -d $HOME/.sonar/

ENV PATH="${PATH}:/home/dockerdev/.sonar/sonar-scanner-${SONAR_SCANNER_VERSION}-linux-x64/bin"

WORKDIR /mnt/workspace/src/swiftnav-ros2

RUN sudo chown -R dockerdev:dockerdev /mnt/workspace/
