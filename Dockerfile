FROM osrf/ros:humble-desktop

ARG GITHUB_TOKEN_arg
ENV GITHUB_TOKEN=$GITHUB_TOKEN_arg

ARG SONAR_TOKEN_arg
ENV SONAR_TOKEN=$SONAR_TOKEN_arg

ARG SONAR_SCANNER_VERSION=4.7.0.2747

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
    libserialport-dev

# Add a "dockerdev" user with sudo capabilities
# 1000 is the first user ID issued on Ubuntu; might
# be different for Mac users. Might need to add more.
RUN \
     useradd -u ${UID} -ms /bin/bash -G sudo dockerdev \
  && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >>/etc/sudoers 

WORKDIR /home/dockerdev
RUN chown -R dockerdev:dockerdev /home/dockerdev
USER dockerdev

RUN git clone https://github.com/swift-nav/libsbp.git && cd libsbp && git checkout v4.4.0
WORKDIR /home/dockerdev/libsbp/c
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
RUN curl -sSLo $HOME/.sonar/sonar-scanner.zip https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.7.0.2747-linux.zip
RUN unzip -o $HOME/.sonar/sonar-scanner.zip -d $HOME/.sonar/
ENV PATH="${PATH}:/home/dockerdev/.sonar/sonar-scanner-4.7.0.2747-linux/bin"

WORKDIR /mnt/workspace/src/swiftnav-ros2
RUN sudo chown -R dockerdev:dockerdev /mnt/workspace/


#CMD ["make", "all"]
