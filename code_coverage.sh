#!/bin/bash

# set -e

export BUILD_WRAPPER_DOWNLOAD_URL=https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip

curl -sSLo $HOME/.sonar/build-wrapper-linux-x86.zip $BUILD_WRAPPER_DOWNLOAD_URL
unzip -o $HOME/.sonar/build-wrapper-linux-x86.zip -d $HOME/.sonar/
echo "$HOME/.sonar/build-wrapper-linux-x86" >> $GITHUB_PATH

mkdir build
cd build
export CFLAGS="--coverage"
cmake ..
make test
gcovr -j 8 --gcov-executable gcov --sonarqube ./code_coverage.xml --root .
