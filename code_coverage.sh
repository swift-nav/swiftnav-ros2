#!/bin/bash

# set -e

export PATH=$HOME/.sonar/build-wrapper-linux-x86:$PATH
export PATH=$HOME/.sonar/sonar-scanner-4.7.0.2747-linux/bin:$PATH

mkdir build
cd build
cmake -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage ..
# build-wrapper-linux-x86-64 --out-dir build_wrapper_output_directory make -j6 all
make -j8 all
make -j8 test
# ./swiftnav_ros2_driver_test
cd ..
gcovr -j 8 --gcov-executable gcov --sonarqube ./build/code_coverage.xml --root . ./build

cat ./build/code_coverage.xml

sonar-scanner -X -Dproject.settings=.github/workflows/sonar-project.properties \
                 -Dsonar.cfamily.cache.enabled=false \
                 -Dsonar.cfamily.compile-commands=./build/compile_commands.json \
                 -Dsonar.coverageReportPaths=./build/code_coverage.xml \
                 -Dsonar.organization=swift-nav \
                 -Dsonar.projectKey=swift-nav_swiftnav-ros2 \
                 -Dsonar.host.url="https://sonarcloud.io" \
                 -Dsonar.pullrequest.branch=sokhealy/sonarcloud \
                 -Dsonar.pullrequest.key=8
                #  -Dsonar.cfamily.build-wrapper-output="./build/build_wrapper_output_directory" \
