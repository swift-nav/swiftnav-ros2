#!/bin/bash

# set -e

export PATH=$HOME/.sonar/build-wrapper-linux-x86:$PATH
export PATH=$HOME/.sonar/sonar-scanner-4.7.0.2747-linux/bin:$PATH

mkdir build
cd build
export CFLAGS="--coverage"
cmake ..
build-wrapper-linux-x86-64 --out-dir build_wrapper_output_directory make -j6 all
make test
gcovr -j 8 --gcov-executable gcov --sonarqube ./code_coverage.xml --root .

sonar-scanner -X -Dproject.settings=../.github/workflows/sonar-project.properties \
                 -Dsonar.cfamily.build-wrapper-output="build_wrapper_output_directory" \
                 -Dsonar.cfamily.cache.enabled=false \
                 -Dsonar.cfamily.compile-commands=./compile_commands.json \
                 -Dsonar.coverageReportPaths=./code_coverage.xml \
                 -Dsonar.organization=swift-nav \
                 -Dsonar.projectKey=swift-nav_swiftnav-ros2 \
                 -Dsonar.host.url="https://sonarcloud.io" \
                 -Dsonar.pullrequest.branch=sokhealy/sonarcloud \
                 -Dsonar.pullrequest.key=8
