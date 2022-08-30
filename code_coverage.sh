#!/bin/bash

set -e

mkdir build
cd build
cmake -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage ..

make -j2 all
make -j2 test

cd ..
gcovr -j 2 --gcov-executable gcov --sonarqube ./build/code_coverage.xml --root . ./build

sonar-scanner -X -Dproject.settings=.github/workflows/sonar-project.properties \
                 -Dsonar.cfamily.cache.enabled=false \
                 -Dsonar.cfamily.compile-commands=./build/compile_commands.json \
                 -Dsonar.coverageReportPaths=./build/code_coverage.xml \
                 -Dsonar.organization=swift-nav \
                 -Dsonar.projectKey=swift-nav_swiftnav-ros2 \
                 -Dsonar.host.url="https://sonarcloud.io" \
                 -Dsonar.pullrequest.branch=sokhealy/sonarcloud \
                 -Dsonar.pullrequest.key=8
