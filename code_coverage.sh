#!/bin/bash

# arguments:
# 1 - number of jobs to use in parallel
# 2 - pull request branch name
# 3 - pull request number

set -e

mkdir -p build
cd build
cmake -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage ..

make -j$1 all
make -j$1 test

cd ..
gcovr -j $1 --gcov-executable gcov --sonarqube ./build/code_coverage.xml --root . ./build

sonar-scanner -X -Dproject.settings=.github/workflows/sonar-project.properties \
                 -Dsonar.cfamily.cache.enabled=false \
                 -Dsonar.cfamily.compile-commands=./build/compile_commands.json \
                 -Dsonar.coverageReportPaths=./build/code_coverage.xml \
                 -Dsonar.organization=swift-nav \
                 -Dsonar.projectKey=swift-nav_swiftnav-ros2 \
                 -Dsonar.host.url="https://sonarcloud.io" \
                 -Dsonar.pullrequest.branch=$2 \
                 -Dsonar.pullrequest.key=$3
