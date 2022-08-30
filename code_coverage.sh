#!/bin/bash

# arguments:
# 1 - github token
# 2 - sonar token
# 3 - number of threads to use in parallel
# 4 - pull request branch name
# 5 - pull request number

# set -e

export GITHUB_TOKEN=$1
export SONAR_TOKEN=$2

mkdir -p build
cd build
cmake -DCMAKE_C_FLAGS=--coverage -DCMAKE_CXX_FLAGS=--coverage ..

make -j$3 all
make -j$3 test

cd ..
gcovr -j $3 --gcov-executable gcov --sonarqube ./build/code_coverage.xml --root . ./build

if [ -n "$4" ] && [ -n "$5" ]; then
    # pull request build
    sonar-scanner -X -Dproject.settings=.github/workflows/sonar-project.properties \
                     -Dsonar.cfamily.cache.enabled=false \
                     -Dsonar.cfamily.compile-commands=./build/compile_commands.json \
                     -Dsonar.coverageReportPaths=./build/code_coverage.xml \
                     -Dsonar.organization=swift-nav \
                     -Dsonar.projectKey=swift-nav_swiftnav-ros2 \
                     -Dsonar.host.url="https://sonarcloud.io" \
                     -Dsonar.pullrequest.branch=$4 \
                     -Dsonar.pullrequest.key=$5
else
    # master build
    sonar-scanner -X -Dproject.settings=.github/workflows/sonar-project.properties \
                     -Dsonar.cfamily.cache.enabled=false \
                     -Dsonar.cfamily.compile-commands=./build/compile_commands.json \
                     -Dsonar.coverageReportPaths=./build/code_coverage.xml \
                     -Dsonar.organization=swift-nav \
                     -Dsonar.projectKey=swift-nav_swiftnav-ros2 \
                     -Dsonar.host.url="https://sonarcloud.io" \
                     -Dsonar.branch.name=master
fi
