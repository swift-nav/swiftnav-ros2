#!/bin/bash

# set -e

mkdir build
cd build
export CFLAGS="--coverage"
cmake ..
make test
gcovr -j 8 --gcov-executable gcov --sonarqube ./code_coverage.xml --root .
