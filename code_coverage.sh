#!/bin/bash

# set -e

export PATH=$HOME/.sonar/build-wrapper-linux-x86:$PATH

mkdir build
cd build
export CFLAGS="--coverage"
cmake ..
build-wrapper-linux-x86-64 --out-dir build_wrapper_output_directory make -j6 all
make test
gcovr -j 8 --gcov-executable gcov --sonarqube ./code_coverage.xml --root .
