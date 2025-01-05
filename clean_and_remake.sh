#!/usr/bin/env bash

# Remove the 'devel' directory
rm -rf devel

# Remove the 'build' directory
rm -rf build

# Run catkin_make with the specified Python executable
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Debug

