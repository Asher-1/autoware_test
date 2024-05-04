#!/usr/bin/env bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --continue-on-error

# Only build demos
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-select autoware_quickstart_examples