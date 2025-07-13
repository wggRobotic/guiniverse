#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

cd ../../
colcon build --cmake-force-configure --packages-select guiniverse --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && \
source install/local_setup.bash 
