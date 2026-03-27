#!/bin/bash

cd ../../
colcon build --cmake-force-configure --packages-select guiniverse && \
source install/local_setup.bash 
