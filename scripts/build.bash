#!/bin/bash

cd ../../
colcon build --packages-select guiniverse && \
source install/local_setup.bash 
