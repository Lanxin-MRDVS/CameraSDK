#!/bin/bash
rm -r build/ install/ log/ 
colcon build
source install/setup.bash

